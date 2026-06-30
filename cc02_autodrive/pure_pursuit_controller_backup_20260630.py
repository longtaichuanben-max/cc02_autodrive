import math
import csv
import os
import time
import datetime
import numpy as np
import pymap3d as pm
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from gnss_ros_standardization.msg import GnssSolution

# このプロジェクトで自動運転に使う十分な精度とみなすStatus
_VALID_STATUSES = (GnssSolution.STATUS_FIX, GnssSolution.STATUS_FLOAT)

# ログ表示用のstatus名（GnssSolution.msgのSTATUS_*定数に対応）
_STATUS_NAMES = {
    GnssSolution.STATUS_NONE:   'NONE/無効',
    GnssSolution.STATUS_FIX:    'FIX',
    GnssSolution.STATUS_FLOAT:  'FLOAT',
    GnssSolution.STATUS_SBAS:   'SBAS',
    GnssSolution.STATUS_DGPS:   'DGPS',
    GnssSolution.STATUS_SINGLE: 'SINGLE',
    GnssSolution.STATUS_PPP:    'PPP',
    GnssSolution.STATUS_EKF:    'EKF',
}


def _format_status_info(status: int, pos_enu_cov) -> str:
    name = _STATUS_NAMES.get(status, f'unknown({status})')
    h_var = pos_enu_cov[0] if len(pos_enu_cov) > 0 else 0.0
    acc_str = f'{math.sqrt(h_var):.1f}m' if h_var > 0.0 else 'n/a'
    return f'status={status}({name}) 水平精度(目安)≈{acc_str}'


# チューニング評価用ログのCSVヘッダー（後からClaude等にデータ分析させるための列）
_TUNING_LOG_HEADER = ['time', 'cross_track_error', 'alpha_deg', 'steer_deg', 'lookahead_dist', 'speed']


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info('Pure Pursuit Controller Node has been started!')

        # パラメータの宣言
        self.declare_parameter('wp_file', 'wp_position_basic.csv')
        self.declare_parameter('wp_radius', 0.6)                # m: この距離以内でWP到達とみなす
        self.declare_parameter('speed_fix',   0.95)             # m/s: RTK-FIX時の速度
                                                                 # 2026-06-29: 0.8->0.95 (デッドゾーン補償)
                                                                 # 2026-06-30: 0.95->1.2 試したが不安定(±0.9m CTE), 0.95に戻してから
                                                                 #             0.1刻みで増加: 0.95->1.05 (run10テスト)
        self.declare_parameter('speed_float', 0.3)              # m/s: RTK-FLOAT時の速度
        self.declare_parameter('wheelbase_m', 0.267)            # m: シャーシの前輪軸-後輪軸間の距離（実測値267mm）
        self.declare_parameter('lookahead_min', 1.0)            # m: 最低ルックアヘッド距離（低速時）
        self.declare_parameter('lookahead_gain', 0.5)           # s: ルックアヘッド距離の速度依存ゲイン（Ld = min + gain * speed）
        self.declare_parameter('max_steering_angle', math.radians(25.0))  # rad: ステアリング最大角（rc_car_driverの実測値25°に合わせる）
        self.declare_parameter('bootstrap_speed', 0.3)          # 方位を確定させるために、最初の数秒間はこの速度で走行する
        self.declare_parameter('min_speed_for_heading', 0.05)    # m/s: この速度以上でvel_enuのヘディングを信頼する
        self.declare_parameter('heading_smoothing_w', 0.15)     # ヘディング推定のEMA平滑化係数(0-1)。大きいほど反応が速いがノイズが残る
                                                                 # 2026-06-29: 0.3に上げて検証した結果、振動が悪化（周期が速くなり衝突2回）したため0.15に戻す。
                                                                 # EMA遅れが不安定化の原因ではなく、むしろ減衰として機能していた可能性が高い。
        self.declare_parameter('max_speed_mps', 2.0)            # m/s: 速度の安全上限（誤設定時の暴走防止）
        self.declare_parameter('gnss_timeout_s', 2.0)           # 秒: 基準局RTCM補正が1Hzのため、0.5秒で毎周期引っかかる
        self.declare_parameter('corner_angle_threshold_deg', 45.0)  # deg: 進行方向の変化がこの角度以上のWPを「急角コーナー」とみなす
        self.declare_parameter('corner_slowdown_speed', 0.5)        # m/s: 急角コーナーに接近中の速度
        self.declare_parameter('corner_slowdown_dist', 1.5)         # m: 急角コーナーWPまでこの距離以内になったら減速を開始する

        # チューニング評価用ログファイル（起動ごとにタイムスタンプ付きの新規ファイルを作成、上書きしない）
        default_tuning_log = 'pure_pursuit_log_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
        self.declare_parameter('tuning_log_file', default_tuning_log)

        wp_file                    = self.get_parameter('wp_file').value
        self.waypoint_radius       = self.get_parameter('wp_radius').value
        self.speed_fix             = self.get_parameter('speed_fix').value
        self.speed_float           = self.get_parameter('speed_float').value
        self.wheelbase             = self.get_parameter('wheelbase_m').value
        self.lookahead_min         = self.get_parameter('lookahead_min').value
        self.lookahead_gain        = self.get_parameter('lookahead_gain').value
        self.max_steer             = self.get_parameter('max_steering_angle').value
        self.bootstrap_speed       = self.get_parameter('bootstrap_speed').value
        self.min_speed_for_heading = self.get_parameter('min_speed_for_heading').value
        self.heading_smoothing_w   = self.get_parameter('heading_smoothing_w').value
        self.max_speed             = self.get_parameter('max_speed_mps').value
        self.gnss_timeout_s        = self.get_parameter('gnss_timeout_s').value
        self.corner_angle_threshold_deg = self.get_parameter('corner_angle_threshold_deg').value
        self.corner_slowdown_speed      = self.get_parameter('corner_slowdown_speed').value
        self.corner_slowdown_dist       = self.get_parameter('corner_slowdown_dist').value
        tuning_log_file            = self.get_parameter('tuning_log_file').value

        # チューニング評価用ログファイルを開く（毎起動ごとに新規ファイル、追記しながらflush）
        self._tuning_csv_file = open(tuning_log_file, 'w', newline='')
        self._tuning_writer = csv.writer(self._tuning_csv_file)
        self._tuning_writer.writerow(_TUNING_LOG_HEADER)
        self._tuning_csv_file.flush()
        self._tuning_n_logged = 0
        self.get_logger().info(f'チューニング評価ログ記録先: {os.path.abspath(tuning_log_file)}')

        # waypointファイルの読み込み
        self.wps_llh = self._load_waypoints_llh(wp_file)
        if not self.wps_llh:
            self.get_logger().error(
                f'Waypointを読み込めませんでした: {wp_file} '
                '-- ファイルパスと形式(WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）を確認してください'
            )
            raise SystemExit(1)
        self.get_logger().info(f'Waypoint {len(self.wps_llh)}点 読み込み完了: {wp_file}')

        self.waypoints = None  # ENU変換後の(x, y)リスト（直線で接続）。原点確定後にセットされる
        self.waypoint_index = 0
        self.origin_ecef = None      # GNSS ENU原点（ECEF）。最初の有効Fixで一度だけ確定

        # 状態変数の初期化
        self.current_x      = None  # 現在地 ENU-X [m]
        self.current_y      = None  # 現在地 ENU-Y [m]
        self.current_speed  = 0.0   # 現在の車速 [m/s]（vel_enu由来）
        self.heading        = None  # 進行方向 [rad]（東=0, 北=π/2）。EMA平滑化後のve/vnから算出
        self._ve_filtered   = 0.0   # ヘディングEMA平滑化用の内部状態（東方向速度）
        self._vn_filtered   = 0.0   # ヘディングEMA平滑化用の内部状態（北方向速度）
        self.current_status = 0     # GNSSステータス（FIX/FLOAT以外も含め、受信した最新の値）
        self.fix_achieved   = False  # 起動後に一度でもFIXを取得したか（取得するまでは走行しない）

        # 競技採点用（ロボットカーコンテスト2026: ゲート通過+10点、周回+50点）
        self.score = 0

        # 安全停止用
        self.last_gnss_time = self.get_clock().now()
        self.last_pos_enu_cov = [0.0] * 9  # ログ表示用（直近メッセージのpos_enu_cov）

        # Publisher/Subscriber/Timer
        self.cmd_pub  = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.gnss_sub = self.create_subscription(GnssSolution, '/gnss/solution', self._gnss_callback, 10)
        self.create_timer(0.1, self._safety_check)

        self.get_logger().info('pure_pursuit_controller 起動完了（GNSS ENU原点確定待ち）')

    # Waypointファイル読み込み（緯度経度のみ、ENU変換は行わない）
    def _load_waypoints_llh(self, filepath: str) -> list:
        waypoints = []
        if not os.path.exists(filepath):
            self.get_logger().error(f'ファイルが存在しません: {filepath}')
            return waypoints
        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)
                for i, row in enumerate(reader):
                    lat = float(row['Latitude(deg)'])
                    lon = float(row['Longitude(deg)'])
                    height = float(row['Ellipsoidal Height(m)'])
                    waypoints.append((lat, lon, height))
                    self.get_logger().debug(f'  WP[{i}]: lat={lat:.8f}, lon={lon:.8f}, h={height:.2f}')
        except Exception as e:
            self.get_logger().error(f'Waypoint読み込みエラー: {e}')
            return []
        return waypoints

    # GNSS原点確定とENU変換（スプラインは使わず、Waypoint間は直線で接続する）
    def _resolve_origin_and_convert(self, msg: GnssSolution) -> bool:
        ox = msg.pos_enu_org_ecef.x
        oy = msg.pos_enu_org_ecef.y
        oz = msg.pos_enu_org_ecef.z

        # gnss_ros_standardization側の仕様: 原点（基準局位置）未確定時は(0,0,0)が入る
        if ox == 0.0 and oy == 0.0 and oz == 0.0:
            return False

        self.origin_ecef = (ox, oy, oz)
        origin_lat, origin_lon, origin_alt = pm.ecef2geodetic(ox, oy, oz)

        # pymap3dはベクトル入力に対応しているため、Waypointごとのforループは不要
        llh_arr = np.array(self.wps_llh)
        e, n, _u = pm.geodetic2enu(
            llh_arr[:, 0], llh_arr[:, 1], llh_arr[:, 2],
            origin_lat, origin_lon, origin_alt
        )
        self.waypoints = np.column_stack([e, n])
        self._compute_corner_angles()

        self.get_logger().info(
            f'GNSS ENU原点確定 → Waypoint {len(self.waypoints)}点を変換完了（直線区間で接続）。'
            f'最初の目標 → X={self.waypoints[0,0]:.2f}m, Y={self.waypoints[0,1]:.2f}m'
        )
        return True

    # 各WPでの進行方向の変化角(度)を事前計算する。急角コーナー手前での減速判定に使う。
    # WP[i-1]→WP[i]の方向とWP[i]→WP[i+1]の方向の差の絶対値。両側の区間がないWP[0]・WP[末尾]は0とする。
    def _compute_corner_angles(self):
        n = len(self.waypoints)
        self.corner_turn_deg = [0.0] * n
        for i in range(1, n - 1):
            x0, y0 = self.waypoints[i - 1]
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]
            in_heading = math.atan2(y1 - y0, x1 - x0)
            out_heading = math.atan2(y2 - y1, x2 - x1)
            diff = math.atan2(math.sin(out_heading - in_heading), math.cos(out_heading - in_heading))
            self.corner_turn_deg[i] = abs(math.degrees(diff))
            if self.corner_turn_deg[i] >= self.corner_angle_threshold_deg:
                self.get_logger().info(
                    f'急角コーナーを検出: WP[{i}] 方向変化={self.corner_turn_deg[i]:.0f}° '
                    f'→ 接近時({self.corner_slowdown_dist:.1f}m以内)に{self.corner_slowdown_speed:.1f}m/sまで減速します'
                )

    # GNSSコールバック
    def _gnss_callback(self, msg: GnssSolution):
        # メッセージは受信できている前提で、status/精度はFIX/FLOAT判定の前に必ず更新する
        # （_safety_checkでの「受信なし」と「精度不足」を区別するため）
        self.current_status = msg.status
        self.last_gnss_time = self.get_clock().now()
        self.last_pos_enu_cov = msg.pos_enu_cov

        # FIX/FLOAT以外（無効・SPP・SBAS・DGPS等）は精度不足として無視する
        if msg.status not in _VALID_STATUSES:
            return

        if self.current_status == GnssSolution.STATUS_FIX and not self.fix_achieved:
            self.fix_achieved = True
            self.get_logger().info('★ 起動後初回のFIXを達成 → 走行を開始します')

        # ENU原点がまだ確定していなければ、このFixで確定を試みる。
        # 変換した直後の1回はcontrolに進まず、次のFixから走行を開始する。
        if self.waypoints is None:
            self._resolve_origin_and_convert(msg)
            return

        self.current_x = msg.pos_enu.x
        self.current_y = msg.pos_enu.y

        # ヘディング = 速度ベクトルのCourse over Ground（vel_enu由来）。
        # GNSS速度ノイズの影響を抑えるため、角度ではなくve/vnベクトル成分にEMA平滑化を
        # かけてからatan2で角度に変換する（角度を直接平均すると0°/360°の境界で破綻するため）。
        ve, vn = msg.vel_enu.x, msg.vel_enu.y
        self.current_speed = math.hypot(ve, vn)
        if self.current_speed >= self.min_speed_for_heading:
            if self.heading is None:
                # 初回はEMAの初期値が無いので測定値をそのまま使う
                self._ve_filtered, self._vn_filtered = ve, vn
            else:
                w = self.heading_smoothing_w
                self._ve_filtered = (1 - w) * self._ve_filtered + w * ve
                self._vn_filtered = (1 - w) * self._vn_filtered + w * vn
            self.heading = math.atan2(self._vn_filtered, self._ve_filtered)

        status_str = 'FIX' if self.current_status == GnssSolution.STATUS_FIX else 'FLOAT'
        self.get_logger().info(
            f'GNSS: Status={status_str}  X={self.current_x:.2f}m  Y={self.current_y:.2f}m  '
            f'speed={self.current_speed:.2f}m/s'
        )

        self._control()

    # 現在のwaypoint_indexから先の直線区間を辿り、ルックアヘッド距離だけ
    # 前方にある点を線形補間で求める（スプラインではなく直線区間で接続）。
    def _lookahead_target(self, lookahead_dist: float):
        idx = self.waypoint_index
        px, py = self.current_x, self.current_y
        remaining = lookahead_dist
        n = len(self.waypoints)
        while idx < n:
            wx, wy = self.waypoints[idx]
            seg_len = math.hypot(wx - px, wy - py)
            if seg_len >= remaining:
                ratio = remaining / seg_len if seg_len > 1e-6 else 0.0
                return px + (wx - px) * ratio, py + (wy - py) * ratio
            remaining -= seg_len
            px, py = wx, wy
            idx += 1
        return float(self.waypoints[-1][0]), float(self.waypoints[-1][1])

    # Pure Pursuit制御（Waypoint間は直線で接続）
    def _control(self):
        if self.current_x is None:
            self._publish_stop()
            return

        # 起動後、一度もFIXを取得していない間は走行しない（FLOATだけでは動かさない）
        if not self.fix_achieved:
            self._publish_stop()
            return

        # 【Catch-22対策】ヘディング未確定時は停止せず、直進ブートストラップを行う。
        if self.heading is None:
            speed = max(0.0, min(self.bootstrap_speed, self.max_speed))
            self._publish_command(speed, 0.0)
            self.get_logger().info('ヘディング未確定 → 直進ブートストラップ中...')
            return

        tx0, ty0 = self.waypoints[self.waypoint_index]

        # Waypoint到達判定（wp_radius以内に入ったら次のWPへ）
        dist_to_wp = math.hypot(tx0 - self.current_x, ty0 - self.current_y)
        if dist_to_wp <= self.waypoint_radius:
            self.score += 10
            self.get_logger().info(
                f'★ WP[{self.waypoint_index}] 通過！ (到達距離={dist_to_wp:.2f}m) '
                f'ゲート通過 +10点 → 合計{self.score}点'
            )
            self.waypoint_index += 1

            if self.waypoint_index >= len(self.waypoints):
                self.score += 50
                self.get_logger().info(
                    f'★★★ 1周完了！ 周回ボーナス +50点 → 合計{self.score}点 '
                    f'停止せずWP[1]から再周回します'
                )
                self.waypoint_index = 1

            tx0, ty0 = self.waypoints[self.waypoint_index]
            self.get_logger().info(
                f'次の目標 → WP[{self.waypoint_index}] X={tx0:.2f}m, Y={ty0:.2f}m'
            )

        # 横偏差(cross_track_error)計算: 直前WP→現在の目標WPを結ぶ線分への符号付き垂直距離。
        # 正なら経路の右側にいることを意味する（stanley_controllerと同じ符号規約）。
        prev_idx = self.waypoint_index - 1
        if prev_idx >= 0:
            sx, sy = self.waypoints[prev_idx]
        else:
            sx, sy = self.current_x, self.current_y
        seg_dx, seg_dy = tx0 - sx, ty0 - sy
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len > 1e-6:
            cross_track_error = ((self.current_x - sx) * seg_dy - (self.current_y - sy) * seg_dx) / seg_len
        else:
            cross_track_error = 0.0

        # 急角コーナー(WP[waypoint_index]の方向変化が大きい)に接近中かどうかを判定する。
        # ルックアヘッドがwp_radius(到達判定半径)より大きいと、まだ正式に到達していないのに
        # ルックアヘッド点だけ次の区間(方向が大きく違う)へ先回りしてしまい、「まだ未到達」と
        # 「先の区間へ向け」の指令が位置のわずかな変化で入れ替わって暴れる(往復振動)。
        # これを防ぐため、コーナー接近中はルックアヘッド距離をwp_radius以下に制限する
        # （wp_radius自体は広げない＝内側のショートカット量を増やさない）。
        dist_to_target = math.hypot(tx0 - self.current_x, ty0 - self.current_y)
        turn_deg = self.corner_turn_deg[self.waypoint_index] if self.waypoint_index < len(self.corner_turn_deg) else 0.0
        near_sharp_corner = turn_deg >= self.corner_angle_threshold_deg and dist_to_target <= self.corner_slowdown_dist

        # ルックアヘッド距離（速度が速いほど遠くを見る）
        lookahead_dist = self.lookahead_min + self.lookahead_gain * self.current_speed
        if near_sharp_corner:
            lookahead_dist = min(lookahead_dist, self.waypoint_radius)
        tx, ty = self._lookahead_target(lookahead_dist)

        # 車両から目標点までの実際の距離と方位
        dx, dy = tx - self.current_x, ty - self.current_y
        ld_actual = math.hypot(dx, dy)
        target_bearing = math.atan2(dy, dx)

        # alpha = 目標点方位と現在のヘディングの差（-π〜πに正規化）
        alpha = target_bearing - self.heading
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Pure Pursuit則: δ = atan2(2 * L * sin(alpha), Ld)
        if ld_actual < 1e-3:
            steer = 0.0
        else:
            steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld_actual)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # 速度: FIX(=1)は高速、FLOAT(=2)は低速（安全上限でクランプ）
        speed = self.speed_fix if self.current_status == GnssSolution.STATUS_FIX else self.speed_float
        speed = max(0.0, min(self.max_speed, speed))

        # 急角コーナーに接近中は速度も落とす（上のルックアヘッド制限と合わせて、コーナー手前での
        # ステアリング暴れを抑える）。
        if near_sharp_corner:
            speed = min(speed, self.corner_slowdown_speed)

        self._publish_command(speed, steer)

        # チューニング評価用ログ（CSV追記 + ターミナル表示）。後からClaude等にCSVを解析させて
        # cross_track_errorの最大値発生時刻やステアリングのハンチング有無を調べられるようにする。
        alpha_deg = math.degrees(alpha)
        steer_deg = math.degrees(steer)
        self._tuning_writer.writerow([
            f'{time.time():.3f}',
            f'{cross_track_error:.3f}',
            f'{alpha_deg:.2f}',
            f'{steer_deg:.2f}',
            f'{lookahead_dist:.3f}',
            f'{self.current_speed:.3f}',
        ])
        self._tuning_csv_file.flush()
        self._tuning_n_logged += 1

        self.get_logger().debug(
            f'WP[{self.waypoint_index}] Ld={ld_actual:.2f}m alpha={alpha_deg:+.1f}° '
            f'ステア={steer_deg:+.1f}°  横偏差={cross_track_error:+.2f}m  速度={speed:.1f}m/s  '
            f'Status={"FIX" if self.current_status == GnssSolution.STATUS_FIX else "FLOAT"}'
        )

    # 安全停止チェック
    def _safety_check(self):
        elapsed = (self.get_clock().now() - self.last_gnss_time).nanoseconds / 1e9
        if elapsed > self.gnss_timeout_s:
            # last_gnss_timeは受信したメッセージのstatusを問わず更新されるため、
            # ここに来るのは本当にメッセージそのものが届いていない場合のみ
            self.get_logger().warn(f'GNSSデータ受信なし（{elapsed:.1f}秒）→ 安全停止')
            self._publish_stop()
        elif self.current_status not in _VALID_STATUSES:
            self.get_logger().warn(
                f'走行に必要な精度(FIX/FLOAT)に未到達: '
                f'{_format_status_info(self.current_status, self.last_pos_enu_cov)} → 待機中',
                throttle_duration_sec=1.0,
            )
            self._publish_stop()

    # コマンド送信
    def _publish_command(self, speed: float, steering_angle: float):
        msg = AckermannDriveStamped()
        msg.drive.speed          = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish_command(0.0, 0.0)

    def destroy_node(self):
        self.get_logger().info(f'チューニング評価ログ終了 → 合計{self._tuning_n_logged}行を記録しました')
        self._tuning_csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
