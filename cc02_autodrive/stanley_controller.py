import math
import csv
import os
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


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.get_logger().info('Stanley Controller Node has been started!')

        # パラメータの宣言
        self.declare_parameter('wp_file', 'wp_position_basic.csv')
        self.declare_parameter('wp_radius', 1.0)                # m: この距離以内でWP到達とみなす
        self.declare_parameter('speed_fix',   0.8)              # m/s: RTK-FIX時の速度
        self.declare_parameter('speed_float', 0.3)              # m/s: RTK-FLOAT時の速度
        self.declare_parameter('stanley_k',  0.8)               # 横偏差ゲイン（大きいほど経路に強く戻る）
        self.declare_parameter('k_soft',     1.0)               # m/s: 低速時の分母ソフトニング定数
        self.declare_parameter('max_steering_angle', math.radians(25.0))  # rad: ステアリング最大角（rc_car_driverの実測値25°に合わせる）
        self.declare_parameter('bootstrap_speed', 0.1)          # 方位を確定させるために、最初の数秒間はこの速度で走行する
        self.declare_parameter('min_speed_for_heading', 0.1)    # m/s: この速度以上でvel_enuのヘディングを信頼する
        self.declare_parameter('max_speed_mps', 2.0)            # m/s: 速度の安全上限（誤設定時の暴走防止）
        self.declare_parameter('gnss_timeout_s', 2.0)           # 秒: 基準局RTCM補正が1Hzのため、0.5秒で毎周期引っかかる

        wp_file                    = self.get_parameter('wp_file').value
        self.waypoint_radius       = self.get_parameter('wp_radius').value
        self.speed_fix             = self.get_parameter('speed_fix').value
        self.speed_float           = self.get_parameter('speed_float').value
        self.stanley_k             = self.get_parameter('stanley_k').value
        self.k_soft                = self.get_parameter('k_soft').value
        self.max_steer             = self.get_parameter('max_steering_angle').value
        self.bootstrap_speed       = self.get_parameter('bootstrap_speed').value
        self.min_speed_for_heading = self.get_parameter('min_speed_for_heading').value
        self.max_speed             = self.get_parameter('max_speed_mps').value
        self.gnss_timeout_s        = self.get_parameter('gnss_timeout_s').value

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
        self.segment_start = None    # 現在追従中の経路（線分）の開始点 (x, y)。直前WP到達時に更新される
        self.origin_ecef = None      # GNSS ENU原点（ECEF）。最初の有効Fixで一度だけ確定

        # 状態変数の初期化
        self.current_x      = None  # 現在地 ENU-X [m]
        self.current_y      = None  # 現在地 ENU-Y [m]
        self.current_speed  = 0.0   # 現在の車速 [m/s]（vel_enu由来）
        self.heading        = None  # 進行方向 [rad]（東=0, 北=π/2）
        self.current_status = 0     # GNSSステータス（FIX/FLOAT以外も含め、受信した最新の値）
        self.fix_achieved   = False  # 起動後に一度でもFIXを取得したか（取得するまでは走行しない）

        # 安全停止用
        self.last_gnss_time = self.get_clock().now()
        self.last_pos_enu_cov = [0.0] * 9  # ログ表示用（直近メッセージのpos_enu_cov）

        # Publisher/Subscriber/Timer
        self.cmd_pub  = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.gnss_sub = self.create_subscription(GnssSolution, '/gnss/solution', self._gnss_callback, 10)
        self.create_timer(0.1, self._safety_check)

        self.get_logger().info('stanley_controller 起動完了（GNSS ENU原点確定待ち）')

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

        self.get_logger().info(
            f'GNSS ENU原点確定 → Waypoint {len(self.waypoints)}点を変換完了（直線区間で接続）。'
            f'最初の目標 → X={self.waypoints[0,0]:.2f}m, Y={self.waypoints[0,1]:.2f}m'
        )
        return True

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
        ve, vn = msg.vel_enu.x, msg.vel_enu.y
        self.current_speed = math.hypot(ve, vn)
        if self.current_speed >= self.min_speed_for_heading:
            self.heading = math.atan2(vn, ve)

        status_str = 'FIX' if self.current_status == GnssSolution.STATUS_FIX else 'FLOAT'
        self.get_logger().info(
            f'GNSS: Status={status_str}  X={self.current_x:.2f}m  Y={self.current_y:.2f}m  '
            f'speed={self.current_speed:.2f}m/s'
        )

        self._control()

    # Stanley制御（Waypoint間は直線で接続）
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

        # 現在追従している経路（線分）の開始点。最初は現在地から開始する。
        if self.segment_start is None:
            self.segment_start = (self.current_x, self.current_y)

        tx, ty = self.waypoints[self.waypoint_index]

        # Waypoint到達判定（wp_radius以内に入ったら次のWPへ）
        dist_to_wp = math.hypot(tx - self.current_x, ty - self.current_y)
        if dist_to_wp <= self.waypoint_radius:
            self.get_logger().info(
                f'★ WP[{self.waypoint_index}] 通過！ (到達距離={dist_to_wp:.2f}m)'
            )
            # 到達したWPが次の経路区間の開始点になる
            self.segment_start = (tx, ty)
            self.waypoint_index += 1

            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info('★★★ 1周完了！ 停止せずWP[1]から再周回します')
                self.waypoint_index = 1

            tx, ty = self.waypoints[self.waypoint_index]
            self.get_logger().info(
                f'次の目標 → WP[{self.waypoint_index}] X={tx:.2f}m, Y={ty:.2f}m'
            )

        sx, sy = self.segment_start

        # 経路方位（区間開始点 → 目標Waypoint の角度）
        path_heading = math.atan2(ty - sy, tx - sx)

        # 方位誤差（経路方位 - 現在の進行方向、-π〜πに正規化）
        heading_error = path_heading - self.heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 横偏差（経路からの符号付き垂直距離）。正なら経路の右側にいることを意味する。
        cross_track_error = (
            math.sin(path_heading) * (self.current_x - sx)
            - math.cos(path_heading) * (self.current_y - sy)
        )

        # Stanley則: δ = heading_error + atan2(k * 横偏差, k_soft + 速度)
        cross_track_term = math.atan2(
            self.stanley_k * cross_track_error,
            self.k_soft + self.current_speed
        )
        steer = heading_error + cross_track_term
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # 速度: FIX(=1)は高速、FLOAT(=2)は低速（安全上限でクランプ）
        speed = self.speed_fix if self.current_status == GnssSolution.STATUS_FIX else self.speed_float
        speed = max(0.0, min(self.max_speed, speed))

        self._publish_command(speed, steer)

        self.get_logger().debug(
            f'WP[{self.waypoint_index}] '
            f'方位誤差={math.degrees(heading_error):+.1f}°  '
            f'横偏差={cross_track_error:+.2f}m  '
            f'ステア={math.degrees(steer):+.1f}°  '
            f'速度={speed:.1f}m/s  '
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


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
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
