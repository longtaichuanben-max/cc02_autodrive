import math
import csv
import os
import numpy as np
import pymap3d as pm
from scipy.interpolate import splprep, splev
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from gnss_ros_standardization.msg import GnssSolution

# このプロジェクトで自動運転に使う十分な精度とみなすStatus
_VALID_STATUSES = (GnssSolution.STATUS_FIX, GnssSolution.STATUS_FLOAT)


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.get_logger().info('Stanley Controller Node has been started!')

        # パラメータの宣言
        self.declare_parameter('wp_file', 'wp_position.csv')
        self.declare_parameter('path_spacing', 0.05)            # m: スプライン再サンプリング間隔（数センチ刻み）
        self.declare_parameter('speed_fix',   0.5)              # m/s: RTK-FIX時の速度
        self.declare_parameter('speed_float', 0.3)              # m/s: RTK-FLOAT時の速度
        self.declare_parameter('stanley_k',  0.8)               # 横偏差ゲイン（大きいほど経路に強く戻る）
        self.declare_parameter('k_soft',     0.5)               # m/s: 低速時の分母ソフトニング定数
        self.declare_parameter('max_steering_angle', math.radians(25.0))  # rad: ステアリング最大角（rc_car_driverの実測値25°に合わせる）
        self.declare_parameter('bootstrap_speed', 0.1)          # 方位を確定させるために、最初の数秒間はこの速度で走行する
        self.declare_parameter('min_speed_for_heading', 0.1)    # m/s: この速度以上でvel_enuのヘディングを信頼する
        self.declare_parameter('max_speed_mps', 2.0)            # m/s: 速度の安全上限（誤設定時の暴走防止）
        self.declare_parameter('gnss_timeout_s', 2.0)           # 秒: 基準局RTCM補正が1Hzのため、0.5秒で毎周期引っかかる
        self.declare_parameter('goal_tolerance', 0.3)           # m: 経路終端のこの距離以内でゴール（停止）とみなす
        self.declare_parameter('search_window', 200)            # 最近傍点の前方探索窓（点数）。経路の後戻り防止

        wp_file                    = self.get_parameter('wp_file').value
        self.path_spacing          = self.get_parameter('path_spacing').value
        self.speed_fix             = self.get_parameter('speed_fix').value
        self.speed_float           = self.get_parameter('speed_float').value
        self.stanley_k             = self.get_parameter('stanley_k').value
        self.k_soft                = self.get_parameter('k_soft').value
        self.max_steer             = self.get_parameter('max_steering_angle').value
        self.bootstrap_speed       = self.get_parameter('bootstrap_speed').value
        self.min_speed_for_heading = self.get_parameter('min_speed_for_heading').value
        self.max_speed             = self.get_parameter('max_speed_mps').value
        self.gnss_timeout_s        = self.get_parameter('gnss_timeout_s').value
        self.goal_tolerance        = self.get_parameter('goal_tolerance').value
        self.search_window         = int(self.get_parameter('search_window').value)

        # waypointファイルの読み込み
        self.wps_llh = self._load_waypoints_llh(wp_file)
        if not self.wps_llh:
            self.get_logger().error(
                f'Waypointを読み込めませんでした: {wp_file} '
                '-- ファイルパスと形式(WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）を確認してください'
            )
            raise SystemExit(1)
        self.get_logger().info(f'Waypoint {len(self.wps_llh)}点 読み込み完了: {wp_file}')

        # スプライン化したENU経路（原点確定後にセットされる）
        self.path_xy       = None  # 滑らかなカーブ上の点 (M, 2) numpy配列
        self.path_heading  = None  # 各点での接線方位 (M,) numpy配列 [rad]
        self.wp_path_idx   = None  # 元の各Waypointに最も近い経路点のindex（通過判定用）
        self.origin_ecef   = None  # GNSS ENU原点（ECEF）。最初の有効Fixで一度だけ確定

        self.nearest_idx     = 0   # 現在の最近傍経路点index（前方探索の起点）
        self.next_wp_announce = 0  # 次に通過announceする元Waypointのindex
        self.goal_reached    = False

        # 状態変数の初期化
        self.current_x      = None  # 現在地 ENU-X [m]
        self.current_y      = None  # 現在地 ENU-Y [m]
        self.current_speed  = 0.0   # 現在の車速 [m/s]（vel_enu由来）
        self.heading        = None  # 進行方向 [rad]（東=0, 北=π/2）
        self.current_status = 0     # GNSSステータス

        # 安全停止用
        self.last_gnss_time = self.get_clock().now()

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

    # 元の疎なWaypoint(ENU)をスプラインで数センチ刻みの滑らかなカーブに変換する
    def _build_spline_path(self, enu_waypoints: list):
        pts = np.array(enu_waypoints, dtype=float)

        # splprepは連続重複点でエラーになるため除去する
        keep = np.ones(len(pts), dtype=bool)
        keep[1:] = np.any(np.diff(pts, axis=0) != 0.0, axis=1)
        pts = pts[keep]

        n = len(pts)
        if n < 2:
            raise ValueError('スプライン生成には2点以上のユニークなWaypointが必要です')

        # 点数に応じてスプライン次数を決める（cubicは4点以上必要）
        k = min(3, n - 1)
        tck, _u = splprep([pts[:, 0], pts[:, 1]], s=0, k=k)

        # 一旦細かくサンプリングして全長(弧長)を求める
        uu = np.linspace(0.0, 1.0, 4000)
        sx, sy = splev(uu, tck)
        seg = np.hypot(np.diff(sx), np.diff(sy))
        cumlen = np.concatenate([[0.0], np.cumsum(seg)])
        total_len = float(cumlen[-1])

        # 弧長基準で path_spacing 間隔に再サンプリング
        n_samples = max(2, int(round(total_len / self.path_spacing)) + 1)
        target = np.linspace(0.0, total_len, n_samples)
        u_arc = np.interp(target, cumlen, uu)

        px, py = splev(u_arc, tck)
        dx, dy = splev(u_arc, tck, der=1)        # 接線ベクトル → 経路方位
        headings = np.arctan2(dy, dx)

        path_xy = np.column_stack([px, py])

        # 元の各Waypointに最も近い経路点index（通過announce用）
        wp_path_idx = []
        for wx, wy in enu_waypoints:
            d2 = (path_xy[:, 0] - wx) ** 2 + (path_xy[:, 1] - wy) ** 2
            wp_path_idx.append(int(np.argmin(d2)))

        return path_xy, headings, wp_path_idx, total_len, n_samples

    # GNSS原点確定とENU変換 + スプライン経路生成
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
        # （18点程度ではこの行数削減自体が主な利点で、速度差は誤差程度）
        llh_arr = np.array(self.wps_llh)
        e, n, _u = pm.geodetic2enu(
            llh_arr[:, 0], llh_arr[:, 1], llh_arr[:, 2],
            origin_lat, origin_lon, origin_alt
        )
        enu_waypoints = np.column_stack([e, n])

        try:
            path_xy, headings, wp_path_idx, total_len, n_samples = \
                self._build_spline_path(enu_waypoints)
        except Exception as exc:
            self.get_logger().error(f'スプライン経路の生成に失敗しました: {exc}')
            raise SystemExit(1)

        self.path_xy      = path_xy
        self.path_heading = headings
        self.wp_path_idx  = wp_path_idx

        self.get_logger().info(
            f'GNSS ENU原点確定 → Waypoint {len(enu_waypoints)}点を'
            f'スプライン化（全長={total_len:.1f}m, {n_samples}点 / {self.path_spacing*100:.0f}cm刻み）。'
            f'最初の目標 → X={path_xy[0,0]:.2f}m, Y={path_xy[0,1]:.2f}m'
        )
        return True

    # GNSSコールバック
    def _gnss_callback(self, msg: GnssSolution):
        # FIX/FLOAT以外（無効・SPP・SBAS・DGPS等）は精度不足として無視する
        if msg.status not in _VALID_STATUSES:
            return

        self.current_status = msg.status
        self.last_gnss_time = self.get_clock().now()

        # ENU原点がまだ確定していなければ、このFixで確定を試みる。
        # 変換した直後の1回はcontrolに進まず、次のFixから走行を開始する。
        if self.path_xy is None:
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

    # 車両に最も近いスプライン経路点のindexを前方探索で求める
    def _find_nearest_index(self) -> int:
        start = self.nearest_idx
        end = min(start + self.search_window, len(self.path_xy))
        seg = self.path_xy[start:end]
        d2 = (seg[:, 0] - self.current_x) ** 2 + (seg[:, 1] - self.current_y) ** 2
        idx = start + int(np.argmin(d2))
        self.nearest_idx = idx
        return idx

    # Stanley制御（スプライン経路追従）
    def _control(self):
        if self.current_x is None:
            self._publish_stop()
            return

        if self.goal_reached:
            self._publish_stop()
            return

        # 【Catch-22対策】ヘディング未確定時は停止せず、直進ブートストラップを行う。
        if self.heading is None:
            speed = max(0.0, min(self.bootstrap_speed, self.max_speed))
            self._publish_command(speed, 0.0)
            self.get_logger().info('ヘディング未確定 → 直進ブートストラップ中...')
            return

        # 滑らかなカーブ上で車両に最も近い点を探す
        idx = self._find_nearest_index()
        px, py = self.path_xy[idx]
        path_heading = float(self.path_heading[idx])

        # 元のWaypointを通過したらannounce（経路index通過ベースなのでGNSSノイズに強い）
        while (self.next_wp_announce < len(self.wp_path_idx)
               and idx >= self.wp_path_idx[self.next_wp_announce]):
            self.get_logger().info(f'★ WP[{self.next_wp_announce}] 通過！')
            self.next_wp_announce += 1

        # 経路終端付近に到達したらゴール（停止）
        dist_to_goal = math.hypot(
            self.path_xy[-1, 0] - self.current_x,
            self.path_xy[-1, 1] - self.current_y
        )
        if idx >= len(self.path_xy) - 1 and dist_to_goal <= self.goal_tolerance:
            self.get_logger().info(f'★★★ 経路終端に到達！（残り{dist_to_goal:.2f}m）→ 停止')
            self.goal_reached = True
            self._publish_stop()
            return

        # 方位誤差（経路接線方位 - 現在の進行方向、-π〜πに正規化）
        heading_error = path_heading - self.heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 横偏差（経路からの符号付き垂直距離）。正なら経路の右側にいることを意味する。
        cross_track_error = (
            math.sin(path_heading) * (self.current_x - px)
            - math.cos(path_heading) * (self.current_y - py)
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
            f'path[{idx}/{len(self.path_xy)-1}] '
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
            self.get_logger().warn(f'GNSSデータが{elapsed:.1f}秒途絶えています → 安全停止')
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
