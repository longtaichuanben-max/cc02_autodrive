import math
import csv
import os
import time
import numpy as np
import pymap3d as pm
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from gnss_ros_standardization.msg import GnssSolution

_VALID_STATUSES = (GnssSolution.STATUS_FIX, GnssSolution.STATUS_FLOAT)

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

_TUNING_LOG_HEADER = [
    'time', 'cross_track_error', 'heading_error_deg', 'cross_track_term_deg', 'steer_deg', 'speed',
]


def _format_status_info(status: int, pos_enu_cov) -> str:
    name = _STATUS_NAMES.get(status, f'unknown({status})')
    h_var = pos_enu_cov[0] if len(pos_enu_cov) > 0 else 0.0
    acc_str = f'{math.sqrt(h_var):.1f}m' if h_var > 0.0 else 'n/a'
    return f'status={status}({name}) 水平精度(目安)≈{acc_str}'


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.get_logger().info('Stanley Controller Node has been started!')

        self.declare_parameter('wp_file', 'wp_position_basic.csv')
        self.declare_parameter('wp_radius', 0.7)
        self.declare_parameter('wp_radii', "")           # 個別半径 例:"1:0.5,3:2.0" (arc挿入後インデックス)
        self.declare_parameter('speed_fix', 1.5)         # m/s: RTK-FIX時の速度
        self.declare_parameter('speed_float', 1.0)       # m/s: RTK-FLOAT時の速度
        self.declare_parameter('stanley_k', 0.5)         # 横偏差ゲイン
        self.declare_parameter('k_soft', 1.0)            # m/s: 低速時ソフトニング定数
        self.declare_parameter('max_steering_angle', math.radians(25.0))
        self.declare_parameter('wheelbase_m', 0.267)     # m: 前後軸間距離（コーナー円弧計算に使用）
        self.declare_parameter('bootstrap_speed', 0.3)
        self.declare_parameter('min_speed_for_heading', 0.05)
        self.declare_parameter('heading_smoothing_w', 0.35)
        self.declare_parameter('max_speed_mps', 4.0)
        self.declare_parameter('gnss_timeout_s', 2.0)
        self.declare_parameter('corner_wp_indices', '0,2,6,8')
        self.declare_parameter('wp_skip_indices', "3,7")
        self.declare_parameter('corner_slowdown_speed', 1.0)
        self.declare_parameter('corner_slowdown_dist', 8.0)
        default_tuning_log = os.path.join(
            os.path.expanduser('~'), 'ros2_ws', 'gnss_logs', 'stanley_log_latest.csv'
        )
        self.declare_parameter('tuning_log_file', default_tuning_log)

        wp_file                    = self.get_parameter('wp_file').value
        self.waypoint_radius       = self.get_parameter('wp_radius').value
        _wp_radii_str = self.get_parameter('wp_radii').value.strip()
        self._wp_radius_map: dict = {}
        if _wp_radii_str:
            for token in _wp_radii_str.split(','):
                token = token.strip()
                if ':' in token:
                    idx_s, r_s = token.split(':', 1)
                    try:
                        self._wp_radius_map[int(idx_s.strip())] = float(r_s.strip())
                    except ValueError:
                        self.get_logger().warn(f'wp_radii の書式エラー: "{token}" を無視します')
        self.speed_fix             = self.get_parameter('speed_fix').value
        self.speed_float           = self.get_parameter('speed_float').value
        self.stanley_k             = self.get_parameter('stanley_k').value
        self.k_soft                = self.get_parameter('k_soft').value
        self.max_steer             = self.get_parameter('max_steering_angle').value
        self.wheelbase             = self.get_parameter('wheelbase_m').value
        self.bootstrap_speed       = self.get_parameter('bootstrap_speed').value
        self.min_speed_for_heading = self.get_parameter('min_speed_for_heading').value
        self.heading_smoothing_w   = self.get_parameter('heading_smoothing_w').value
        self.max_speed             = self.get_parameter('max_speed_mps').value
        self.gnss_timeout_s        = self.get_parameter('gnss_timeout_s').value
        raw_str = self.get_parameter('corner_wp_indices').value.strip()
        self._corner_wp_set = set(
            int(s.strip()) for s in raw_str.split(',')
            if s.strip().lstrip('-').isdigit() and int(s.strip()) >= 0
        ) if raw_str else set()
        raw_skip = self.get_parameter('wp_skip_indices').value.strip()
        self._skip_wp_set = set(
            int(s.strip()) for s in raw_skip.split(',')
            if s.strip().lstrip('-').isdigit() and int(s.strip()) > 0
        ) if raw_skip else set()
        self.corner_slowdown_speed = self.get_parameter('corner_slowdown_speed').value
        self.corner_slowdown_dist  = self.get_parameter('corner_slowdown_dist').value
        tuning_log_file            = self.get_parameter('tuning_log_file').value

        self._tuning_csv_file = open(tuning_log_file, 'w', newline='')
        self._tuning_writer = csv.writer(self._tuning_csv_file)
        self._tuning_writer.writerow(_TUNING_LOG_HEADER)
        self._tuning_csv_file.flush()
        self._tuning_n_logged = 0
        self.get_logger().info(f'チューニング評価ログ記録先: {os.path.abspath(tuning_log_file)}')

        self.wps_llh = self._load_waypoints_llh(wp_file)
        if not self.wps_llh:
            self.get_logger().error(
                f'Waypointを読み込めませんでした: {wp_file} '
                '-- ファイルパスと形式(WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）を確認してください'
            )
            raise SystemExit(1)
        self.get_logger().info(f'Waypoint {len(self.wps_llh)}点 読み込み完了: {wp_file}')

        self.waypoints  = None
        self.waypoint_index = 0
        self.origin_ecef = None

        self.current_x      = None
        self.current_y      = None
        self.current_speed  = 0.0
        self.heading        = None
        self._ve_filtered   = 0.0
        self._vn_filtered   = 0.0
        self.current_status = 0
        self.fix_achieved   = False
        self._corner_exit_dist_remaining = 0.0
        self._last_ctrl_x   = None
        self._last_ctrl_y   = None

        self.score = 0

        self.last_gnss_time = self.get_clock().now()
        self.last_pos_enu_cov = [0.0] * 9

        self.cmd_pub  = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.gnss_sub = self.create_subscription(GnssSolution, '/gnss/solution', self._gnss_callback, 10)
        self.create_timer(0.1, self._safety_check)

        self.get_logger().info('stanley_controller 起動完了（GNSS ENU原点確定待ち）')

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

    def _resolve_origin_and_convert(self, msg: GnssSolution) -> bool:
        ox = msg.pos_enu_org_ecef.x
        oy = msg.pos_enu_org_ecef.y
        oz = msg.pos_enu_org_ecef.z

        if ox == 0.0 and oy == 0.0 and oz == 0.0:
            return False

        self.origin_ecef = (ox, oy, oz)
        origin_lat, origin_lon, origin_alt = pm.ecef2geodetic(ox, oy, oz)

        llh_arr = np.array(self.wps_llh)
        e, n, _u = pm.geodetic2enu(
            llh_arr[:, 0], llh_arr[:, 1], llh_arr[:, 2],
            origin_lat, origin_lon, origin_alt
        )
        self.waypoints = np.column_stack([e, n])

        if self._skip_wp_set:
            n_total = len(self.waypoints)
            valid_skip = {i for i in self._skip_wp_set if 0 < i < n_total - 1}
            if valid_skip:
                keep = [i for i in range(n_total) if i not in valid_skip]
                old_to_new = {old: new for new, old in enumerate(keep)}
                self.waypoints = self.waypoints[np.array(keep)]
                self._corner_wp_set = {old_to_new[i] for i in self._corner_wp_set if i in old_to_new}
                self._wp_radius_map = {
                    old_to_new[i]: r for i, r in self._wp_radius_map.items() if i in old_to_new
                }
                self.get_logger().info(
                    f'WPスキップ: 元WP{sorted(valid_skip)} を経路から除外 → {len(self.waypoints)}点で走行'
                )

        if self._corner_wp_set:
            for idx in sorted(self._corner_wp_set):
                self.get_logger().info(
                    f'減速WP指定: WP[{idx}] → {self.corner_slowdown_dist:.1f}m手前からランプ減速 → '
                    f'{self.corner_slowdown_speed:.1f}m/s、通過後{self.corner_slowdown_dist:.1f}mかけてランプ加速'
                )
        else:
            self.get_logger().info('減速WP指定なし → 全WPを順に追従')

        if not self._wp_radius_map:
            self.get_logger().info(f'WP到達半径: 全WP共通 {self.waypoint_radius:.2f}m')

        self.get_logger().info(
            f'GNSS ENU原点確定 → Waypoint {len(self.waypoints)}点を変換完了（直線区間で接続）。'
            f'最初の目標 → X={self.waypoints[0,0]:.2f}m, Y={self.waypoints[0,1]:.2f}m'
        )
        return True

    def _gnss_callback(self, msg: GnssSolution):
        self.current_status = msg.status
        self.last_gnss_time = self.get_clock().now()
        self.last_pos_enu_cov = msg.pos_enu_cov

        if msg.status not in _VALID_STATUSES:
            return

        if self.current_status == GnssSolution.STATUS_FIX and not self.fix_achieved:
            self.fix_achieved = True
            self.get_logger().info('★ 起動後初回のFIXを達成 → 走行を開始します')

        if self.waypoints is None:
            self._resolve_origin_and_convert(msg)
            return

        ve, vn = msg.vel_enu.x, msg.vel_enu.y
        self.current_speed = math.hypot(ve, vn)
        if self.current_speed >= self.min_speed_for_heading:
            if self.heading is None:
                self._ve_filtered, self._vn_filtered = ve, vn
            else:
                w = self.heading_smoothing_w
                self._ve_filtered = (1 - w) * self._ve_filtered + w * ve
                self._vn_filtered = (1 - w) * self._vn_filtered + w * vn
            self.heading = math.atan2(self._vn_filtered, self._ve_filtered)

        self.current_x = msg.pos_enu.x
        self.current_y = msg.pos_enu.y

        status_str = 'FIX' if self.current_status == GnssSolution.STATUS_FIX else 'FLOAT'
        self.get_logger().info(
            f'GNSS: Status={status_str}  '
            f'pos=({self.current_x:.2f},{self.current_y:.2f})m  '
            f'speed={self.current_speed:.2f}m/s'
        )

        self._control()

    def _control(self):
        if self.current_x is None:
            self._publish_stop()
            return

        if not self.fix_achieved:
            self._publish_stop()
            return

        if self.heading is None:
            self._publish_command(max(0.0, min(self.bootstrap_speed, self.max_speed)), 0.0)
            self.get_logger().info('ヘディング未確定 → 直進ブートストラップ中...')
            return

        if self._last_ctrl_x is not None and self._corner_exit_dist_remaining > 0:
            dist_moved = math.hypot(self.current_x - self._last_ctrl_x,
                                    self.current_y - self._last_ctrl_y)
            self._corner_exit_dist_remaining = max(0.0, self._corner_exit_dist_remaining - dist_moved)
        self._last_ctrl_x = self.current_x
        self._last_ctrl_y = self.current_y

        tx, ty = self.waypoints[self.waypoint_index]

        wp_r = self._wp_radius_map.get(self.waypoint_index, self.waypoint_radius)
        dist_to_wp = math.hypot(tx - self.current_x, ty - self.current_y)
        if dist_to_wp <= wp_r:
            passed_idx = self.waypoint_index
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

            if passed_idx in self._corner_wp_set:
                self._corner_exit_dist_remaining = self.corner_slowdown_dist
                self.get_logger().info(
                    f'コーナーWP[{passed_idx}]通過 → 立ち上がり減速 {self.corner_slowdown_dist:.1f}m 継続'
                )

            tx, ty = self.waypoints[self.waypoint_index]

        # 経路セグメント（直前WP → 目標WP）
        prev_idx = self.waypoint_index - 1
        if prev_idx >= 0:
            sx, sy = float(self.waypoints[prev_idx][0]), float(self.waypoints[prev_idx][1])
        else:
            sx, sy = self.current_x, self.current_y

        path_heading = math.atan2(ty - sy, tx - sx)

        heading_error = path_heading - self.heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        seg_dx, seg_dy = tx - sx, ty - sy
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len > 1e-6:
            cross_track_error = (
                (self.current_x - sx) * seg_dy - (self.current_y - sy) * seg_dx
            ) / seg_len
        else:
            cross_track_error = 0.0

        # Stanley則: δ = heading_error + atan2(k × CTE, k_soft + v)
        cross_track_term = math.atan2(
            self.stanley_k * cross_track_error,
            self.k_soft + self.current_speed
        )
        steer = heading_error + cross_track_term
        steer = max(-self.max_steer, min(self.max_steer, steer))

        speed = self.speed_fix if self.current_status == GnssSolution.STATUS_FIX else self.speed_float
        speed = max(0.0, min(self.max_speed, speed))
        dist_to_target = math.hypot(tx - self.current_x, ty - self.current_y)
        approaching_corner = (
            self.waypoint_index in self._corner_wp_set
            and dist_to_target <= self.corner_slowdown_dist
        )
        if approaching_corner:
            ratio = max(0.0, min(1.0, dist_to_target / self.corner_slowdown_dist))
            speed = self.corner_slowdown_speed + ratio * (speed - self.corner_slowdown_speed)
        elif self._corner_exit_dist_remaining > 0:
            ratio = max(0.0, min(1.0, self._corner_exit_dist_remaining / self.corner_slowdown_dist))
            speed = self.corner_slowdown_speed + (1.0 - ratio) * (speed - self.corner_slowdown_speed)

        self._publish_command(speed, steer)

        self._tuning_writer.writerow([
            f'{time.time():.3f}',
            f'{cross_track_error:.3f}',
            f'{math.degrees(heading_error):.2f}',
            f'{math.degrees(cross_track_term):.2f}',
            f'{math.degrees(steer):.2f}',
            f'{self.current_speed:.3f}',
        ])
        self._tuning_csv_file.flush()
        self._tuning_n_logged += 1

        self.get_logger().debug(
            f'WP[{self.waypoint_index}] '
            f'方位誤差={math.degrees(heading_error):+.1f}°  '
            f'横偏差={cross_track_error:+.2f}m  '
            f'CTE項={math.degrees(cross_track_term):+.1f}°  '
            f'ステア={math.degrees(steer):+.1f}°  '
            f'速度={speed:.1f}m/s  '
            f'Status={"FIX" if self.current_status == GnssSolution.STATUS_FIX else "FLOAT"}'
        )

    def _safety_check(self):
        elapsed = (self.get_clock().now() - self.last_gnss_time).nanoseconds / 1e9
        if elapsed > self.gnss_timeout_s:
            self.get_logger().warn(f'GNSSデータ受信なし（{elapsed:.1f}秒）→ 安全停止')
            self._publish_stop()
        elif self.current_status not in _VALID_STATUSES:
            self.get_logger().warn(
                f'走行に必要な精度(FIX/FLOAT)に未到達: '
                f'{_format_status_info(self.current_status, self.last_pos_enu_cov)} → 待機中',
                throttle_duration_sec=1.0,
            )
            self._publish_stop()

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
