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
    'time', 'enu_x', 'enu_y', 'actual_speed',
    'waypoint_index', 'dist_to_target', 'approaching_corner', 'ramp_active', 'lh_ramp_active',
    'cross_track_error', 'alpha_deg', 'steer_deg',
    'lookahead_dist', 'cmd_speed', 'seg_speed',
]


def _format_status_info(status: int, pos_enu_cov) -> str:
    name = _STATUS_NAMES.get(status, f'unknown({status})')
    h_var = pos_enu_cov[0] if len(pos_enu_cov) > 0 else 0.0
    acc_str = f'{math.sqrt(h_var):.1f}m' if h_var > 0.0 else 'n/a'
    return f'status={status}({name}) 水平精度(目安)≈{acc_str}'


def _parse_int_set(raw: str, positive_only: bool = False) -> set:
    return {
        int(v) for s in raw.split(',')
        if (v := s.strip()).lstrip('-').isdigit()
        and (int(v) > 0 if positive_only else int(v) >= 0)
    } if raw else set()


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info('Pure Pursuit Controller Node has been started!')

        self.declare_parameter('wp_file', 'wp_position_basic.csv')
        self.declare_parameter('wp_radius', 1.0)
        self.declare_parameter('wp_radii', '2:1.5,3:1.0,5:1.2,6:1.7,7:2.0,8:1.7')
        self.declare_parameter('speed_min', 1.0)
        self.declare_parameter('speed_max', 3.0)
        self.declare_parameter('speed_dist_short', 5.0)
        self.declare_parameter('speed_dist_long', 10.0)
        self.declare_parameter('wheelbase_m', 0.267)
        self.declare_parameter('lookahead_min', 2.0)
        self.declare_parameter('lookahead_fraction', 0.5)
        self.declare_parameter('max_steering_angle', math.radians(25.0))
        self.declare_parameter('bootstrap_speed', 0.5)
        self.declare_parameter('min_speed_for_heading', 0.05)
        self.declare_parameter('heading_smoothing_w', 0.35)
        self.declare_parameter('max_speed_mps', 4.0)
        self.declare_parameter('gnss_timeout_s', 2.0)
        self.declare_parameter('corner_wp_indices', '2,6,8')
        self.declare_parameter('lh_ramp_wp_indices', '6,8')
        self.declare_parameter('wp_skip_indices', '')
        self.declare_parameter('corner_slowdown_ratio', 0.6)
        self.declare_parameter('corner_slowdown_base_dist', 10.0)
        default_log = os.path.join(
            os.path.expanduser('~'), 'ros2_ws', 'gnss_logs', 'pure_pursuit_log_latest.csv'
        )
        self.declare_parameter('tuning_log_file', default_log)

        wp_file                   = self.get_parameter('wp_file').value
        self.waypoint_radius      = self.get_parameter('wp_radius').value
        self._wp_radius_map: dict = {}
        for token in self.get_parameter('wp_radii').value.strip().split(','):
            if ':' not in (token := token.strip()):
                continue
            idx_s, r_s = token.split(':', 1)
            try:
                self._wp_radius_map[int(idx_s.strip())] = float(r_s.strip())
            except ValueError:
                self.get_logger().warn(f'wp_radii の書式エラー: "{token}" を無視します')
        self.speed_min                 = self.get_parameter('speed_min').value
        self.speed_max                 = self.get_parameter('speed_max').value
        self.speed_dist_short          = self.get_parameter('speed_dist_short').value
        self.speed_dist_long           = self.get_parameter('speed_dist_long').value
        self.wheelbase                 = self.get_parameter('wheelbase_m').value
        self.lookahead_min             = self.get_parameter('lookahead_min').value
        self.lookahead_fraction        = self.get_parameter('lookahead_fraction').value
        self.max_steer                 = self.get_parameter('max_steering_angle').value
        self.bootstrap_speed           = self.get_parameter('bootstrap_speed').value
        self.min_speed_for_heading     = self.get_parameter('min_speed_for_heading').value
        self.heading_smoothing_w       = self.get_parameter('heading_smoothing_w').value
        self.max_speed                 = self.get_parameter('max_speed_mps').value
        self.gnss_timeout_s            = self.get_parameter('gnss_timeout_s').value
        self._corner_wp_set  = _parse_int_set(self.get_parameter('corner_wp_indices').value.strip())
        self._lh_ramp_wp_set = _parse_int_set(self.get_parameter('lh_ramp_wp_indices').value.strip())
        self._skip_wp_set    = _parse_int_set(self.get_parameter('wp_skip_indices').value.strip(),
                                              positive_only=True)
        self.corner_slowdown_ratio      = self.get_parameter('corner_slowdown_ratio').value
        self.corner_slowdown_base_dist  = self.get_parameter('corner_slowdown_base_dist').value
        tuning_log_file                = self.get_parameter('tuning_log_file').value

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

        self.waypoints           = None
        self.waypoint_index      = 0
        self._seg_speeds: list   = []
        self._seg_gains: list    = []

        self.current_x      = None
        self.current_y      = None
        self.current_speed  = 0.0
        self.heading        = None
        self._ve_filtered   = 0.0
        self._vn_filtered   = 0.0
        self.current_status = 0
        self.fix_achieved   = False

        self._speed_ramp_remaining = 0.0
        self._speed_ramp_total     = 1.0
        self._speed_ramp_from      = 0.0
        self._speed_ramp_to        = 0.0
        self._last_cmd_speed       = 0.0
        self._lh_gain_ramp_remaining = 0.0
        self._lh_gain_ramp_total     = 1.0
        self._lh_gain_ramp_to        = 0.0
        self._last_ctrl_x = None
        self._last_ctrl_y = None

        self.score = 0

        self.last_gnss_time   = self.get_clock().now()
        self.last_pos_enu_cov = [0.0] * 9

        self.cmd_pub  = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.gnss_sub = self.create_subscription(
            GnssSolution, '/gnss/solution', self._gnss_callback, 10
        )
        self.create_timer(0.1, self._safety_check)

        self.get_logger().info('pure_pursuit_controller 起動完了（GNSS ENU原点確定待ち）')

    def _load_waypoints_llh(self, filepath: str) -> list:
        waypoints = []
        if not os.path.exists(filepath):
            self.get_logger().error(f'ファイルが存在しません: {filepath}')
            return waypoints
        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)
                for i, row in enumerate(reader):
                    lat    = float(row['Latitude(deg)'])
                    lon    = float(row['Longitude(deg)'])
                    height = float(row['Ellipsoidal Height(m)'])
                    waypoints.append((lat, lon, height))
                    self.get_logger().debug(
                        f'  WP[{i}]: lat={lat:.8f}, lon={lon:.8f}, h={height:.2f}'
                    )
        except Exception as e:
            self.get_logger().error(f'Waypoint読み込みエラー: {e}')
            return []
        return waypoints

    def _compute_segment_params(self):
        speeds, gains = [], []
        d_range = self.speed_dist_long - self.speed_dist_short

        self.get_logger().info('--- セグメントパラメータ ---')
        for i, ((e1, n1), (e2, n2)) in enumerate(
            zip(self.waypoints[:-1], self.waypoints[1:])
        ):
            d = math.hypot(e2 - e1, n2 - n1)

            t = (d - self.speed_dist_short) / d_range if d_range else 0.0
            seg_speed = self.speed_min + max(0.0, min(1.0, t)) * (self.speed_max - self.speed_min)
            speeds.append(seg_speed)

            th = max(0.0, (self.lookahead_fraction * d - self.lookahead_min) / seg_speed)
            gains.append(th)

            ld_at_seg = self.lookahead_min + th * seg_speed
            self.get_logger().info(
                f'  seg[{i}] d={d:.2f}m  speed={seg_speed:.2f}m/s  '
                f'T_h={th:.2f}s  Ld={ld_at_seg:.2f}m'
            )

        self._seg_speeds = speeds
        self._seg_gains  = gains

    def _resolve_origin_and_convert(self, msg: GnssSolution):
        ox, oy, oz = msg.pos_enu_org_ecef.x, msg.pos_enu_org_ecef.y, msg.pos_enu_org_ecef.z

        if ox == 0.0 and oy == 0.0 and oz == 0.0:
            return

        origin_lat, origin_lon, origin_alt = pm.ecef2geodetic(ox, oy, oz)
        llh_arr = np.array(self.wps_llh)
        e, n, _u = pm.geodetic2enu(
            llh_arr[:, 0], llh_arr[:, 1], llh_arr[:, 2],
            origin_lat, origin_lon, origin_alt
        )
        self.waypoints = np.column_stack([e, n])

        if self._skip_wp_set:
            n_total    = len(self.waypoints)
            valid_skip = {i for i in self._skip_wp_set if 0 < i < n_total - 1}
            if valid_skip:
                keep         = [i for i in range(n_total) if i not in valid_skip]
                old_to_new   = {old: new for new, old in enumerate(keep)}
                self.waypoints       = self.waypoints[np.array(keep)]
                self._corner_wp_set  = {old_to_new[i] for i in self._corner_wp_set  if i in old_to_new}
                self._lh_ramp_wp_set = {old_to_new[i] for i in self._lh_ramp_wp_set if i in old_to_new}
                self._wp_radius_map  = {
                    old_to_new[i]: r for i, r in self._wp_radius_map.items() if i in old_to_new
                }
                self.get_logger().info(
                    f'WPスキップ: 元WP{sorted(valid_skip)} を除外 → {len(self.waypoints)}点で走行'
                )

        self._compute_segment_params()

        if self._corner_wp_set:
            for idx in sorted(self._corner_wp_set):
                seg_idx    = max(0, idx - 1) % len(self._seg_speeds)
                approach_v = self._seg_speeds[seg_idx]
                corner_v   = approach_v * self.corner_slowdown_ratio
                corner_d   = self.corner_slowdown_base_dist * (approach_v / self.speed_min)
                r          = self._wp_radius_map.get(idx, self.waypoint_radius)
                self.get_logger().info(
                    f'  コーナーWP[{idx}]  到達半径={r:.2f}m  '
                    f'減速 {corner_d:.1f}m手前 {approach_v:.2f}→{corner_v:.2f}m/s'
                )
        else:
            self.get_logger().info('減速WP指定なし → 全WPを順に追従')

        if self._wp_radius_map:
            for idx, r in sorted(self._wp_radius_map.items()):
                self.get_logger().info(f'  WP[{idx}] 到達半径: {r:.2f}m（個別指定）')
        else:
            self.get_logger().info(f'WP到達半径: 全WP共通 {self.waypoint_radius:.2f}m')

        self.get_logger().info(
            f'GNSS ENU原点確定 → Waypoint {len(self.waypoints)}点変換完了。'
            f'最初の目標 → X={self.waypoints[0, 0]:.2f}m, Y={self.waypoints[0, 1]:.2f}m'
        )

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

        self.get_logger().info(
            f'GNSS: Status={"FIX" if self.current_status == GnssSolution.STATUS_FIX else "FLOAT"}  '
            f'pos=({self.current_x:.2f},{self.current_y:.2f})m  '
            f'speed={self.current_speed:.2f}m/s'
        )

        self._control()

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

    def _seg_params(self):
        if self.waypoint_index == 0 or not self._seg_speeds:
            return self.speed_min, 0.0
        idx = (self.waypoint_index - 1) % len(self._seg_speeds)
        return self._seg_speeds[idx], self._seg_gains[idx]

    def _control(self):
        if self.current_x is None or not self.fix_achieved:
            self._publish_stop()
            return

        if self.heading is None:
            self._publish_command(max(0.0, min(self.bootstrap_speed, self.max_speed)), 0.0)
            self.get_logger().info('ヘディング未確定 → 直進ブートストラップ中...')
            return

        if self._last_ctrl_x is not None and (
            self._speed_ramp_remaining or self._lh_gain_ramp_remaining
        ):
            dist_moved = math.hypot(
                self.current_x - self._last_ctrl_x, self.current_y - self._last_ctrl_y
            )
            if self._speed_ramp_remaining:
                self._speed_ramp_remaining = max(0.0, self._speed_ramp_remaining - dist_moved)
            if self._lh_gain_ramp_remaining:
                self._lh_gain_ramp_remaining = max(0.0, self._lh_gain_ramp_remaining - dist_moved)
        self._last_ctrl_x = self.current_x
        self._last_ctrl_y = self.current_y

        tx0, ty0 = self.waypoints[self.waypoint_index]

        seg_speed, seg_gain = self._seg_params()
        corner_speed = seg_speed * self.corner_slowdown_ratio
        corner_dist  = self.corner_slowdown_base_dist * (seg_speed / self.speed_min)

        dist_to_wp     = math.hypot(tx0 - self.current_x, ty0 - self.current_y)
        arrival_radius = self._wp_radius_map.get(self.waypoint_index, self.waypoint_radius)
        if dist_to_wp <= arrival_radius:
            passed_idx = self.waypoint_index
            self.score += 10
            self.get_logger().info(
                f'★ WP[{passed_idx}] 通過！ (到達距離={dist_to_wp:.2f}m) '
                f'ゲート通過 +10点 → 合計{self.score}点'
            )
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.score += 50
                self.get_logger().info(
                    f'★★★ 1周完了！ 周回ボーナス +50点 → 合計{self.score}点 '
                    f'停止せずWP[0]から再周回します'
                )
                self.waypoint_index = 0

            next_seg_speed, next_seg_gain = self._seg_params()
            if passed_idx in self._corner_wp_set:
                ramp_from = corner_speed
                ramp_dist = corner_dist
                self.get_logger().info(
                    f'コーナーWP[{passed_idx}]通過 → 出口ランプ {ramp_dist:.1f}m '
                    f'({ramp_from:.2f}→{next_seg_speed:.2f}m/s)'
                )
            else:
                ramp_from = max(self.speed_min, self._last_cmd_speed)
                max_delta = max(self.speed_max - self.speed_min, 1e-3)
                ramp_dist = self.corner_slowdown_base_dist * abs(next_seg_speed - ramp_from) / max_delta
                if ramp_dist > 0.01:
                    self.get_logger().info(
                        f'WP[{passed_idx}]通過 → 遷移ランプ {ramp_dist:.1f}m '
                        f'({ramp_from:.2f}→{next_seg_speed:.2f}m/s)'
                    )
            if ramp_dist > 0.01 and abs(next_seg_speed - ramp_from) > 0.01:
                self._speed_ramp_from      = ramp_from
                self._speed_ramp_to        = next_seg_speed
                self._speed_ramp_total     = ramp_dist
                self._speed_ramp_remaining = ramp_dist
            else:
                self._speed_ramp_remaining = 0.0

            if passed_idx in self._lh_ramp_wp_set:
                self._lh_gain_ramp_to        = next_seg_gain
                self._lh_gain_ramp_total     = max(ramp_dist, 0.1)
                self._lh_gain_ramp_remaining = max(ramp_dist, 0.1)
                self.get_logger().info(
                    f'lh_gain_ramp発動 WP[{passed_idx}] → Ld={self.lookahead_min:.1f}m固定 '
                    f'{ramp_dist:.1f}mかけてLd={self.lookahead_min + next_seg_gain * next_seg_speed:.1f}mへ回復'
                )
            else:
                self._lh_gain_ramp_remaining = 0.0

            tx0, ty0 = self.waypoints[self.waypoint_index]
            self.get_logger().info(
                f'次の目標 → WP[{self.waypoint_index}] X={tx0:.2f}m, Y={ty0:.2f}m'
            )

        prev_idx = self.waypoint_index - 1
        sx, sy = (
            (float(self.waypoints[prev_idx][0]), float(self.waypoints[prev_idx][1]))
            if prev_idx >= 0 else (self.current_x, self.current_y)
        )
        seg_dx, seg_dy = tx0 - sx, ty0 - sy
        seg_len = math.hypot(seg_dx, seg_dy)
        cross_track_error = (
            ((self.current_x - sx) * seg_dy - (self.current_y - sy) * seg_dx) / seg_len
            if seg_len > 1e-6 else 0.0
        )

        dist_to_target = math.hypot(tx0 - self.current_x, ty0 - self.current_y)
        approaching_corner = (
            self.waypoint_index in self._corner_wp_set and dist_to_target <= corner_dist
        )

        if approaching_corner:
            ratio     = max(0.0, min(1.0, dist_to_target / corner_dist))
            cmd_speed = corner_speed + ratio * (seg_speed - corner_speed)
        elif self._speed_ramp_remaining:
            ratio     = max(0.0, min(1.0, self._speed_ramp_remaining / self._speed_ramp_total))
            cmd_speed = self._speed_ramp_to + ratio * (self._speed_ramp_from - self._speed_ramp_to)
        else:
            cmd_speed = seg_speed

        cmd_speed = max(0.0, min(self.max_speed, cmd_speed))

        if self._lh_gain_ramp_remaining:
            lh_ratio       = max(0.0, min(1.0, self._lh_gain_ramp_remaining / self._lh_gain_ramp_total))
            effective_gain = self._lh_gain_ramp_to * (1.0 - lh_ratio)
        else:
            effective_gain = seg_gain
        lookahead_dist = self.lookahead_min + effective_gain * cmd_speed

        if approaching_corner:
            lookahead_dist = min(lookahead_dist, dist_to_target)

        tx, ty         = self._lookahead_target(lookahead_dist)
        dx, dy         = tx - self.current_x, ty - self.current_y
        ld_actual      = math.hypot(dx, dy)
        target_bearing = math.atan2(dy, dx)

        alpha = math.atan2(math.sin(target_bearing - self.heading),
                           math.cos(target_bearing - self.heading))

        steer = (
            math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld_actual)
            if ld_actual >= 1e-3 else 0.0
        )
        steer = max(-self.max_steer, min(self.max_steer, steer))

        self._publish_command(cmd_speed, steer)
        self._last_cmd_speed = cmd_speed

        alpha_deg = math.degrees(alpha)
        steer_deg = math.degrees(steer)
        self._tuning_writer.writerow([
            f'{time.time():.3f}',
            f'{self.current_x:.3f}',
            f'{self.current_y:.3f}',
            f'{self.current_speed:.3f}',
            f'{self.waypoint_index}',
            f'{dist_to_target:.3f}',
            f'{1 if approaching_corner else 0}',
            f'{1 if self._speed_ramp_remaining else 0}',
            f'{1 if self._lh_gain_ramp_remaining else 0}',
            f'{cross_track_error:.3f}',
            f'{alpha_deg:.2f}',
            f'{steer_deg:.2f}',
            f'{lookahead_dist:.3f}',
            f'{cmd_speed:.3f}',
            f'{seg_speed:.3f}',
        ])
        self._tuning_csv_file.flush()
        self._tuning_n_logged += 1

        self.get_logger().debug(
            f'WP[{self.waypoint_index}] Ld={ld_actual:.2f}m alpha={alpha_deg:+.1f}° '
            f'ステア={steer_deg:+.1f}°  CTE={cross_track_error:+.2f}m  '
            f'速度={cmd_speed:.2f}m/s(seg={seg_speed:.2f})  '
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
        self.get_logger().info(
            f'チューニング評価ログ終了 → 合計{self._tuning_n_logged}行を記録しました'
        )
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
