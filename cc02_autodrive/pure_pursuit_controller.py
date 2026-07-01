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

_TUNING_LOG_HEADER = ['time', 'cross_track_error', 'alpha_deg', 'steer_deg', 'lookahead_dist', 'speed']


def _format_status_info(status: int, pos_enu_cov) -> str:
    name = _STATUS_NAMES.get(status, f'unknown({status})')
    h_var = pos_enu_cov[0] if len(pos_enu_cov) > 0 else 0.0
    acc_str = f'{math.sqrt(h_var):.1f}m' if h_var > 0.0 else 'n/a'
    return f'status={status}({name}) 水平精度(目安)≈{acc_str}'


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info('Pure Pursuit Controller Node has been started!')

        self.declare_parameter('wp_file', 'wp_position_basic.csv')
        self.declare_parameter('wp_radius', 0.7)    # m: WP到達判定のデフォルト半径
        self.declare_parameter('wp_radii', "")       # WPごとの半径上書き（例: "1:0.7,3:2.0"、未指定WPはwp_radiusを使用）
        self.declare_parameter('speed_fix',   3.0)              # m/s: RTK-FIX時の速度
        self.declare_parameter('speed_float', 2.5)              # m/s: RTK-FLOAT時の速度
        self.declare_parameter('wheelbase_m', 0.267)            # m: 前後輪軸間距離（実測値）　!調整済み！
        self.declare_parameter('lookahead_min', 1.0)            # m: 最低ルックアヘッド距離
        self.declare_parameter('lookahead_gain', 3.5)           # s: ルックアヘッド速度ゲイン（Ld = min + gain × speed）
        self.declare_parameter('max_steering_angle', math.radians(25.0))  # rad: ステアリング最大角　！調整済み！
        self.declare_parameter('bootstrap_speed', 0.3)          # m/s: ヘディング確定前の直進速度　　！調整済み！
        self.declare_parameter('min_speed_for_heading', 0.05)   # m/s: ヘディング推定に使う最低速度
        self.declare_parameter('heading_smoothing_w', 0.35)     # ヘディングEMA係数（小さいほど遅延・安定）
        self.declare_parameter('max_speed_mps', 4.0)            # m/s: 速度の安全上限
        self.declare_parameter('gnss_timeout_s', 2.0)           # s: GNSS受信タイムアウト　　！調整済み！
        self.declare_parameter('corner_wp_indices', '0,2,6,8')              # 減速対象WPインデックス カンマ区切り（例: "1,3,5"、空=なし）
        self.declare_parameter('wp_skip_indices', "3,7")                # 無視するWPインデックス カンマ区切り（例: "7"、WP0・最終WPは不可）
        self.declare_parameter('corner_slowdown_speed', 1.0)        # m/s: コーナー接近時の速度上限
        self.declare_parameter('corner_slowdown_dist', 8.0)        # m: コーナー減速を開始する距離
        self.declare_parameter('corner_arc_n', 8)                   # コーナー円弧の補間点数（0=円弧挿入なし）
        self.declare_parameter('kf_pos_noise_std', 0.02)  # m: GNSS位置ノイズσ（観測ノイズR）実測0.008m×2.5倍
        self.declare_parameter('kf_vel_noise_std', 0.1)   # m/s: GNSS速度ノイズσ（観測ノイズR）
        self.declare_parameter('kf_process_noise', 1.0)   # m/s²: 加速度不確かさσ（プロセスノイズQ）

        default_tuning_log = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'gnss_logs', 'pure_pursuit_log_latest.csv')
        self.declare_parameter('tuning_log_file', default_tuning_log)

        wp_file                    = self.get_parameter('wp_file').value
        self.waypoint_radius = self.get_parameter('wp_radius').value
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
        self.wheelbase             = self.get_parameter('wheelbase_m').value
        self.lookahead_min         = self.get_parameter('lookahead_min').value
        self.lookahead_gain        = self.get_parameter('lookahead_gain').value
        self.max_steer             = self.get_parameter('max_steering_angle').value
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
        self.corner_slowdown_speed = self.get_parameter('corner_slowdown_speed').value
        self.corner_slowdown_dist  = self.get_parameter('corner_slowdown_dist').value
        self.corner_arc_n          = self.get_parameter('corner_arc_n').value
        self._arc_wp_set: set = set()   # 円弧補間点（到達半径=wp_radius、速度減速トリガーなし）
        raw_skip = self.get_parameter('wp_skip_indices').value.strip()
        self._skip_wp_set = set(
            int(s.strip()) for s in raw_skip.split(',')
            if s.strip().lstrip('-').isdigit() and int(s.strip()) > 0
        ) if raw_skip else set()
        self.kf_pos_noise_std = self.get_parameter('kf_pos_noise_std').value
        self.kf_vel_noise_std = self.get_parameter('kf_vel_noise_std').value
        self.kf_process_noise = self.get_parameter('kf_process_noise').value
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

        self.waypoints = None       # ENU変換後の(x, y)リスト。原点確定後にセットされる
        self.waypoint_index = 0
        self.origin_ecef = None     # GNSS ENU原点（ECEF）。最初の有効Fixで一度だけ確定

        self.current_x      = None  # KF平滑化後の現在地 ENU-X [m]（ルックアヘッド計算に使う）
        self.current_y      = None  # KF平滑化後の現在地 ENU-Y [m]（ルックアヘッド計算に使う）
        self.raw_x          = None  # 生のGNSS位置 ENU-X [m]（WP到達判定専用）
        self.raw_y          = None  # 生のGNSS位置 ENU-Y [m]（WP到達判定専用）
        self.current_speed  = 0.0   # 現在の車速 [m/s]
        self.heading        = None  # 進行方向 [rad]（東=0, 北=π/2）
        self._ve_filtered   = 0.0   # ヘディングEMA内部状態（東方向速度）
        self._vn_filtered   = 0.0   # ヘディングEMA内部状態（北方向速度）
        self.current_status = 0     # 最新のGNSSステータス
        self.fix_achieved   = False  # 起動後に一度でもFIXを取得したか
        self._kf_state      = None  # カルマンフィルタ状態 [x, y, vx, vy]
        self._kf_P          = None  # カルマンフィルタ共分散行列 (4×4)
        self._kf_last_time  = None  # 前回KF更新時刻 [s]
        self._corner_exit_dist_remaining = 0.0  # WP通過後の立ち上がり減速残距離 [m]
        self._last_ctrl_x   = None  # 前回制御時の位置（走行距離計算用）
        self._last_ctrl_y   = None

        self.score = 0  # 競技採点用（ゲート通過+10点、周回+50点）

        self.last_gnss_time = self.get_clock().now()
        self.last_pos_enu_cov = [0.0] * 9

        self.cmd_pub  = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.gnss_sub = self.create_subscription(GnssSolution, '/gnss/solution', self._gnss_callback, 10)
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
                    lat = float(row['Latitude(deg)'])
                    lon = float(row['Longitude(deg)'])
                    height = float(row['Ellipsoidal Height(m)'])
                    waypoints.append((lat, lon, height))
                    self.get_logger().debug(f'  WP[{i}]: lat={lat:.8f}, lon={lon:.8f}, h={height:.2f}')
        except Exception as e:
            self.get_logger().error(f'Waypoint読み込みエラー: {e}')
            return []
        return waypoints

    def _insert_corner_arcs(self):
        """corner_wp_set の各コーナーWPを最小旋回半径の円弧で置き換える。
        コーナーWP → 接点P1(入口) + 円弧中間点×n + 接点P2(出口) に展開。
        _corner_wp_set を P1 インデックスに更新し、
        _arc_wp_set に中間点・P2 インデックスを記録する。
        """
        if not self._corner_wp_set or self.corner_arc_n <= 0:
            return

        R_min = self.wheelbase / math.tan(self.max_steer)
        n_arc = self.corner_arc_n
        n_total = len(self.waypoints)

        new_wps = []
        new_corner_set = set()
        new_arc_set = set()
        old_to_new: dict = {}   # 非コーナーWPの旧→新インデックス（_wp_radius_mapのリマップ用）

        for i in range(n_total):
            pt = self.waypoints[i]

            if i not in self._corner_wp_set or i == 0 or i >= n_total - 1:
                old_to_new[i] = len(new_wps)
                new_wps.append(pt)
                continue

            prev_pt = self.waypoints[i - 1]
            next_pt = self.waypoints[i + 1]
            v_in  = pt - prev_pt
            v_out = next_pt - pt
            d_in  = float(np.linalg.norm(v_in))
            d_out = float(np.linalg.norm(v_out))

            if d_in < 0.1 or d_out < 0.1:
                old_to_new[i] = len(new_wps)
                new_wps.append(pt)
                new_corner_set.add(old_to_new[i])
                continue

            u_in  = v_in  / d_in
            u_out = v_out / d_out

            cos_phi = float(np.clip(np.dot(u_in, u_out), -1.0, 1.0))
            phi   = math.acos(cos_phi)          # 方向ベクトル間の角度（φ≈π:直線、φ≈0:Uターン）
            theta = math.pi - phi               # 転向角（θ≈0:直線、θ≈π:Uターン）

            if theta < math.radians(5.0):
                old_to_new[i] = len(new_wps)
                new_wps.append(pt)
                continue

            tan_half = math.tan(theta / 2.0)
            t = min(R_min / tan_half, d_in * 0.45, d_out * 0.45)
            if t < 0.05:
                old_to_new[i] = len(new_wps)
                new_wps.append(pt)
                new_corner_set.add(old_to_new[i])
                continue

            R = t * tan_half                    # 実際の旋回半径（セグメント短い場合は縮小）

            P1 = pt - t * u_in                  # 入口接点
            P2 = pt + t * u_out                 # 出口接点

            cross_z = float(u_in[0] * u_out[1] - u_in[1] * u_out[0])
            if cross_z >= 0:
                perp = np.array([-u_in[1],  u_in[0]])  # 左法線（左折=CCW）
            else:
                perp = np.array([ u_in[1], -u_in[0]])  # 右法線（右折=CW）
            arc_center = P1 + R * perp

            ang_s = math.atan2(float(P1[1] - arc_center[1]), float(P1[0] - arc_center[0]))
            ang_e = math.atan2(float(P2[1] - arc_center[1]), float(P2[0] - arc_center[0]))
            if cross_z >= 0:
                while ang_e <= ang_s:
                    ang_e += 2.0 * math.pi
            else:
                while ang_e >= ang_s:
                    ang_e -= 2.0 * math.pi

            # P1 をコーナーWP（速度減速トリガー）として追加
            old_to_new[i] = len(new_wps)       # 旧コーナーWP → P1 にマップ
            new_wps.append(P1)
            new_corner_set.add(len(new_wps) - 1)

            # 円弧中間点
            for j in range(1, n_arc + 1):
                ratio = j / (n_arc + 1)
                ang = ang_s + (ang_e - ang_s) * ratio
                new_wps.append(arc_center + R * np.array([math.cos(ang), math.sin(ang)]))
                new_arc_set.add(len(new_wps) - 1)

            # P2 を arc_wp として追加
            new_wps.append(P2)
            new_arc_set.add(len(new_wps) - 1)

            arc_deg = math.degrees(abs(ang_e - ang_s))
            self.get_logger().info(
                f'  コーナー円弧挿入: 元WP[{i}] 転向{arc_deg:.1f}° '
                f'R={R:.2f}m 接線={t:.2f}m → {n_arc + 2}点で置換'
            )

        self.waypoints       = np.array(new_wps)
        self._corner_wp_set  = new_corner_set
        self._arc_wp_set     = new_arc_set
        # 非コーナーWPの wp_radius_map をリマップ（コーナーWPはデフォルト半径を使用）
        self._wp_radius_map  = {
            old_to_new[old]: r
            for old, r in self._wp_radius_map.items()
            if old in old_to_new and old not in new_corner_set
        }

    def _resolve_origin_and_convert(self, msg: GnssSolution) -> bool:
        ox = msg.pos_enu_org_ecef.x
        oy = msg.pos_enu_org_ecef.y
        oz = msg.pos_enu_org_ecef.z

        # gnss_ros_standardization の仕様: 基準局未確定時は (0,0,0)
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

        # WPスキップ: wp_skip_indices で指定されたWPを経路から除外し、インデックスをリマップ
        if self._skip_wp_set:
            n_total = len(self.waypoints)
            valid_skip = {i for i in self._skip_wp_set if 0 < i < n_total - 1}
            if valid_skip:
                keep = [i for i in range(n_total) if i not in valid_skip]
                old_to_new = {old: new for new, old in enumerate(keep)}
                self.waypoints = self.waypoints[np.array(keep)]
                self._corner_wp_set = {old_to_new[i] for i in self._corner_wp_set if i in old_to_new}
                self._wp_radius_map = {old_to_new[i]: r for i, r in self._wp_radius_map.items() if i in old_to_new}
                self.get_logger().info(
                    f'WPスキップ: 元WP{sorted(valid_skip)} を経路から除外 → {len(self.waypoints)}点で走行'
                )

        # コーナーWPを円弧点列に置換（最小旋回半径に基づく、C1連続パス）
        self._insert_corner_arcs()

        if self._corner_wp_set:
            for idx in sorted(self._corner_wp_set):
                self.get_logger().info(
                    f'減速WP指定: WP[{idx}] → {self.corner_slowdown_dist:.1f}m手前からランプ減速 → '
                    f'{self.corner_slowdown_speed:.1f}m/s、通過後{self.corner_slowdown_dist:.1f}mかけてランプ加速'
                )
        else:
            self.get_logger().info('減速WP指定なし → 全WPを順に追従')
        n_total = len(self.waypoints)
        for i in range(n_total):
            r = self._wp_radius_map.get(i, self.waypoint_radius)
            if i in self._wp_radius_map:
                self.get_logger().info(f'  WP[{i}] 到達半径: {r:.2f}m（個別指定）')
        if not self._wp_radius_map:
            self.get_logger().info(f'WP到達半径: 全WP共通 {self.waypoint_radius:.2f}m')

        self.get_logger().info(
            f'GNSS ENU原点確定 → Waypoint {len(self.waypoints)}点を変換完了（直線区間で接続）。'
            f'最初の目標 → X={self.waypoints[0,0]:.2f}m, Y={self.waypoints[0,1]:.2f}m'
        )
        return True

    # 線形カルマンフィルタ（定速度モデル）。状態=[x, y, vx, vy]、観測=[x, y, vx, vy]。
    # H=I のため S=P_pred+R、K=P_pred@inv(S) に簡略化している。
    def _kf_update(self, x: float, y: float, vx: float, vy: float, dt: float):
        F = np.array([[1.0, 0.0,  dt, 0.0],
                      [0.0, 1.0, 0.0,  dt],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])
        q = self.kf_process_noise ** 2
        Q = q * np.array([[dt**4 / 4, 0.0,       dt**3 / 2, 0.0      ],
                           [0.0,       dt**4 / 4, 0.0,       dt**3 / 2],
                           [dt**3 / 2, 0.0,       dt**2,     0.0      ],
                           [0.0,       dt**3 / 2, 0.0,       dt**2    ]])
        rp = self.kf_pos_noise_std ** 2
        rv = self.kf_vel_noise_std ** 2
        R = np.diag([rp, rp, rv, rv])

        if self._kf_state is None:
            self._kf_state = np.array([x, y, vx, vy])
            self._kf_P = R.copy()
            return x, y

        x_pred = F @ self._kf_state
        P_pred = F @ self._kf_P @ F.T + Q
        z = np.array([x, y, vx, vy])
        K = P_pred @ np.linalg.inv(P_pred + R)
        self._kf_state = x_pred + K @ (z - x_pred)
        self._kf_P = (np.eye(4) - K) @ P_pred
        return float(self._kf_state[0]), float(self._kf_state[1])

    def _gnss_callback(self, msg: GnssSolution):
        # status は FIX/FLOAT 判定の前に更新する（タイムアウト検出との区別のため）
        self.current_status = msg.status
        self.last_gnss_time = self.get_clock().now()
        self.last_pos_enu_cov = msg.pos_enu_cov

        if msg.status not in _VALID_STATUSES:
            return

        if self.current_status == GnssSolution.STATUS_FIX and not self.fix_achieved:
            self.fix_achieved = True
            self.get_logger().info('★ 起動後初回のFIXを達成 → 走行を開始します')

        # ENU原点未確定なら確定を試みる。確定直後の1回は制御に進まない。
        if self.waypoints is None:
            self._resolve_origin_and_convert(msg)
            return

        self.raw_x = msg.pos_enu.x
        self.raw_y = msg.pos_enu.y

        # ヘディング推定: ve/vnをEMAしてからatan2（角度を直接EMAすると±180°境界で破綻するため）
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

        # カルマンフィルタで位置平滑化（ルックアヘッド計算に使う）
        now = time.time()
        dt = min(max(now - self._kf_last_time, 0.01), 0.5) if self._kf_last_time is not None else 0.1
        self._kf_last_time = now
        kf_x, kf_y = self._kf_update(msg.pos_enu.x, msg.pos_enu.y, ve, vn, dt)
        self.current_x = kf_x
        self.current_y = kf_y

        status_str = 'FIX' if self.current_status == GnssSolution.STATUS_FIX else 'FLOAT'
        self.get_logger().info(
            f'GNSS: Status={status_str}  '
            f'raw=({self.raw_x:.2f},{self.raw_y:.2f})m  KF=({self.current_x:.2f},{self.current_y:.2f})m  '
            f'speed={self.current_speed:.2f}m/s'
        )

        self._control()

    # waypoint_index 以降の直線区間を lookahead_dist だけ辿り、線形補間で目標点を返す
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

    # Pure Pursuit 制御
    def _control(self):
        if self.current_x is None:
            self._publish_stop()
            return

        if not self.fix_achieved:
            self._publish_stop()
            return

        # ヘディング未確定時は直進して速度ベクトルを確定させる
        if self.heading is None:
            speed = max(0.0, min(self.bootstrap_speed, self.max_speed))
            self._publish_command(speed, 0.0)
            self.get_logger().info('ヘディング未確定 → 直進ブートストラップ中...')
            return

        # コーナー通過後の立ち上がり減速残距離を走行距離分だけ消費
        if self._last_ctrl_x is not None and self._corner_exit_dist_remaining > 0:
            dist_moved = math.hypot(self.current_x - self._last_ctrl_x,
                                    self.current_y - self._last_ctrl_y)
            self._corner_exit_dist_remaining = max(0.0, self._corner_exit_dist_remaining - dist_moved)
        self._last_ctrl_x = self.current_x
        self._last_ctrl_y = self.current_y

        tx0, ty0 = self.waypoints[self.waypoint_index]

        # WP到達判定は生のGNSS位置で行う（KF位置ではWP通過タイミングが遅れる）
        dist_to_wp = math.hypot(tx0 - self.raw_x, ty0 - self.raw_y)
        arrival_radius = self._wp_radius_map.get(self.waypoint_index, self.waypoint_radius)
        if dist_to_wp <= arrival_radius:
            passed_idx = self.waypoint_index
            is_arc_pt = passed_idx in self._arc_wp_set
            if not is_arc_pt:
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
                    f'停止せずWP[1]から再周回します'
                )
                self.waypoint_index = 1

            # 通過したWPがコーナー対象なら立ち上がり減速をセット
            if passed_idx in self._corner_wp_set:
                self._corner_exit_dist_remaining = self.corner_slowdown_dist
                self.get_logger().info(
                    f'コーナーWP[{passed_idx}]通過 → 立ち上がり減速 {self.corner_slowdown_dist:.1f}m 継続'
                )

            tx0, ty0 = self.waypoints[self.waypoint_index]
            self.get_logger().info(
                f'次の目標 → WP[{self.waypoint_index}] X={tx0:.2f}m, Y={ty0:.2f}m'
            )

        # 横偏差: 直前WP→目標WPのセグメントへの符号付き垂直距離（右側が正）
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

        # コーナー接近中はルックアヘッドをwp_radius以下に制限する。
        # これを超えると次区間へ先回りして振動するのを防ぐため。
        dist_to_target = math.hypot(tx0 - self.current_x, ty0 - self.current_y)
        approaching_corner = self.waypoint_index in self._corner_wp_set and dist_to_target <= self.corner_slowdown_dist
        near_sharp_corner = approaching_corner or self._corner_exit_dist_remaining > 0

        lookahead_dist = self.lookahead_min + self.lookahead_gain * self.current_speed
        corner_ld_min = self.lookahead_min + self.lookahead_gain * self.corner_slowdown_speed
        if approaching_corner:
            # コーナー手前: Ldも速度と同様にランプ縮小（急なLdジャンプによる蛇行を防ぐ）
            ratio = max(0.0, min(1.0, dist_to_target / self.corner_slowdown_dist))
            lookahead_dist = corner_ld_min + ratio * (lookahead_dist - corner_ld_min)
            # LdがWPを超えないようにキャップ: WP手前で次セグメントに先回りするのを防ぐ
            lookahead_dist = min(lookahead_dist, dist_to_target)
        elif self._corner_exit_dist_remaining > 0:
            # コーナー通過後: corner_ld_minから始めてLdをランプ拡大（小さいLdによる制御飽和・発散を防ぐ）
            ratio = max(0.0, min(1.0, self._corner_exit_dist_remaining / self.corner_slowdown_dist))
            lookahead_dist = corner_ld_min + (1.0 - ratio) * (lookahead_dist - corner_ld_min)
        tx, ty = self._lookahead_target(lookahead_dist)

        dx, dy = tx - self.current_x, ty - self.current_y
        ld_actual = math.hypot(dx, dy)
        target_bearing = math.atan2(dy, dx)

        # alpha: 目標点方位とヘディングの差（-π〜πに正規化）
        alpha = target_bearing - self.heading
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Pure Pursuit 則: δ = atan2(2L sin(α), Ld)
        if ld_actual < 1e-3:
            steer = 0.0
        else:
            steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld_actual)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        speed = self.speed_fix if self.current_status == GnssSolution.STATUS_FIX else self.speed_float
        speed = max(0.0, min(self.max_speed, speed))
        if approaching_corner:
            # コーナー手前: corner_slowdown_dist→0 にかけて speed_fix→corner_slowdown_speed へ線形ランプ
            ratio = max(0.0, min(1.0, dist_to_target / self.corner_slowdown_dist))
            speed = self.corner_slowdown_speed + ratio * (speed - self.corner_slowdown_speed)
        elif self._corner_exit_dist_remaining > 0:
            # コーナー通過後: corner_slowdown_dist→0 にかけて corner_slowdown_speed→speed_fix へ線形ランプ
            ratio = max(0.0, min(1.0, self._corner_exit_dist_remaining / self.corner_slowdown_dist))
            speed = self.corner_slowdown_speed + (1.0 - ratio) * (speed - self.corner_slowdown_speed)

        self._publish_command(speed, steer)

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

    def _safety_check(self):
        elapsed = (self.get_clock().now() - self.last_gnss_time).nanoseconds / 1e9
        if elapsed > self.gnss_timeout_s:
            # last_gnss_time はステータス不問で更新されるため、ここはメッセージ未着の場合のみ
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
