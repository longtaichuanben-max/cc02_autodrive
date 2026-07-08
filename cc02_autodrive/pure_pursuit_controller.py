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


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info('Pure Pursuit Controller Node has been started!')

        self.declare_parameter('wp_file', 'wp_position_basic.csv')     # 走行するウェイポイントCSVファイルのパス
        self.declare_parameter('wp_radius_max', 2.0)                    # WP到達半径の上限 [m]：直線WP（偏向角0°）に適用される最大値
        self.declare_parameter('wp_radius_min', 0.5)                    # WP到達半径の下限 [m]：急コーナーや短セグメントでもこれ以下にはならない
        self.declare_parameter('wp_radius_seg_ratio', 0.3)              # WP到達半径のセグメント長制約：隣接セグメント長×この値を上限とする（スキップ防止）
        self.declare_parameter('speed_min', 1.0)                        # 最低走行速度 [m/s]：最も短いセグメントに割り当てられる速度
        self.declare_parameter('speed_max', 3.0)                        # 最高走行速度 [m/s]：最も長いセグメントに割り当てられる速度
        self.declare_parameter('speed_dist_short', 5.0)                 # speed_minを割り当てるセグメント長の上限 [m]：これ以下のセグメントはすべてspeed_min
        self.declare_parameter('speed_dist_long', 10.0)                 # speed_maxを割り当てるセグメント長の下限 [m]：これ以上のセグメントはすべてspeed_max
        self.declare_parameter('wheelbase_m', 0.267)                    # 前後車軸間距離 [m]：Pure Pursuit操舵角計算に使用（実車体から実測値）
        self.declare_parameter('lookahead_min', 2.0)                    # ルックアヘッド距離の最小値 [m]：速度がゼロでもこの距離だけ先を目標とする
        self.declare_parameter('lookahead_fraction', 0.5)               # ルックアヘッド距離のセグメント長に対する割合：Ld = lookahead_min + T_h × speed を導く係数
        self.declare_parameter('max_steering_angle', math.radians(25.0))  # 操舵角の上限 [rad]：サーボの物理限界に合わせて設定
        self.declare_parameter('bootstrap_speed', 0.5)                  # ヘディング未確定時の直進速度 [m/s]：起動直後にGNSS速度ベクトルが得られるまでの仮走行速度
        self.declare_parameter('min_speed_for_heading', 0.05)           # ヘディング推定に使う最低速度 [m/s]：これ未満の速度ベクトルはノイズとみなして無視
        self.declare_parameter('heading_smoothing_w', 0.35)             # ヘディングの指数移動平均の重み（0〜1）：大きいほど最新値に追従、小さいほど平滑化
        self.declare_parameter('max_speed_mps', 4.0)                    # 速度コマンドの絶対上限 [m/s]：いかなる計算結果もこれを超えない
        self.declare_parameter('gnss_timeout_s', 2.0)                   # GNSSデータが途絶えたとみなすタイムアウト時間 [s]：超過で安全停止
        self.declare_parameter('corner_angle_thresh_deg', 30.0)         # コーナー自動検出の偏向角閾値 [deg]：これ以上の偏向角を持つWPをコーナーと判定
        self.declare_parameter('lh_ramp_angle_thresh_deg', 50.0)        # lh_ramp自動検出の偏向角閾値 [deg]：コーナー出口でルックアヘッドを徐々に伸ばす対象
        self.declare_parameter('corner_slowdown_ratio', 0.6)            # コーナー通過速度の割合：コーナーWP通過時の速度 = セグメント速度 × この値
        self.declare_parameter('corner_slowdown_base_dist', 10.0)       # 減速・加速ランプの基準距離 [m]：speed_minセグメントでのランプ距離、速度に比例してスケール
        default_log = os.path.join(
            os.path.expanduser('~'), 'ros2_ws', 'gnss_logs', 'pure_pursuit_log_latest.csv'
        )
        self.declare_parameter('tuning_log_file', default_log)          # チューニング評価ログの出力先CSVパス

        wp_file                        = self.get_parameter('wp_file').value
        self._wp_radius_max            = self.get_parameter('wp_radius_max').value
        self._wp_radius_min            = self.get_parameter('wp_radius_min').value
        self._wp_radius_seg_ratio      = self.get_parameter('wp_radius_seg_ratio').value
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
        self._corner_angle_thresh_deg  = self.get_parameter('corner_angle_thresh_deg').value
        self._lh_ramp_angle_thresh_deg = self.get_parameter('lh_ramp_angle_thresh_deg').value
        self.corner_slowdown_ratio     = self.get_parameter('corner_slowdown_ratio').value
        self.corner_slowdown_base_dist = self.get_parameter('corner_slowdown_base_dist').value
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
        self._corner_wp_set: set = set()
        self._lh_ramp_wp_set: set = set()
        self._wp_radius_map: dict = {}

        self.current_x      = None
        self.current_y      = None
        self.current_speed  = 0.0
        self.heading        = None
        self._ve_filtered   = 0.0
        self._vn_filtered   = 0.0
        self.current_status = 0
        self.fix_achieved   = False

        self._gnss_pos_x    = None   # 受信機が出力した最新の位置（ENU X）
        self._gnss_pos_y    = None   # 受信機が出力した最新の位置（ENU Y）
        self._gnss_recv_time = None  # 最新GNSSメッセージを受け取ったROSタイム
        self._vel_e         = 0.0   # 最新の東方向速度 [m/s]（外挿用）
        self._vel_n         = 0.0   # 最新の北方向速度 [m/s]（外挿用）

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
        self.create_timer(0.02, self._control_timer)  # 50Hz制御タイマー

        self.get_logger().info('pure_pursuit_controller 起動完了（GNSS ENU原点確定待ち）')

    # ================================================================
    # 1. ウェイポイント管理
    # ================================================================
    #　WPファイルを読み込む関数、そしてwaypointsというリストで返す
    def _load_waypoints_llh(self, filepath: str) -> list:
        """CSVファイルから(lat, lon, height)のリストを読み込んで返す。"""
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
    #　ENU座標系：地球上のある1点を原点として「東・北・上に何メートルか」で位置を表すローカルな座標系。
    #　ECEF座標系：地球の重心を原点として全地球を1つの座標系で表す。単位はメートル。
    #　ECEF座標系をpm.ecef2geodetic(ox, oy, oz)で緯度経度にし、返している
    def _get_enu_origin(self, msg: GnssSolution):
        """ENU原点のECEF座標を検証し(lat, lon, alt)を返す。未確定ならNoneを返す。"""
        ox, oy, oz = msg.pos_enu_org_ecef.x, msg.pos_enu_org_ecef.y, msg.pos_enu_org_ecef.z
        if ox == 0.0 and oy == 0.0 and oz == 0.0:
            return None
        return pm.ecef2geodetic(ox, oy, oz)
    #　全WPのLLH座標系（緯度経度高さ）をENU座標系基準にする。また、高さは使わない
    def _convert_llh_to_enu(self, origin_lat: float, origin_lon: float,
                             origin_alt: float) -> np.ndarray:
        """全WPのLLH座標をENU平面座標(x, y)に変換してndarrayで返す。"""
        llh_arr = np.array(self.wps_llh)
        e, n, _ = pm.geodetic2enu(
            llh_arr[:, 0], llh_arr[:, 1], llh_arr[:, 2],
            origin_lat, origin_lon, origin_alt
        )
        return np.column_stack([e, n])
    #　起動時にセグメントやWPごとの設定を行う関数
    def _log_route_info(self):
        #　コーナーの設定
        if self._corner_wp_set:
            for idx in sorted(self._corner_wp_set):
                seg_idx    = (idx - 1) % len(self._seg_speeds)
                approach_v = self._seg_speeds[seg_idx]
                corner_v   = approach_v * self.corner_slowdown_ratio
                corner_d   = self.corner_slowdown_base_dist * (approach_v / self.speed_min)
                self.get_logger().info(
                    f'  コーナーWP[{idx}]  '
                    f'減速 {corner_d:.1f}m手前 {approach_v:.2f}→{corner_v:.2f}m/s'
                )
        else:
            self.get_logger().info('コーナーWP検出なし → 全WPを順に追従')
        #　到達半径の設定
        overrides = ', '.join(
            f'WP[{i}]={r:.2f}m' for i, r in sorted(self._wp_radius_map.items())
        )
        self.get_logger().info(f'WP到達半径（自動）: {overrides}')
        #　ENU変換完了と最初の目標WP
        self.get_logger().info(
            f'GNSS ENU原点確定 → Waypoint {len(self.waypoints)}点変換完了。'
            f'最初の目標 → X={self.waypoints[0, 0]:.2f}m, Y={self.waypoints[0, 1]:.2f}m'
        )
    #　3つの関数を決まった順序で呼ぶ関数
    def _initialize_route(self, msg: GnssSolution):
        """ENU原点確定後にルートを初期化する。
        原点解決 → ENU変換 → セグメントパラメータ計算 → WP自動解析 → ログ出力
        """
        origin = self._get_enu_origin(msg)
        if origin is None:
            return
        self.waypoints = self._convert_llh_to_enu(*origin)
        self._compute_segment_params()
        self._detect_corner_wps()
        self._log_route_info()
    #　どこからでも走行開始できるように現在地と各WPの差を計算して最も近いインデックスを返す
    def _nearest_wp_index(self) -> int:
        """現在位置に最も近いWPのインデックスを返す。
        途中スタートやリカバリ時に waypoint_index を現在地に合わせるために使う。
        """
        dists = np.hypot(
            self.waypoints[:, 0] - self.current_x,
            self.waypoints[:, 1] - self.current_y
        )
        return int(np.argmin(dists))
    #　到達したWPの判定をして、到達している場合は次のWPのENUをかえす。
    def _check_wp_arrival(self, tx0: float, ty0: float) -> tuple:
        """現在WPへの到達を判定し、到達していればインデックスを更新する。
        Returns:
            arrived  (bool)  : WPに到達したか
            passed_idx (int) : 通過したWPのインデックス
            dist (float)     : WPまでの距離 [m]
            new_tx0 (float)  : 更新後の目標WP X座標
            new_ty0 (float)  : 更新後の目標WP Y座標
        """
        dist = math.hypot(tx0 - self.current_x, ty0 - self.current_y)
        radius = self._wp_radius_map.get(self.waypoint_index, self._wp_radius_min)
        #　半径より距離が大きいのでまだ到達していないとき
        if dist > radius:
            return False, self.waypoint_index, dist, tx0, ty0

        passed_idx = self.waypoint_index
        self.score += 10
        self.get_logger().info(
            f'★ WP[{passed_idx}] 通過！ (到達距離={dist:.2f}m) '
            f'ゲート通過 +10点 → 合計{self.score}点'
        )

        self.waypoint_index += 1
        #　すべてのWPを到達したときそれを記録して、再周回するためにインデックスを0に
        if self.waypoint_index >= len(self.waypoints):
            self.score += 50
            self.get_logger().info(
                f'★★★ 1周完了！ 周回ボーナス +50点 → 合計{self.score}点 '
                f'停止せずWP[0]から再周回します'
            )
            self.waypoint_index = 0

        new_tx0, new_ty0 = self.waypoints[self.waypoint_index]
        self.get_logger().info(
            f'次の目標 → WP[{self.waypoint_index}] X={new_tx0:.2f}m, Y={new_ty0:.2f}m'
        )

        return True, passed_idx, dist, float(new_tx0), float(new_ty0)

    # ================================================================
    # 2. セグメントパラメータ計算
    # ================================================================

    def _compute_segment_params(self):
        speeds, gains = [], []
        d_range = self.speed_dist_long - self.speed_dist_short

        self.get_logger().info('--- セグメントパラメータ ---')
        for i, ((e1, n1), (e2, n2)) in enumerate(
            zip(self.waypoints, np.roll(self.waypoints, -1, axis=0))
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

    def _detect_corner_wps(self):
        """セグメント間の偏向角からコーナーWP・lh_ramp WP・到達半径を自動検出する。"""
        n = len(self.waypoints)
        corner_set = set()
        lh_ramp_set = set()
        radius_map = {}
        self.get_logger().info(
            f'--- WP自動解析 '
            f'(corner≥{self._corner_angle_thresh_deg:.0f}°, '
            f'lh_ramp≥{self._lh_ramp_angle_thresh_deg:.0f}°, '
            f'r_min={self._wp_radius_min:.1f}m r_max={self._wp_radius_max:.1f}m) ---'
        )
        for i in range(n):
            v_in  = self.waypoints[i] - self.waypoints[(i - 1) % n]
            v_out = self.waypoints[(i + 1) % n] - self.waypoints[i]
            len_in  = np.linalg.norm(v_in)
            len_out = np.linalg.norm(v_out)
            if len_in < 0.1 or len_out < 0.1:
                self.get_logger().info(f'  WP[{i}] スキップ（ほぼ同位置）')
                continue
            cos_a = float(np.clip(np.dot(v_in, v_out) / (len_in * len_out), -1.0, 1.0))
            deg = math.degrees(math.acos(cos_a))
            tag = ''
            if deg >= self._corner_angle_thresh_deg:
                corner_set.add(i)
                tag += ' ← コーナー'
            if deg >= self._lh_ramp_angle_thresh_deg:
                lh_ramp_set.add(i)
                tag += ' [lh_ramp]'
            r_angle = self._wp_radius_max * (1.0 - deg / 180.0)
            r_seg   = min(len_in, len_out) * self._wp_radius_seg_ratio
            r = max(self._wp_radius_min, min(r_angle, r_seg))
            radius_map[i] = r
            tag += f'  r={r:.2f}m'
            self.get_logger().info(f'  WP[{i}] 偏向角={deg:.1f}°{tag}')
        self._corner_wp_set  = corner_set
        self._lh_ramp_wp_set = lh_ramp_set
        self._wp_radius_map  = radius_map

    # ================================================================
    # 3. GNSSコールバック・自己位置推定
    # ================================================================

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
            self._initialize_route(msg)
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

        self._vel_e = ve
        self._vel_n = vn
        self._gnss_pos_x = msg.pos_enu.x
        self._gnss_pos_y = msg.pos_enu.y
        self._gnss_recv_time = self.get_clock().now()

        self.get_logger().info(
            f'GNSS: Status={"FIX" if self.current_status == GnssSolution.STATUS_FIX else "FLOAT"}  '
            f'pos=({self._gnss_pos_x:.2f},{self._gnss_pos_y:.2f})m  '
            f'speed={self.current_speed:.2f}m/s'
        )

    # ================================================================
    # 4. 50Hz制御タイマー：FIX時はGNSS位置そのまま、FLOAT時は速度外挿
    # ================================================================

    def _control_timer(self):
        """50Hzで呼ばれる制御タイマー。
        FIX  : 受信機の最新位置をそのまま使う（10Hzで更新される値を5回使いまわす）
        FLOAT: 最後に受け取ったGNSS位置から vel_enu で外挿して位置を補間する
        """
        if self._gnss_pos_x is None or self._gnss_recv_time is None:
            return

        if self.current_status == GnssSolution.STATUS_FIX:
            self.current_x = self._gnss_pos_x
            self.current_y = self._gnss_pos_y
        else:
            dt = (self.get_clock().now() - self._gnss_recv_time).nanoseconds / 1e9
            dt = min(dt, 0.5)  # 外挿の暴走防止（最大0.5秒）
            self.current_x = self._gnss_pos_x + self._vel_e * dt
            self.current_y = self._gnss_pos_y + self._vel_n * dt

        self._control()

    # ================================================================
    # 5. Pure Pursuit ステアリング・速度制御
    # ================================================================

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
        if not self._seg_speeds:
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

        arrived, passed_idx, _, tx0, ty0 = self._check_wp_arrival(tx0, ty0)
        if arrived:
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

    # ================================================================
    # 6. 安全系
    # ================================================================

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
