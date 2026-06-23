import math
import csv
import os
import pymap3d as pm
import rclpy #ROS2の機能をPythonで使うための超巨大な道具箱(raspberry piの中でデータの送受信が可能に！)
from rclpy.node import Node#道具箱（rclpy）の中から、特に重要な Node（ノード＝プログラムの本体になる部品） という道具をピンポイントで取り出している
from ackermann_msgs.msg import AckermannDriveStamped  # 追加：自動運転の標準的な「手足」の命令メッセージ
from gnss_ros_standardization.msg import GnssSolution  #GNSSデータのメッセージ

# このプロジェクトで自動運転に使う十分な精度とみなすStatus
_VALID_STATUSES = (GnssSolution.STATUS_FIX, GnssSolution.STATUS_FLOAT)

class PidController(Node):#[PID制御のノード]という新しいクラスを定義している。Nodeクラスを継承しているので、Nodeの機能も使えるようになる。親
    def __init__(self):#クラスの初期化関数。クラスが呼び出されたときに最初に実行される特別な関数。(1回だけ実行される)
        super().__init__('pid_controller')#Nodeクラスの初期化関数を呼び出している。引数の 'pid_controller' は、このノードの名前になる。子供
        self.get_logger().info('PID Controller Node has been started!')#このノードが起動したときに、ログに「PID Controller Node has been started!」というメッセージを表示する。self.get_logger()は、このノード専用のロガー（ログを記録する道具）を取得するための関数。info()は、そのロガーを使って情報レベルのログメッセージを出力するための関数。

        #パラメータの宣言（ROS2のパラメータサーバーに宣言）
        self.declare_parameter('wp_file', 'wp_position.csv')#ROS2のパラメータを宣言している。パラメータ名は 'waypoint_file'、デフォルト値は 'waypoints.csv' になる。
        self.declare_parameter('wp_radius', 1.0)                # m: この距離以内でWP到達とみなす
        self.declare_parameter('speed_fix',   2.0)              # m/s: RTK-FIX時の速度
        self.declare_parameter('speed_float', 1.5)              # m/s: RTK-FLOAT時の速度
        self.declare_parameter('kp_gain',    1.0)               # ステアリングPIDゲイン（比例）
        self.declare_parameter('ki_gain',    0.0)               # ステアリングPIDゲイン（積分）
        self.declare_parameter('kd_gain',    0.1)               # ステアリングPIDゲイン（微分）
        self.declare_parameter('max_steering_angle', 0.5)       # rad: ステアリング最大角（≈28.6°）
        self.declare_parameter('bootstrap_speed', 0.1)         # 方位を確定させるために、最初の数秒間はこの速度で走行する
        self.declare_parameter('min_speed_for_heading', 0.1)    # m/s: この速度以上でvel_enuのヘディングを信頼する。要するにドップラーノイズのフィルタリング
        self.declare_parameter('max_speed_mps', 2.0)            # m/s: 速度の安全上限（誤設定時の暴走防止）後で再設定
        self.declare_parameter('derivative_filter_alpha', 0.2)  # 微分項ローパスフィルタ係数（小さいほど滑らか）
        self.declare_parameter('gnss_timeout_s', 2.0)           # 秒: は基準局RTCM補正が1Hzのため、0.5秒で毎周期引っかかる
        #wp_fileはwaypointファイルの読み込みにしか使わない
        wp_file                     = self.get_parameter('wp_file').value
        #self.を付けることでクラスの中でいつでも使える共通の変数になる。self.を付けないと、関数の中でしか使えないローカル変数になる。
        self.waypoint_radius         = self.get_parameter('wp_radius').value
        self.speed_fix              = self.get_parameter('speed_fix').value
        self.speed_float            = self.get_parameter('speed_float').value
        self.kp                     = self.get_parameter('kp_gain').value
        self.ki                     = self.get_parameter('ki_gain').value
        self.kd                     = self.get_parameter('kd_gain').value
        self.max_steer              = self.get_parameter('max_steering_angle').value
        self.bootstrap_speed        = self.get_parameter('bootstrap_speed').value
        self.min_speed_for_heading  = self.get_parameter('min_speed_for_heading').value
        self.max_speed              = self.get_parameter('max_speed_mps').value
        self.deriv_alpha            = self.get_parameter('derivative_filter_alpha').value
        self.gnss_timeout_s          = self.get_parameter('gnss_timeout_s').value
        #waypointファイルの読み込み
        self.wps_llh             = self._load_waypoints_llh(wp_file)
        #waypointファイルが読み込めなかった場合のエラー処理（暴走防止）
        if not self.wps_llh:
            self.get_logger().error(
                f'Waypointを読み込めませんでした: {wp_file} '
                '-- ファイルパスと形式(WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）を確認してください'
            )
            raise SystemExit(1)
        #waypointファイルが読み込めた場合のログ出力
        self.get_logger().info(f'Waypoint {len(self.wps_llh)}点 読み込み完了: {wp_file}')

        self.waypoints = None  # ENU変換後の(x, y)リスト。原点確定後にセットされる
        self.waypoint_index = 0
        self.origin_ecef = None      # GNSS ENU原点（ECEF）。最初の有効Fixで一度だけ確定

        #状態変数の初期化
        self.current_x       = None# 現在地 ENU-X [m]
        self.current_y       = None# 現在地 ENU-Y [m]
        self.heading         = None# 進行方向 [rad]（東=0, 北=π/2）
        self.current_status  = 0   # GNSSステータス
        #PID用
        self.integral_error = 0.0#積分誤差の初期化
        self.filtered_deriv = 0.0#微分誤差の初期化（ローパスフィルタ用）
        self.prev_error     = 0.0#前回の誤差の初期化
        self.prev_time      = None#前回の時間の初期化
        #安全停止用
        #self.clock().now()は、ROS2のノードが持っている時計から現在の時間を取得するための関数。これを使って、最後にGNSSデータを受け取った時間を記録しておくことで、一定時間以上GNSSデータが更新されない場合に安全停止するための処理を実装する
        self.last_gnss_time = self.get_clock().now()
        #走行開始フラグ。RTK FIXに初めて到達した時点でTrueになる（手動トリガーなし）
        self.is_running = False
        #Publisher/Subscriber/Timer
        self.cmd_pub        = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)#self.create_publisher(送信するデータの「言語（型）」, 送信先のトピック名, キューサイズ)
        self.gnss_sub       = self.create_subscription(GnssSolution,'/gnss/solution',self._gnss_callback,10)#self.create_subscription(受信するデータの「言語（型）」, 受信するトピック名, 受信したときに呼び出す関数, キューサイズ)
        #GNSSがgnss_timeout_s秒以上途絶えたら安全停止
        self.create_timer(0.1, self._safety_check)#self.create_timer(周期, 呼び出す関数)
        #このノードの起動が完了したことをログに出力する
        self.get_logger().info('pid_controller 起動完了（GNSS ENU原点確定待ち）')
    #Waypointファイル読み込み（緯度経度のみ、ENU変換は行わない）
    def _load_waypoints_llh(self, filepath: str) -> list:
        waypoints = []
        if not os.path.exists(filepath):
            self.get_logger().error(f'ファイルが存在しません: {filepath}')
            return waypoints
        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)#ヘッダー行をキーとする辞書形式でCSVを読み込むためのクラス。これを使うと、CSVの各行が辞書として扱えるようになる。例えば、row['Latitude(deg)']のようにして、'Latitude(deg)'というヘッダーの列の値を取得できる。
                for i, row in enumerate(reader):
                    lat = float(row['Latitude(deg)'])
                    lon = float(row['Longitude(deg)'])
                    height = float(row['Ellipsoidal Height(m)'])
                    waypoints.append((lat, lon, height))#waypointsリストに、(lat, lon, height)のタプルを追加している。これで、CSVファイルから読み込んだ各行の緯度、経度、高度がwaypointsリストに格納されることになる。
                    self.get_logger().debug(f'  WP[{i}]: lat={lat:.8f}, lon={lon:.8f}, h={height:.2f}')
        except Exception as e:
            self.get_logger().error(f'Waypoint読み込みエラー: {e}')
            return []
        return waypoints

    #GNSS原点確定とENU変換
    def _resolve_origin_and_convert(self, msg: GnssSolution) -> bool:
        ox = msg.pos_enu_org_ecef.x
        oy = msg.pos_enu_org_ecef.y
        oz = msg.pos_enu_org_ecef.z

        # gnss_ros_standardization側の仕様: 原点（基準局位置）未確定時は(0,0,0)が入る
        if ox == 0.0 and oy == 0.0 and oz == 0.0:
            return False

        self.origin_ecef = (ox, oy, oz)
        origin_lat, origin_lon, origin_alt = pm.ecef2geodetic(ox, oy, oz)

        converted = []
        for lat_deg, lon_deg, height in self.wps_llh:
            e, n, _u = pm.geodetic2enu(lat_deg, lon_deg, height, origin_lat, origin_lon, origin_alt)
            converted.append((float(e), float(n)))

        self.waypoints = converted
        self.get_logger().info(
            f'GNSS ENU原点確定 → Waypoint {len(self.waypoints)}点を変換完了。'
            f'最初の目標 → X={self.waypoints[0][0]:.2f}m, Y={self.waypoints[0][1]:.2f}m'
        )
        return True

    #GNSSコールバック
    def _gnss_callback(self, msg: GnssSolution):
        # FIX/FLOAT以外（無効・SPP・SBAS・DGPS等）は精度不足として無視する
        if msg.status not in _VALID_STATUSES:
            return

        self.current_status = msg.status
        self.last_gnss_time = self.get_clock().now()

        # ENU原点がまだ確定していなければ、このFixで確定を試みる。
        # 変換した直後の1回はcontrolに進まず、次のFixから走行を開始する。
        if self.waypoints is None:
            self._resolve_origin_and_convert(msg)
            return

        self.current_x = msg.pos_enu.x
        self.current_y = msg.pos_enu.y

        # ヘディング = 速度ベクトルのCourse over Ground（vel_enu由来）。
        # 位置の差分(dead-reckoning)よりGNSSノイズに強く、停車中の振動の影響を受けにくい。
        ve, vn = msg.vel_enu.x, msg.vel_enu.y
        speed = math.hypot(ve, vn)
        if speed >= self.min_speed_for_heading:
            self.heading = math.atan2(vn, ve)

        self.get_logger().debug(
            f'GNSS: X={self.current_x:.2f}m  Y={self.current_y:.2f}m  '
            f'Status={self.current_status}  speed={speed:.2f}m/s'
        )

        # RTK FIXに初めて到達した時点で自動的に走行を開始する（手動トリガーなし）
        if not self.is_running:
            if self.current_status == GnssSolution.STATUS_FIX:
                self.is_running = True
                self.integral_error = 0.0
                self.filtered_deriv = 0.0
                self.prev_error = 0.0
                self.prev_time = None
                self.get_logger().info('▶️ RTK FIXに到達 → 走行開始')
            else:
                self._publish_stop()
                return

        # 全Waypoint完了チェック
        if self.waypoint_index >= len(self.waypoints):
            self.get_logger().info('全Waypoint到達！停止します。')
            self._publish_stop()
            return

        # 現在Waypointへの距離チェック（到達判定）
        tx, ty = self.waypoints[self.waypoint_index]
        dist = math.sqrt((tx - self.current_x) ** 2 + (ty - self.current_y) ** 2)

        if dist < self.waypoint_radius:
            self.get_logger().info(
                f'WP[{self.waypoint_index}] 到達（残り{dist:.2f}m）→ 次のWPへ'
            )
            self.waypoint_index += 1
            self.integral_error = 0.0  # WP切替時に積分項をリセット
            self.prev_error = 0.0

            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info('全Waypoint到達！停止します。')
                self._publish_stop()
                return

        self._control()

    #PID制御
    def _control(self):
        if self.current_x is None:
            self._publish_stop()
            return

        # 【Catch-22対策】ヘディング未確定時は停止せず、直進ブートストラップを行う。
        # ヘディングはvel_enu（速度ベクトル）からしか求められない（IMU非搭載のため）。
        # 停止し続けると速度が常に0になり、ヘディングが永久に確定しない。
        # steer=0で低速直進することで速度ベクトルを発生させ、ヘディングを確定させる。
        if self.heading is None:
            speed = max(0.0, min(self.bootstrap_speed, self.max_speed))
            self._publish_command(speed, 0.0)
            self.get_logger().info('ヘディング未確定 → 直進ブートストラップ中...')
            return

        tx, ty = self.waypoints[self.waypoint_index]

        # 目標方位（現在地 → Waypoint の角度）
        bearing = math.atan2(ty - self.current_y, tx - self.current_x)

        # 方位誤差（-π〜πに正規化）
        error = bearing - self.heading
        error = math.atan2(math.sin(error), math.cos(error))

        # dt（前回コールバックからの経過時間）
        now = self.get_clock().now()
        if self.prev_time is None:
            # 初回はI/D項を計算せずP項のみで応答する（dtが無いため）
            derivative = 0.0
        else:
            dt = (now - self.prev_time).nanoseconds / 1e9
            dt = max(dt, 0.001)  # ゼロ除算防止

            # 積分（ワインドアップ防止: -π〜πにクランプ）
            self.integral_error = max(-math.pi, min(math.pi, self.integral_error + error * dt))

            # 微分 + 一次ローパスフィルタ（GNSSノイズによる急激な増幅を抑制）
            raw_derivative = (error - self.prev_error) / dt
            self.filtered_deriv = (
                self.deriv_alpha * raw_derivative + (1.0 - self.deriv_alpha) * self.filtered_deriv
            )
            derivative = self.filtered_deriv

        # ステアリング角 = P + I + D（最大角でクランプ）
        steer = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        steer = max(-self.max_steer, min(self.max_steer, steer))

        self.prev_error = error
        self.prev_time  = now

        # 速度: FIX(=1)は高速、FLOAT(=2)は低速（安全上限でクランプ）
        speed = self.speed_fix if self.current_status == GnssSolution.STATUS_FIX else self.speed_float
        speed = max(0.0, min(self.max_speed, speed))

        self._publish_command(speed, steer)

        self.get_logger().debug(
            f'WP[{self.waypoint_index}] '
            f'誤差={math.degrees(error):+.1f}°  '
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
    rclpy.init(args=args)#ros2の通信エンジンを始動。
    node = PidController()#ラズパイのメモリ上でノードを本当に起動する。
    rclpy.spin(node)#この関数は、ROS2のイベントループを開始するための関数。引数にノードを渡すと、そのノードがROS2のイベントループに参加することになる。これによって、ノードはROS2の通信機能を使ってデータの送受信ができるようになる。
    node.destroy_node()#メモリや通信ポートを綺麗にお掃除して、安全にプログラムを終了させるためのお約束のコード。
    rclpy.shutdown()#ROS2の通信エンジンを安全にシャットダウンするためのお約束のコード。

if __name__ == '__main__':
    main()
