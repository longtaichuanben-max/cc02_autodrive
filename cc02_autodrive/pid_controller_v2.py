"""
pid_controller_v2.py

pid_controller.py からの主な改良点:
  - Status=0（無効GNSSデータ）のフィルタリング
  - CSVファイルからWaypointを読み込む（教授が測った座標に対応）
  - PID制御ロジックの実装（方位誤差 → ステアリング角）
  - FIX（Status=1）とFLOAT（Status=2）で速度を切り替え
  - GNSS信号が途絶えたときの安全停止
  - ROS2パラメータでPIDゲイン・速度・ファイルパスを実行時に変更可能

Waypointファイル形式（CSV）:
  x,y
  10.0,0.0
  10.0,10.0
  ...
  ※ /gnss/solution の pos_enu と同じENU座標系（単位: m）
"""

import math
import csv
import os

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from gnss_ros_standardization.msg import GnssSolution


class PidControllerV2(Node):

    def __init__(self):
        super().__init__('pid_controller_v2')

        # -------------------------------------------------------
        # ROS2パラメータ（ros2 run時に --ros-args -p key:=value で上書き可能）
        # -------------------------------------------------------
        self.declare_parameter('waypoint_file', 'waypoints.csv')
        self.declare_parameter('waypoint_radius', 1.0)     # m: この距離以内でWP到達とみなす
        self.declare_parameter('speed_fix',   0.5)         # m/s: RTK-FIX時の速度
        self.declare_parameter('speed_float', 0.3)         # m/s: RTK-FLOAT時の速度
        self.declare_parameter('kp_steer',    1.0)         # ステアリングPIDゲイン（比例）
        self.declare_parameter('ki_steer',    0.0)         # ステアリングPIDゲイン（積分）
        self.declare_parameter('kd_steer',    0.1)         # ステアリングPIDゲイン（微分）
        self.declare_parameter('max_steering_angle', 0.5)  # rad: ステアリング最大角（≈28.6°）

        waypoint_file        = self.get_parameter('waypoint_file').value
        self.waypoint_radius = self.get_parameter('waypoint_radius').value
        self.speed_fix       = self.get_parameter('speed_fix').value
        self.speed_float     = self.get_parameter('speed_float').value
        self.kp              = self.get_parameter('kp_steer').value
        self.ki              = self.get_parameter('ki_steer').value
        self.kd              = self.get_parameter('kd_steer').value
        self.max_steer       = self.get_parameter('max_steering_angle').value

        # -------------------------------------------------------
        # Waypointファイル読み込み
        # -------------------------------------------------------
        self.waypoints = self._load_waypoints(waypoint_file)
        self.waypoint_index = 0

        if not self.waypoints:
            self.get_logger().error(
                f'Waypointを読み込めませんでした: {waypoint_file} '
                '-- ファイルパスと形式（x,y のCSV）を確認してください'
            )
            return

        self.get_logger().info(f'Waypoint {len(self.waypoints)}点 読み込み完了: {waypoint_file}')
        self.get_logger().info(
            f'最初の目標 → X={self.waypoints[0][0]:.2f}m, Y={self.waypoints[0][1]:.2f}m'
        )

        # -------------------------------------------------------
        # 状態変数
        # -------------------------------------------------------
        self.current_x      = None   # 現在地 ENU-X [m]
        self.current_y      = None   # 現在地 ENU-Y [m]
        self.heading        = None   # 進行方向 [rad]（東=0, 北=π/2）
        self.current_status = 0      # GNSSステータス

        # PID用
        self.integral_error = 0.0
        self.prev_error     = 0.0
        self.prev_time      = None

        # 安全停止用
        self.last_gnss_time = self.get_clock().now()

        # -------------------------------------------------------
        # Publisher / Subscriber / Timer
        # -------------------------------------------------------
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

        self.gnss_sub = self.create_subscription(
            GnssSolution,
            '/gnss/solution',
            self._gnss_callback,
            10
        )

        # GNSSが0.5秒以上途絶えたら安全停止
        self.create_timer(0.5, self._safety_check)

        self.get_logger().info('PID Controller V2 起動完了')

    # -----------------------------------------------------------
    # Waypointファイル読み込み
    # -----------------------------------------------------------
    def _load_waypoints(self, filepath: str) -> list:
        waypoints = []
        if not os.path.exists(filepath):
            self.get_logger().error(f'ファイルが存在しません: {filepath}')
            return waypoints
        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)
                for i, row in enumerate(reader):
                    x = float(row['x'])
                    y = float(row['y'])
                    waypoints.append((x, y))
                    self.get_logger().debug(f'  WP[{i}]: X={x:.2f}, Y={y:.2f}')
        except Exception as e:
            self.get_logger().error(f'Waypoint読み込みエラー: {e}')
        return waypoints

    # -----------------------------------------------------------
    # GNSSコールバック
    # -----------------------------------------------------------
    def _gnss_callback(self, msg: GnssSolution):
        # Status=0（無効データ）は無視する
        if msg.status == 0:
            return

        self.current_status = msg.status
        self.last_gnss_time = self.get_clock().now()

        new_x = msg.pos_enu.x
        new_y = msg.pos_enu.y

        # 進行方向（ヘディング）を前回位置との差分から推定
        if self.current_x is not None:
            dx = new_x - self.current_x
            dy = new_y - self.current_y
            moved = math.sqrt(dx**2 + dy**2)
            if moved > 0.05:  # 5cm以上動いたときだけ更新（GNSSノイズ除去）
                self.heading = math.atan2(dy, dx)

        self.current_x = new_x
        self.current_y = new_y

        self.get_logger().debug(
            f'GNSS: X={self.current_x:.2f}m  Y={self.current_y:.2f}m  Status={self.current_status}'
        )

        # 全Waypoint完了チェック
        if self.waypoint_index >= len(self.waypoints):
            self.get_logger().info('全Waypoint到達！停止します。')
            self._publish_stop()
            return

        # 現在Waypointへの距離チェック（到達判定）
        tx, ty = self.waypoints[self.waypoint_index]
        dist = math.sqrt((tx - self.current_x)**2 + (ty - self.current_y)**2)

        if dist < self.waypoint_radius:
            self.get_logger().info(
                f'✅ WP[{self.waypoint_index}] 到達（残り{dist:.2f}m）→ 次のWPへ'
            )
            self.waypoint_index += 1
            self.integral_error = 0.0  # WP切替時に積分項をリセット
            self.prev_error = 0.0

            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info('全Waypoint到達！停止します。')
                self._publish_stop()
                return

        self._control()

    # -----------------------------------------------------------
    # PID制御
    # -----------------------------------------------------------
    def _control(self):
        # ヘディング未確定（まだ動き出していない）は停止
        if self.current_x is None or self.heading is None:
            self._publish_stop()
            return

        tx, ty = self.waypoints[self.waypoint_index]

        # 目標方位（現在地 → Waypoint の角度）
        bearing = math.atan2(ty - self.current_y, tx - self.current_x)

        # 方位誤差（-π〜πに正規化）
        error = bearing - self.heading
        error = math.atan2(math.sin(error), math.cos(error))

        # dt（前回コールバックからの経過時間）
        now = self.get_clock().now()
        dt = 0.1  # デフォルト（初回）
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            dt = max(dt, 0.001)  # ゼロ除算防止

        # 積分（ワインドアップ防止: -π〜πにクランプ）
        self.integral_error = max(-math.pi, min(math.pi, self.integral_error + error * dt))

        # 微分
        derivative = (error - self.prev_error) / dt

        # ステアリング角 = P + I + D（最大角でクランプ）
        steer = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        steer = max(-self.max_steer, min(self.max_steer, steer))

        self.prev_error = error
        self.prev_time  = now

        # 速度: FIX(=1)は高速、FLOAT(=2)は低速
        speed = self.speed_fix if self.current_status == 1 else self.speed_float

        self._publish_command(speed, steer)

        self.get_logger().info(
            f'WP[{self.waypoint_index}] '
            f'誤差={math.degrees(error):+.1f}°  '
            f'ステア={math.degrees(steer):+.1f}°  '
            f'速度={speed:.1f}m/s  '
            f'Status={"FIX" if self.current_status == 1 else "FLOAT"}'
        )

    # -----------------------------------------------------------
    # 安全停止チェック（タイマーで0.5秒ごとに呼ばれる）
    # -----------------------------------------------------------
    def _safety_check(self):
        elapsed = (self.get_clock().now() - self.last_gnss_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.get_logger().warn(f'⚠️  GNSSデータが{elapsed:.1f}秒途絶えています → 安全停止')
            self._publish_stop()

    # -----------------------------------------------------------
    # コマンド送信
    # -----------------------------------------------------------
    def _publish_command(self, speed: float, steering_angle: float):
        msg = AckermannDriveStamped()
        msg.drive.speed          = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish_command(0.0, 0.0)


# -----------------------------------------------------------
# エントリーポイント
# -----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PidControllerV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
