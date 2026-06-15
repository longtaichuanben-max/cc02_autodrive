import rclpy #ROS2の機能をPythonで使うための超巨大な道具箱(raspberry piの中でデータの送受信が可能に！)
from rclpy.node import Node#道具箱（rclpy）の中から、特に重要な Node（ノード＝プログラムの本体になる部品） という道具をピンポイントで取り出している
from ackermann_msgs.msg import AckermannDriveStamped  # 追加：自動運転の標準的な「手足」の命令メッセージ
from gnss_ros_standardization.msg import GnssSolution  #GNSSデータのメッセージ

class PidController(Node):#[PID制御のノード]という新しいクラスを定義している。Nodeクラスを継承しているので、Nodeの機能も使えるようになる。親
    def __init__(self):#クラスの初期化関数。クラスが呼び出されたときに最初に実行される特別な関数。(1回だけ実行される)
        super().__init__('pid_controller')#Nodeクラスの初期化関数を呼び出している。引数の 'pid_controller' は、このノードの名前になる。子供
        self.get_logger().info('PID Controller Node has been started!')#このノードが起動したときに、ログに「PID Controller Node has been started!」というメッセージを表示する。self.get_logger()は、このノード専用のロガー（ログを記録する道具）を取得するための関数。info()は、そのロガーを使って情報レベルのログメッセージを出力するための関数。
        
# --- 【手足】 モーターへの命令発信機 ---
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.timer = self.create_timer(0.1, self.publish_test_command)
        
        # --- 【目】 GNSSデータの受信機 ---
        # /gnss/nmea_solution というコンセントから、GnssSolution 型のデータを受け取る設定
        self.gnss_sub = self.create_subscription(
            GnssSolution,
            '/gnss/solution',  # データが届くコンセントの名前
            self.gnss_callback,  # データが届くたびにこの関数を呼び出す
            10
        )
        
        # 現在地を記憶しておくための変数
        self.current_x = 0.0
        self.current_y = 0.0

    def gnss_callback(self, msg):
        # 🌟 データが届いた瞬間に実行される関数
        self.current_x = msg.pos_enu.x
        self.current_y = msg.pos_enu.y
        status = msg.status  # 1ならRTK Fix（最高精度）
        
        # 届いた現在地を画面に表示する
        self.get_logger().info(f'📍 現在地を受信！ X:{self.current_x:.2f}m, Y:{self.current_y:.2f}m, Status:{status}')

    def publish_test_command(self):
        msg = AckermannDriveStamped()
        # いったん安全のためモーターは止めておく（ステアリングはまっすぐ）
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)#ros2の通信エンジンを始動。
    node = PidController()#ラズパイのメモリ上でノードを本当に起動する。
    rclpy.spin(node)#この関数は、ROS2のイベントループを開始するための関数。引数にノードを渡すと、そのノードがROS2のイベントループに参加することになる。これによって、ノードはROS2の通信機能を使ってデータの送受信ができるようになる。
    node.destroy_node()#メモリや通信ポートを綺麗にお掃除して、安全にプログラムを終了させるためのお約束のコード。
    rclpy.shutdown()#ROS2の通信エンジンを安全にシャットダウンするためのお約束のコード。

if __name__ == '__main__':
    main()