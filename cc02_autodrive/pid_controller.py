import rclpy #ROS2の機能をPythonで使うための超巨大な道具箱(raspberry piの中でデータの送受信が可能に！)
from rclpy.node import Node#道具箱（rclpy）の中から、特に重要な Node（ノード＝プログラムの本体になる部品） という道具をピンポイントで取り出している

class PidController(Node):#[PID制御のノード]という新しいクラスを定義している。Nodeクラスを継承しているので、Nodeの機能も使えるようになる。親
    def __init__(self):#クラスの初期化関数。クラスが呼び出されたときに最初に実行される特別な関数。(1回だけ実行される)
        super().__init__('pid_controller')#Nodeクラスの初期化関数を呼び出している。引数の 'pid_controller' は、このノードの名前になる。子供
        self.get_logger().info('PID Controller Node has been started!')#このノードが起動したときに、ログに「PID Controller Node has been started!」というメッセージを表示する。self.get_logger()は、このノード専用のロガー（ログを記録する道具）を取得するための関数。info()は、そのロガーを使って情報レベルのログメッセージを出力するための関数。
        
        # TODO: ここに /gnss/solution を受け取る設定（Subscriber）を書く
        # TODO: ここに /ackermann_cmd を送る設定（Publisher）を書く
        # TODO: ここにPIDの計算ロジックを書く

def main(args=None):
    rclpy.init(args=args)#ros2の通信エンジンを始動。
    node = PidController()#ラズパイのメモリ上でノードを本当に起動する。
    rclpy.spin(node)#この関数は、ROS2のイベントループを開始するための関数。引数にノードを渡すと、そのノードがROS2のイベントループに参加することになる。これによって、ノードはROS2の通信機能を使ってデータの送受信ができるようになる。
    node.destroy_node()#メモリや通信ポートを綺麗にお掃除して、安全にプログラムを終了させるためのお約束のコード。
    rclpy.shutdown()#ROS2の通信エンジンを安全にシャットダウンするためのお約束のコード。

if __name__ == '__main__':
    main()