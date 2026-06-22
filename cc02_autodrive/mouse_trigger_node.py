"""
mouse_trigger_node.py

Bluetooth接続したマウスの左クリックを検出し、走行の開始/停止をトグルするノード。
左クリック1回ごとに True/False を反転させ、/mouse_start_stop (std_msgs/Bool) にpublishする。
pid_controller_v2.py はこのトピックを購読し、走行可否を判定する。

事前準備（Raspberry Pi側）:
  sudo apt install python3-evdev
  sudo usermod -aG input $USER   # /dev/input/eventX への読み取り権限。再ログインが必要
"""

import time
import threading

import evdev

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MouseTriggerNode(Node):

    def __init__(self):
        super().__init__('mouse_trigger_node')

        # 空文字（デフォルト）の場合は、BTN_LEFTを持つデバイスを自動検出する
        self.declare_parameter('device_path', '')
        self.declare_parameter('start_stop_topic', '/mouse_start_stop')

        topic = self.get_parameter('start_stop_topic').value
        self.pub = self.create_publisher(Bool, topic, 10)

        self.is_running = False

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'Mouse Trigger Node 起動完了（左クリックで開始/停止をトグル, topic={topic}）'
        )

    # -----------------------------------------------------------
    # マウスデバイスの検出・オープン
    # -----------------------------------------------------------
    def _find_mouse_device(self):
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except OSError:
                continue
            caps = dev.capabilities().get(evdev.ecodes.EV_KEY, [])
            if evdev.ecodes.BTN_LEFT in caps:
                return dev
            dev.close()
        return None

    def _open_device(self):
        device_path = self.get_parameter('device_path').value
        if device_path:
            try:
                return evdev.InputDevice(device_path)
            except OSError as e:
                self.get_logger().error(f'指定デバイスを開けません: {device_path} ({e})')
                return None
        return self._find_mouse_device()

    # -----------------------------------------------------------
    # イベント読み取りループ（別スレッドで実行）
    # Bluetooth切断時は安全停止を発行し、再接続を待って自動的に復帰する
    # -----------------------------------------------------------
    def _read_loop(self):
        while rclpy.ok():
            device = self._wait_for_device()
            if device is None:
                return  # rclpy.ok()がFalseになった（ノード終了）

            self.get_logger().info(f'マウスデバイスを検出: {device.name} ({device.path})')

            try:
                for event in device.read_loop():
                    if not rclpy.ok():
                        return
                    if (event.type == evdev.ecodes.EV_KEY
                            and event.code == evdev.ecodes.BTN_LEFT
                            and event.value == 1):  # 押下時のみ（離した時は反応しない）
                        self._toggle()
            except OSError as e:
                self.get_logger().error(f'マウスデバイスとの接続が切れました: {e}')
                self._force_stop()
                self.get_logger().warn('再接続を待機します...')

    def _wait_for_device(self):
        device = None
        while rclpy.ok() and device is None:
            device = self._open_device()
            if device is None:
                self.get_logger().warn(
                    'マウスデバイスが見つかりません。2秒後に再試行します...'
                )
                time.sleep(2.0)
        return device

    # -----------------------------------------------------------
    # 開始/停止トグル
    # -----------------------------------------------------------
    def _toggle(self):
        self.is_running = not self.is_running
        msg = Bool()
        msg.data = self.is_running
        self.pub.publish(msg)
        state = '開始 ▶️' if self.is_running else '停止 ⏸'
        self.get_logger().info(f'🖱️ 左クリック検出 → {state}')

    # -----------------------------------------------------------
    # マウス切断時の安全停止（走行中だった場合のみ停止を発行）
    # -----------------------------------------------------------
    def _force_stop(self):
        if self.is_running:
            self.is_running = False
            msg = Bool()
            msg.data = False
            self.pub.publish(msg)
            self.get_logger().info('⏸ マウス切断のため走行停止')


def main(args=None):
    rclpy.init(args=args)
    node = MouseTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
