import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from gnss_ros_standardization.msg import GnssSolution

_STATUS_NAMES = {
    GnssSolution.STATUS_NONE:   'NONE',
    GnssSolution.STATUS_FIX:    'FIX',
    GnssSolution.STATUS_FLOAT:  'FLOAT',
    GnssSolution.STATUS_SBAS:   'SBAS',
    GnssSolution.STATUS_DGPS:   'DGPS',
    GnssSolution.STATUS_SINGLE: 'SINGLE',
    GnssSolution.STATUS_PPP:    'PPP',
    GnssSolution.STATUS_EKF:    'EKF',
}

_CSV_HEADER = [
    'wall_time', 'ros_stamp_sec', 'gps_week', 'gps_tow',
    'status', 'status_str', 'num_sats', 'ratio', 'hdop',
    'latitude', 'longitude', 'altitude',
    'enu_x', 'enu_y', 'speed_mps',
]


class GnssLogger(Node):
    def __init__(self):
        super().__init__('gnss_logger')

        default_log_name = f'gnss_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.declare_parameter('log_file', default_log_name)
        log_file = self.get_parameter('log_file').value

        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        self._csv_file = open(log_file, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(_CSV_HEADER)
        self._csv_file.flush()
        self._n_logged = 0

        self.gnss_sub = self.create_subscription(
            GnssSolution, '/gnss/solution', self._gnss_callback, 10
        )

        self.get_logger().info(f'gnss_logger 起動完了 → 記録先: {os.path.abspath(log_file)}')

    def _gnss_callback(self, msg: GnssSolution):
        ros_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        speed = (msg.vel_enu.x ** 2 + msg.vel_enu.y ** 2) ** 0.5

        self._writer.writerow([
            f'{time.time():.3f}',
            f'{ros_stamp_sec:.3f}',
            msg.time_week,
            f'{msg.time_tow:.3f}',
            msg.status,
            _STATUS_NAMES.get(msg.status, f'UNKNOWN({msg.status})'),
            msg.num_sats,
            f'{msg.ratio:.2f}',
            f'{msg.hdop:.2f}',
            f'{msg.latitude:.8f}',
            f'{msg.longitude:.8f}',
            f'{msg.altitude:.3f}',
            f'{msg.pos_enu.x:.3f}',
            f'{msg.pos_enu.y:.3f}',
            f'{speed:.3f}',
        ])
        self._csv_file.flush()
        self._n_logged += 1

    def destroy_node(self):
        self.get_logger().info(f'gnss_logger 終了 → 合計{self._n_logged}行を記録しました')
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GnssLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
