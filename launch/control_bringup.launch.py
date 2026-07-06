"""
control_bringup.launch.py

走行制御に必要なノードを起動する（GNSS受信機側はgnss_bringup.launch.pyで別途
立てっぱなしにしておく想定。GNSS側を毎回再起動せずに、このlaunchファイルだけを
何度も再起動してパラメータを変えたり再走行したりできる）。

  cc02_autodrive: pure_pursuit_node, gnss_logger_node
  rc_car_driver : vehicle_driver

走行の開始トリガーは無く、起動後最初にRTK FIXを一度取得した時点で自動的に走行を開始する
（FLOATだけでは動かない。一度FIXを取得した後はFLOATへの低下を許容して走行を継続する）。
GNSS側（gnss_bringup.launch.py）が既にFIXに達していれば、このlaunchファイルを
起動した直後の最初の/gnss/solutionでFIXが判定されるので、収束待ちは発生しない。

waypointは緯度経度（WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）の
CSVを使用する（デフォルト: wp_position_basic.csv）。

gnss_logger_nodeは/gnss/solutionを毎回CSVに記録する（デフォルト: ~/ros2_ws/gnss_logs/
gnss_log_latest.csv。固定ファイル名で、毎回起動時に上書きされる。過去のログを残したい
場合はlog_file引数で別名を指定すること）。走行後にmatlab/plot_log_map.mで地図に
表示できる（MATLAB環境で、ログCSVを転送してから実行: 詳細はCLAUDE.md参照）。

使用例:
  ros2 launch cc02_autodrive control_bringup.launch.py
  ros2 launch cc02_autodrive control_bringup.launch.py wp_file:=/path/to/wp.csv
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rc_car_share = get_package_share_directory('rc_car_driver')
    cc02_share = get_package_share_directory('cc02_autodrive')

    vehicle_driver_config = os.path.join(rc_car_share, 'config', 'vehicle_driver.yaml')
    default_wp_file = os.path.join(cc02_share, 'wp_position_basic.csv')

    log_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'gnss_logs')
    os.makedirs(log_dir, exist_ok=True)

    default_log_file = os.path.join(log_dir, 'gnss_log_latest.csv')
    default_pp_log   = os.path.join(log_dir, 'pure_pursuit_log_latest.csv')

    wp_file_arg = DeclareLaunchArgument(
        'wp_file',
        default_value=default_wp_file,
        description='Waypoint CSV（WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）の絶対パス'
    )
    log_file_arg = DeclareLaunchArgument(
        'log_file',
        default_value=default_log_file,
        description='/gnss/solutionの記録先CSVファイルの絶対パス'
    )
    tuning_log_file_arg = DeclareLaunchArgument(
        'tuning_log_file',
        default_value=default_pp_log,
        description='Pure Pursuit チューニングログの絶対パス'
    )
    corner_wp_indices_arg = DeclareLaunchArgument(
        'corner_wp_indices',
        default_value='2,6,8',
        description='減速するWPインデックスをカンマ区切りで指定（0-based、例: 2,6,8）'
    )
    wp_radii_arg = DeclareLaunchArgument(
        'wp_radii',
        default_value='',
        description='WPごとの到達半径（0-basedインデックス:半径のカンマ区切り、例: 3:1.5,7:2.0）空=全WP共通wp_radiusを使用'
    )

    pure_pursuit_node = Node(
        package='cc02_autodrive',
        executable='pure_pursuit_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'wp_file': LaunchConfiguration('wp_file'),
            'corner_wp_indices': LaunchConfiguration('corner_wp_indices'),
            'tuning_log_file': LaunchConfiguration('tuning_log_file'),
            'wp_radii': LaunchConfiguration('wp_radii'),
        }],
    )

    vehicle_driver_node = Node(
        package='rc_car_driver',
        executable='vehicle_driver',
        name='vehicle_driver',
        output='screen',
        parameters=[vehicle_driver_config],
    )

    gnss_logger_node = Node(
        package='cc02_autodrive',
        executable='gnss_logger_node',
        name='gnss_logger',
        output='screen',
        parameters=[{'log_file': LaunchConfiguration('log_file')}],
    )

    return LaunchDescription([
        wp_file_arg,
        log_file_arg,
        tuning_log_file_arg,
        corner_wp_indices_arg,
        wp_radii_arg,
        pure_pursuit_node,
        vehicle_driver_node,
        gnss_logger_node,
    ])
