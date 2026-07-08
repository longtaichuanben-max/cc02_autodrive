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

course引数でコースを選択する（デフォルト: basic）:
  ros2 launch cc02_autodrive control_bringup.launch.py course:=basic
  ros2 launch cc02_autodrive control_bringup.launch.py course:=advance

courseに応じてwaypointファイルとログファイル名が自動で切り替わる:
  basic   → wp_position_basic.csv   / gnss_log_basic_latest.csv
  advance → wp_position_advance.csv / gnss_log_advance_latest.csv

gnss_logger_nodeは/gnss/solutionをCSVに記録する。走行後にmatlab/plot_log_map.mで
地図に表示できる（MATLAB環境で、ログCSVを転送してから実行: 詳細はCLAUDE.md参照）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rc_car_share = get_package_share_directory('rc_car_driver')
    cc02_share = get_package_share_directory('cc02_autodrive')

    vehicle_driver_config = os.path.join(rc_car_share, 'config', 'vehicle_driver.yaml')

    log_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'gnss_logs')
    os.makedirs(log_dir, exist_ok=True)

    course_arg = DeclareLaunchArgument(
        'course',
        default_value='basic',
        description="コース選択: 'basic' / 'advance'"
    )
    wp_file_arg = DeclareLaunchArgument(
        'wp_file',
        default_value=PythonExpression([
            f'"{cc02_share}/wp_position_" + "',
            LaunchConfiguration('course'),
            '" + ".csv"'
        ]),
        description='Waypoint CSV の絶対パス（省略時は course から自動決定）'
    )
    log_file_arg = DeclareLaunchArgument(
        'log_file',
        default_value=PythonExpression([
            f'"{log_dir}/gnss_log_" + "',
            LaunchConfiguration('course'),
            '" + "_latest.csv"'
        ]),
        description='/gnss/solution の記録先CSVファイルの絶対パス（省略時は course から自動決定）'
    )
    tuning_log_file_arg = DeclareLaunchArgument(
        'tuning_log_file',
        default_value=PythonExpression([
            f'"{log_dir}/pure_pursuit_log_" + "',
            LaunchConfiguration('course'),
            '" + "_latest.csv"'
        ]),
        description='Pure Pursuit チューニングログの絶対パス（省略時は course から自動決定）'
    )
    pure_pursuit_node = Node(
        package='cc02_autodrive',
        executable='pure_pursuit_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'wp_file': LaunchConfiguration('wp_file'),
            'tuning_log_file': LaunchConfiguration('tuning_log_file'),
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
        course_arg,
        wp_file_arg,
        log_file_arg,
        tuning_log_file_arg,
        pure_pursuit_node,
        vehicle_driver_node,
        gnss_logger_node,
    ])
