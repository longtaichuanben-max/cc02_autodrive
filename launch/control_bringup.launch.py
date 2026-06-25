"""
control_bringup.launch.py

走行制御に必要な3ノードを起動する（GNSS受信機側はgnss_bringup.launch.pyで別途
立てっぱなしにしておく想定。GNSS側を毎回再起動せずに、このlaunchファイルだけを
何度も再起動してコントローラーを切り替えたり再走行したりできる）。

  cc02_autodrive: pid_node / stanley_node / pure_pursuit_node（controller引数で選択）,
                  gnss_logger_node
  rc_car_driver : vehicle_driver

走行の開始トリガーは無く、起動後最初にRTK FIXを一度取得した時点で自動的に走行を開始する
（FLOATだけでは動かない。一度FIXを取得した後はFLOATへの低下を許容して走行を継続する）。
GNSS側（gnss_bringup.launch.py）が既にFIXに達していれば、このlaunchファイルを
起動した直後の最初の/gnss/solutionでFIXが判定されるので、収束待ちは発生しない。

controller引数で走行制御アルゴリズムを選べる（デフォルト: pid）:
  ros2 launch cc02_autodrive control_bringup.launch.py controller:=stanley
  ros2 launch cc02_autodrive control_bringup.launch.py controller:=pure_pursuit

waypointは緯度経度（WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）の
CSVを使用する（デフォルト: wp_position_basic.csv）。

gnss_logger_nodeは/gnss/solutionを毎回CSVに記録する（デフォルト: ~/ros2_ws/gnss_logs/
gnss_log_latest.csv。固定ファイル名で、毎回起動時に上書きされる。過去のログを残したい
場合はlog_file引数で別名を指定すること）。走行後にmatlab/plot_log_map.mで地図に
表示できる（MATLAB環境で、ログCSVを転送してから実行: 詳細はCLAUDE.md参照）。

使用例（PID、デフォルト）:
  ros2 launch cc02_autodrive control_bringup.launch.py

使用例（Stanley）:
  ros2 launch cc02_autodrive control_bringup.launch.py controller:=stanley
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rc_car_share = get_package_share_directory('rc_car_driver')
    cc02_share = get_package_share_directory('cc02_autodrive')

    vehicle_driver_config = os.path.join(rc_car_share, 'config', 'vehicle_driver.yaml')
    default_wp_file = os.path.join(cc02_share, 'wp_position_basic.csv')

    log_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'gnss_logs')
    default_log_file = os.path.join(log_dir, 'gnss_log_latest.csv')

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
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid',
        description="走行制御アルゴリズム: 'pid' / 'stanley' / 'pure_pursuit'"
    )

    is_pid = PythonExpression(["'", LaunchConfiguration('controller'), "' == 'pid'"])
    is_stanley = PythonExpression(["'", LaunchConfiguration('controller'), "' == 'stanley'"])
    is_pure_pursuit = PythonExpression(["'", LaunchConfiguration('controller'), "' == 'pure_pursuit'"])

    pid_node = Node(
        package='cc02_autodrive',
        executable='pid_node',
        name='pid_controller',
        output='screen',
        parameters=[{'wp_file': LaunchConfiguration('wp_file')}],
        condition=IfCondition(is_pid),
    )

    stanley_node = Node(
        package='cc02_autodrive',
        executable='stanley_node',
        name='stanley_controller',
        output='screen',
        parameters=[{'wp_file': LaunchConfiguration('wp_file')}],
        condition=IfCondition(is_stanley),
    )

    pure_pursuit_node = Node(
        package='cc02_autodrive',
        executable='pure_pursuit_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{'wp_file': LaunchConfiguration('wp_file')}],
        condition=IfCondition(is_pure_pursuit),
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
        controller_arg,
        pid_node,
        stanley_node,
        pure_pursuit_node,
        vehicle_driver_node,
        gnss_logger_node,
    ])
