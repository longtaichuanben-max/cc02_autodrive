"""
autodrive_bringup.launch.py

GNSS-RTK waypoint走行に必要な6ノードを一括起動する。

  gnss_ros_standardization: ubx_driver_node, rtcm_decoder_node, real_time_kinematic
  cc02_autodrive          : pid_node / stanley_node / pure_pursuit_node（controller引数で選択）, gnss_logger_node
  rc_car_driver           : vehicle_driver

走行の開始トリガーは無く、起動後最初にRTK FIXを一度取得した時点で自動的に走行を開始する
（FLOATだけでは動かない。一度FIXを取得した後はFLOATへの低下を許容して走行を継続する）。

controller引数で走行制御アルゴリズムを選べる（デフォルト: pid）:
  ros2 launch cc02_autodrive autodrive_bringup.launch.py controller:=stanley ...
  ros2 launch cc02_autodrive autodrive_bringup.launch.py controller:=pure_pursuit ...

waypointは緯度経度（WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）の
CSVを使用する（デフォルト: wp_position.csv）。

gnss_logger_nodeは/gnss/solutionを毎回CSVに記録する（デフォルト: ~/ros2_ws/gnss_logs/
gnss_log_<起動時刻>.csv）。走行後にcc02_autodrive plot_log_mapで地図HTMLに変換できる：
  ros2 run cc02_autodrive plot_log_map ~/ros2_ws/gnss_logs/gnss_log_xxxx.csv \
    --wp-file <wp_fileと同じパス>

NTRIPの接続情報（ユーザー名/パスワードを含む）はGit管理下に置きたくないため、
launch引数で渡す（コミットされるyamlファイルには書き込まない）。

使用例（PID、デフォルト）:
  ros2 launch cc02_autodrive autodrive_bringup.launch.py \
    ntrip_stream_path:="ntrip://user:pass@caster.example.com:2101/MOUNT" \
    gnss_serial_path:="serial:///dev/ttyACM0:115200"

使用例（Stanley）:
  ros2 launch cc02_autodrive autodrive_bringup.launch.py \
    controller:=stanley \
    ntrip_stream_path:="ntrip://user:pass@caster.example.com:2101/MOUNT" \
    gnss_serial_path:="serial:///dev/ttyACM0:115200"
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    gnss_share = get_package_share_directory('gnss_ros_standardization')
    rc_car_share = get_package_share_directory('rc_car_driver')
    cc02_share = get_package_share_directory('cc02_autodrive')

    ubx_driver_config = os.path.join(gnss_share, 'config', 'ubx_driver.yaml')
    rtk_config = os.path.join(gnss_share, 'config', 'real_time_kinematic.yaml')
    vehicle_driver_config = os.path.join(rc_car_share, 'config', 'vehicle_driver.yaml')
    default_wp_file = os.path.join(cc02_share, 'wp_position.csv')

    log_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'gnss_logs')
    default_log_file = os.path.join(
        log_dir, f'gnss_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
    )

    ntrip_stream_path_arg = DeclareLaunchArgument(
        'ntrip_stream_path',
        default_value='',
        description='RTCM補正データ用NTRIPキャスターのURI（例: ntrip://user:pass@host:port/MOUNT）'
    )
    gnss_serial_path_arg = DeclareLaunchArgument(
        'gnss_serial_path',
        default_value='serial:///dev/ttyACM0:115200',
        description='u-blox受信機のシリアルポート'
    )
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

    ubx_driver_node = Node(
        package='gnss_ros_standardization',
        executable='ubx_driver_node',
        name='ubx_driver_node',
        output='screen',
        parameters=[
            ubx_driver_config,
            {'stream_path': LaunchConfiguration('gnss_serial_path')},
        ],
    )

    rtcm_decoder_node = Node(
        package='gnss_ros_standardization',
        executable='rtcm_decoder_node',
        name='rtcm_decoder_node',
        output='screen',
        parameters=[{
            'stream_path': LaunchConfiguration('ntrip_stream_path'),
            'observation_topic': '/base/gnss/observation',
            'ephemeris_topic': '/gnss/ephemeris',
        }],
    )

    real_time_kinematic_node = Node(
        package='gnss_ros_standardization',
        executable='real_time_kinematic',
        name='real_time_kinematic',
        output='screen',
        parameters=[rtk_config],
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
        ntrip_stream_path_arg,
        gnss_serial_path_arg,
        wp_file_arg,
        log_file_arg,
        controller_arg,
        ubx_driver_node,
        rtcm_decoder_node,
        real_time_kinematic_node,
        pid_node,
        stanley_node,
        pure_pursuit_node,
        vehicle_driver_node,
        gnss_logger_node,
    ])
