"""
autodrive_bringup.launch.py

GNSS-RTK waypoint走行に必要な6ノードを一括起動する。

  gnss_ros_standardization: ubx_driver_node, rtcm_decoder_node, real_time_kinematic
  cc02_autodrive          : pid_node（pid_controller.py）, mouse_trigger_node
  rc_car_driver           : vehicle_driver

waypointは緯度経度（WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）の
CSVを使用する（デフォルト: wp_position.csv）。

NTRIPの接続情報（ユーザー名/パスワードを含む）はGit管理下に置きたくないため、
launch引数で渡す（コミットされるyamlファイルには書き込まない）。

使用例:
  ros2 launch cc02_autodrive autodrive_bringup.launch.py \
    ntrip_stream_path:="ntrip://user:pass@caster.example.com:2101/MOUNT" \
    gnss_serial_path:="serial:///dev/ttyACM0:115200" \
    wp_file:="/home/ryutakawamoto/ros2_ws/src/cc02_autodrive/cc02_autodrive/wp_position.csv"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gnss_share = get_package_share_directory('gnss_ros_standardization')
    rc_car_share = get_package_share_directory('rc_car_driver')
    cc02_share = get_package_share_directory('cc02_autodrive')

    ubx_driver_config = os.path.join(gnss_share, 'config', 'ubx_driver.yaml')
    rtk_config = os.path.join(gnss_share, 'config', 'real_time_kinematic.yaml')
    vehicle_driver_config = os.path.join(rc_car_share, 'config', 'vehicle_driver.yaml')
    default_wp_file = os.path.join(cc02_share, 'wp_position.csv')

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

    pid_node = Node(
        package='cc02_autodrive',
        executable='pid_node',
        name='pid_controller',
        output='screen',
        parameters=[{'wp_file': LaunchConfiguration('wp_file')}],
    )

    mouse_trigger_node = Node(
        package='cc02_autodrive',
        executable='mouse_trigger_node',
        name='mouse_trigger_node',
        output='screen',
    )

    vehicle_driver_node = Node(
        package='rc_car_driver',
        executable='vehicle_driver',
        name='vehicle_driver',
        output='screen',
        parameters=[vehicle_driver_config],
    )

    return LaunchDescription([
        ntrip_stream_path_arg,
        gnss_serial_path_arg,
        wp_file_arg,
        ubx_driver_node,
        rtcm_decoder_node,
        real_time_kinematic_node,
        pid_node,
        mouse_trigger_node,
        vehicle_driver_node,
    ])
