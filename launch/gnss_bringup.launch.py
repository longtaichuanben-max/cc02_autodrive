"""
gnss_bringup.launch.py

GNSS受信機側の2プロセスだけを起動する（制御ノードとは独立。立てっぱなしにして
RTK FIXを維持しておき、別ターミナルでcontrol_bringup.launch.pyを何度も
再起動してもFIX収束を待たずに済むようにする運用を想定）。

  gnss_ros_standardization: ubx_driver_node
  str2str (RTKLIB)        : NTRIPキャスターのRTCM補正を受信機のUART1へ送信

RTK計算はu-blox受信機内部で行う（詳細はcc02_autodrive/CLAUDE.md参照）。str2strは
ROS2ノードではない普通のプロセスだが、ExecuteProcessでこのlaunchファイルに統合し、
Ctrl-Cで両方まとめて終了できるようにしている。

2026-06-25にubx_driver_nodeのMON-VER pingが毎回タイムアウトする事象があったが、
str2strとの競合ではなく、USB再接続を繰り返したことで受信機自体がUSB出力を止めて
応答しなくなっていたことが原因と判明（物理的な抜き差しで復旧）。startup_delay_s
（既定5秒、str2strの起動をubx_driver_nodeの初期化後にずらす）は無害なので残してい
るが、根本対策ではない。/dev/ttyACM<N>の番号は再接続のたびにズレる可能性があるため、
gnss_serial_pathの既定値はudevのby-id安定パスを使っている。

NTRIPの接続情報（ユーザー名/パスワードを含む）はGit管理下に置きたくないため、
環境変数 NTRIP_STREAM_PATH から読み込む。例えば ~/.bashrc に以下を追記しておく:
  export NTRIP_STREAM_PATH="ntrip://user:pass@caster.example.com:2101/MOUNT"
（追記後は新しいターミナルを開くか `source ~/.bashrc` すること）

使用例:
  ros2 launch cc02_autodrive gnss_bringup.launch.py
  ros2 launch cc02_autodrive gnss_bringup.launch.py \
    gnss_serial_path:="serial:///dev/ttyACM0:115200" \
    correction_serial_path:="serial://ttyAMA0:115200"
  # ttyACM<N>の番号が分かっている場合は上のように明示的に指定してもよい

環境変数を使わず、その場でNTRIP URIを直接渡すこともできる:
  ros2 launch cc02_autodrive gnss_bringup.launch.py \
    ntrip_stream_path:="ntrip://user:pass@caster.example.com:2101/MOUNT"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gnss_share = get_package_share_directory('gnss_ros_standardization')
    ubx_driver_config = os.path.join(gnss_share, 'config', 'ubx_driver.yaml')

    gnss_serial_path_arg = DeclareLaunchArgument(
        'gnss_serial_path',
        default_value=(
            'serial:///dev/serial/by-id/'
            'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:115200'
        ),
        description=(
            'u-blox受信機のシリアルポート（USB、測位解の読み取り用）。'
            '/dev/ttyACM<N>は再接続のたびに番号がズレる可能性があるため、'
            'udev自動生成のby-id安定パスを既定値にしている'
        )
    )
    ntrip_stream_path_arg = DeclareLaunchArgument(
        'ntrip_stream_path',
        default_value=os.environ.get('NTRIP_STREAM_PATH', ''),
        description=(
            'RTCM補正データ用NTRIPキャスターのURI '
            '（既定値は環境変数NTRIP_STREAM_PATHから取得。未設定なら明示的に指定すること）'
        )
    )
    correction_serial_path_arg = DeclareLaunchArgument(
        'correction_serial_path',
        default_value='serial://ttyAMA0:115200',
        description='str2strの出力先（受信機のUART1ポート）'
    )
    startup_delay_arg = DeclareLaunchArgument(
        'startup_delay_s',
        default_value='5.0',
        description='ubx_driver_nodeの受信機初期化が終わるまでstr2strの起動を遅らせる秒数'
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

    str2str_process = ExecuteProcess(
        cmd=[
            'str2str',
            '-in', LaunchConfiguration('ntrip_stream_path'),
            '-out', LaunchConfiguration('correction_serial_path'),
        ],
        name='str2str',
        output='screen',
    )

    # ubx_driver_nodeのUSB初期化（ping/CFG-GNSS設定等）とstr2strのUART1書き込み開始が
    # 競合してpingタイムアウトを起こすため、str2strの起動だけ遅らせる
    delayed_str2str = TimerAction(
        period=LaunchConfiguration('startup_delay_s'),
        actions=[str2str_process],
    )

    return LaunchDescription([
        gnss_serial_path_arg,
        ntrip_stream_path_arg,
        correction_serial_path_arg,
        startup_delay_arg,
        ubx_driver_node,
        delayed_str2str,
    ])
