import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ryutakawamoto/ros2_ws/src/cc02_autodrive/install/cc02_autodrive'
