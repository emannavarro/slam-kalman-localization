import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/emannavarro/slam-kalman-localization/install/webots_ros2_tiago'
