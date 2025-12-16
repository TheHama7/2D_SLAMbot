import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hama/ros2_ws/thesisFolder/Thesis/ros2/install/esp32_bridge'
