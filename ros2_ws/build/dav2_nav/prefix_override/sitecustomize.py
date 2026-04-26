import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mbot/DAV2-MBot-Nav/ros2_ws/install/dav2_nav'
