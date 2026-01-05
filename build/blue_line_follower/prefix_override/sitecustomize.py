import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wayay/Ros/ROS2_Project/install/blue_line_follower'
