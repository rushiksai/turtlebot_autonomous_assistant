import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rushiksaiii/turtlebot_autonomous_assistant/assistant_ws/install/door_control_pkg'
