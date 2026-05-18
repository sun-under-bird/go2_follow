import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/go2_follow/install/go2_dynamic_follow_avoidance'
