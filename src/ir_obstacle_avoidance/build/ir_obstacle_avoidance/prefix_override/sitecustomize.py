import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/krutharthmc/Downloads/ir_obstacle_avoidance/install/ir_obstacle_avoidance'
