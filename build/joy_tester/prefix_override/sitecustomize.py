import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tusharchillal/PhysicalBot_ws/install/joy_tester'
