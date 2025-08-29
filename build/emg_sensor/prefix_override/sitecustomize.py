import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ganeshkoka/Github/EMG-Project/install/emg_sensor'
