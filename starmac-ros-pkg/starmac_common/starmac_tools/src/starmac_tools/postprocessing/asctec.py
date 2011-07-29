import numpy as np
from generic import get_header_time

# From asctec_adapter.cpp:
ROLL_SCALE = 40.0 # counts/deg
PITCH_SCALE = 40.0 # counts/deg
YAW_SCALE = 2047 / 254.760 # counts/deg/s

def process_asctec_ctrl_input(bag, topic, output_var):
    """
    """
    output_var.t = get_header_time(bag, topic)
    output_var.roll_cmd, output_var.pitch_cmd, output_var.yaw_rate_cmd  = \
        np.array(tuple(bag._data[topic + '/%s' % ax]/scale 
                       for ax, scale in (('roll', ROLL_SCALE), 
                                         ('pitch', PITCH_SCALE), 
                                         ('yaw', YAW_SCALE))
                       ))