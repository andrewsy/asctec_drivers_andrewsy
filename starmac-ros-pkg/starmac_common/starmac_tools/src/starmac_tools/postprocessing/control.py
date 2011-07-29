import numpy as np
from generic import get_header_time

def process_controller_status(bag, topic, output_var):
    output_var.t = get_header_time(bag, topic)
    output_var.active_mode = bag._data[topic+'/active_mode']
    output_var.autoseq_idxs = np.where(np.array([{'autosequence':True}.get(x,False) for x in output_var.active_mode]))[0]
    
def process_control_mode_output(bag, topic, output_var):
    output_var.t = get_header_time(bag, topic)
    output_var.control_mode = bag._data[topic+'/control_mode']
    (output_var.alt_cmd, 
     output_var.pitch_cmd, 
     output_var.roll_cmd, 
     output_var.yaw_cmd, 
     output_var.yaw_rate_cmd) = \
        (bag._data[topic+'/%s_cmd' % ax] for ax in ('alt', 'pitch', 'roll', 'yaw', 'yaw_rate'))
        
