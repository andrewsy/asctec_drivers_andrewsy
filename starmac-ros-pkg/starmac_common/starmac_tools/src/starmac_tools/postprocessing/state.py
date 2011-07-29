import numpy as np
from generic import get_header_time
from starmac_tools.timeseries import quaternion_to_eulerypr

def process_state(bag, topic, output_var):
    """
    Process the state data in a given (nav_msgs/Odom) topic and place results in the
    north, east, up, {north,east,up}_vel, ori_quat and ori_ypr (yaw/pitch/roll) attributes
    of the provided output_var argument
    """
    output_var.t = get_header_time(bag, topic)
    output_var.north, output_var.east, output_var.up = \
        (mult*bag._data[topic+'/pose/pose/position/%s' % ax] for (ax, mult) in zip('xyz',(1.,1.,-1.)))
    output_var.north_vel, output_var.east_vel, output_var.up_vel = (bag._data[topic+'/twist/twist/linear/%s' % ax] for ax in 'xyz')
    output_var.ori_quat = np.array(tuple(bag._data[topic+'/pose/pose/orientation/%s' % ax] for ax in 'wxyz')).T
    output_var.ori_ypr = np.degrees(np.array(quaternion_to_eulerypr(output_var.ori_quat.T)).T)
    
