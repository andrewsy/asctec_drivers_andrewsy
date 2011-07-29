import numpy as np
from generic import get_header_time
from starmac_tools.timeseries import quaternion_to_eulerypr

def process_imu(bag, topic, output_var):
    """
    """
    output_var.t = get_header_time(bag, topic)
    output_var.ori_quat = np.array(tuple(bag._data[topic+'/orientation/%s' % ax] for ax in 'wxyz')).T
    output_var.ori_ypr = np.degrees(np.array(quaternion_to_eulerypr(output_var.ori_quat.T)).T)
