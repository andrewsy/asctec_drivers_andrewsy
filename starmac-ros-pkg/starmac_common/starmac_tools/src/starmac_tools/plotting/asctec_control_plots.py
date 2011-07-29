#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, UC Regents
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of California nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('starmac_tools')

import sys
import scipy.io as sio
import numpy as np
from pylab import plt

from starmac_tools.load_bag import BagLoader
from starmac_tools.postprocessing.generic import Dummy
from starmac_tools.postprocessing.state import process_state
from starmac_tools.postprocessing.control import process_control_mode_output
from starmac_tools.postprocessing.imu import process_imu
from starmac_tools.postprocessing.asctec import process_asctec_ctrl_input
from starmac_tools.plotting.util import newfig, show
from starmac_tools.timeseries import uniform_resample

in_bag_fname = sys.argv[1]

input_bag = BagLoader(in_bag_fname)

v = Dummy()
v.state = Dummy()
v.control = Dummy()
v.imu = Dummy()
v.asctec_ctrl_input = Dummy()

process_state(input_bag, 'odom', v.state)
process_control_mode_output(input_bag, 'controller_mux/output', v.control)
process_imu(input_bag, 'asctec/imu', v.imu)
process_asctec_ctrl_input(input_bag, 'asctec/CTRL_INPUT', v.asctec_ctrl_input)
tout, data_out = uniform_resample((('linear', v.imu.t[1:], v.imu.ori_ypr[1:,0]), 
                                   ('linear', v.state.t[1:], v.state.ori_ypr[1:,0])), 0.02)

def plot_axis_control(v, axis):
    axismap = {'roll': (v.control.roll_cmd, 2, +1, v.asctec_ctrl_input.roll_cmd, -1),
               'pitch': (v.control.pitch_cmd, 1, -1, v.asctec_ctrl_input.pitch_cmd, -1),
               'yaw': (v.control.yaw_cmd, 0, -1, v.asctec_ctrl_input.yaw_rate_cmd, +1) # not sure about the multiplier here
               }
    control_mode_cmd, state_axis, imu_mult, asctec_cmd, asctec_cmd_mult = axismap[axis]
    newfig("%s Axis Control" % axis.capitalize(), "time [s]", "%s [deg]" % axis.capitalize())
    # np.clip() and the [1:] stuff in the following to attempt deal with bogus initial data points in IMU data:
    plt.plot(v.control.t, control_mode_cmd, label='cmd (from mux)')
    plt.plot(v.state.t[1:], v.state.ori_ypr[1:,state_axis], label='meas (Vicon)')
    plt.plot(np.clip(v.imu.t[1:], 0, np.inf), imu_mult*v.imu.ori_ypr[1:,state_axis], label='meas (IMU)')
    if axis is not 'yaw':
        plt.plot(v.asctec_ctrl_input.t, asctec_cmd_mult*asctec_cmd, label='cmd (AscTec)')
    # Plot difference between vicon and imu:
    tout, data_out = uniform_resample((('linear', v.imu.t[1:], v.imu.ori_ypr[1:,state_axis]), 
                                       ('linear', v.state.t[1:], v.state.ori_ypr[1:,state_axis])), 
                                       0.02)
    plt.plot(tout, imu_mult*data_out[0][0] - data_out[1][0], label='IMU - Vicon')

    plt.legend()
    
def plot_alt_control(v):
    newfig("Altitude Control", "time [s]", "Alt [m]")
    plt.plot(v.control.t, v.control.alt_cmd, label="cmd")
    plt.plot(v.state.t[1:], v.state.up[1:], label="meas (Vicon)")
    plt.legend()
    
plot_axis_control(v, 'roll')
plot_axis_control(v, 'pitch')
plot_axis_control(v, 'yaw')
plot_alt_control(v)

show()