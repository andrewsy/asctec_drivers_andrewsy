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

import os
import roslib
roslib.load_manifest('starmac_templates') # change this to your package name
import rospy
from starmac_roslib.timers import Timer
# Message imports:
# Make sure corresponding packages are listed as dependencies in package manifest!
from std_msgs.msg import Bool, Float64

class ProgramRunner(object):
    """
    ROS node to run an arbitrary program. Useful in launch files, e.g.:
    
    <node pkg="starmac_tools" type="program_runner.py" name="run_watch_df" args="watch -n 2 df -h" launch-prefix="xterm-e"/>
    """
    def __init__(self):
        # Remember that init_node hasn't been called yet so only do stuff here that
        # doesn't require node handles etc.
        pass

    def start(self):
        rospy.init_node('program_runner')
        self.init_timers()
        rospy.spin()
        
    def init_timers(self):
        self.runprog_timer = Timer(rospy.Duration(1/10.0), self.runprog_timer_callback, oneshot=True)
        
    def runprog_timer_callback(self, event):
        myargv = rospy.myargv()[1:]
        cmd = " ".join(myargv)
        rospy.loginfo('Executing command: ' + cmd)
        retval = os.system(cmd)
        msg = 'Program terminated with exit status %d' % retval
        rospy.loginfo(msg)
        rospy.signal_shutdown(msg)
        
if __name__ == "__main__":
    self = ProgramRunner()
    self.start()