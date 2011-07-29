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
roslib.load_manifest('vicon_mocap') # change this to your package name
import rospy
from starmac_roslib.timers import Timer
# Message imports:
# Make sure corresponding packages are listed as dependencies in package manifest!
from geometry_msgs.msg import TransformStamped
from tf.broadcaster import TransformBroadcaster

class DummyViconNode(object):
    """
    Example python node including reading parameters, publishers and subscribers, and timer.
    """
    def __init__(self):
        # Remember that init_node hasn't been called yet so only do stuff here that
        # doesn't require node handles etc.
        pass

    def start(self):
        rospy.init_node('python_node_example')
        self.tfb = TransformBroadcaster()
        self.init_params()
        self.init_publishers()
        self.init_timers()
        rospy.spin()
        
    def init_params(self):
        pass
#        self.param_one = rospy.get_param("~param_one", 100.0)
#        self.param_two = rospy.get_param("~param_two", False)
    
    def init_publishers(self):
        self.pub = rospy.Publisher('vicon_recv_direct/output', TransformStamped)
        
    def init_timers(self):
        self.vicon_timer = Timer(rospy.Duration(1/120.0), self.vicon_timer_callback) # Call at 10 Hz
                                          
    def vicon_timer_callback(self, event):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.transform.rotation.w = 1.0
        self.pub.publish(msg)
        self.tfb.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), '/pelican1/flyer_vicon', '/enu')
    

        
        
if __name__ == "__main__":
    self = DummyViconNode()
    self.start()