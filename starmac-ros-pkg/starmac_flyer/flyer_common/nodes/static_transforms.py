#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, UC Regents
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

"""
static_transforms.py

Uses a tf broadcaster to broadcast static transforms as necessary. These can be specific to
a particular flyer (e.g. the Vicon-to-IMU offset) or global, as in the /enu to /ned offset.

Parameters:
~mode: specifies what to broadcast:
'flyer' - broadcast transform(s) specific to flyer (see 'flyer' parameter)
'global' - broadcast global transform(s)
'both' - broadcast both flyer and global transforms

~flyer: specifies the name of the flyer (e.g. 'grey' or 'yellow')

~period: interval at which to publish the transform(s), in milliseconds
"""
import roslib; roslib.load_manifest('flyer_common')
import rospy
import tf
from math import radians

tft = tf.transformations

global_transforms = (# parent child trans rot (Y-P-R, degrees)
                     ('/enu','/ned', (0,0,0), (90, 0, 180)),
                    )
all_flyer_transforms = (
                        ('flyer_imu', 'flyer_frame', (0, 0, 0), (45, 0, 0)),
                        )

class StaticTransformBroadcaster:
    
    def __init__(self):
        rospy.init_node('static_transforms', anonymous=True)
        self.tfb = tf.TransformBroadcaster()
        self.mode = rospy.get_param('~mode')
        self.flyer = rospy.get_param('~flyer','')
        self.period = rospy.get_param('~period', 1000)
        self.get_frame_params()
        
    def get_frame_params(self):
        self.flyer_transforms = rospy.get_param('frames')
        
    def broadcast_one(self, parent, child, trans, rot):
        rot_rad = [radians(r) for r in rot]
        quat = tft.quaternion_from_euler(rot_rad[0], rot_rad[1], rot_rad[2], 'rzyx')
        self.tfb.sendTransform(trans, quat, rospy.Time.now() + self.dur, child, parent)
        
    def broadcast(self):
        if self.mode == 'global' or self.mode == 'both':
            for transform in global_transforms:
                self.broadcast_one(*transform)
        if self.mode == 'flyer' or self.mode == 'both':
            flyer_prefix = '/'+self.flyer+'/'
            for f in self.flyer_transforms.values():
                self.broadcast_one(flyer_prefix+f['parent'], flyer_prefix+f['child'], f['translation'], f['rotation'])
                            
    def start(self):
        self.dur = rospy.Duration(self.period/1000.0)
        while not rospy.is_shutdown():
            self.broadcast()
            rospy.sleep(self.dur)
        
if __name__ == "__main__":
    stb = StaticTransformBroadcaster()
    stb.start()
