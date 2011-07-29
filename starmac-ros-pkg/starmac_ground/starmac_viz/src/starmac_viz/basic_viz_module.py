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
from starmac_viz.viz_module_base import VizModuleBase
import rospy

import random
import numpy as np
from math import degrees, radians

import tf.transformations as tft
from tf import TransformBroadcaster

from starmac_roslib.viz.colors import Alpha, Colors
from starmac_roslib.viz.markers import MarkerGroup, VerticalLineMarker, TextMarker, TrailMarker, \
    HeadingMarker, PolygonMarker

# messages:
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from flyer_controller.msg import controller_status

from util import alt_color

XMAX = 1.5
YMAX = 1.5
XMIN = -1.5
YMIN = -1.5
WARN_DIST = 0.5

class BasicVizModule(VizModuleBase):
    """
    A 'basic' visualization module. Provides the following:
    
    - altitude (graphical and text)
    - heading (graphical and text)
    - position (text)
    - trajectory 'trail', with altitude-color-coding
    - ground track, the projection of the trajectory on the ground plane
    - boundary, changes color from green -> yellow -> red depending on proximity of flyer
    """
    
    def __init__(self):
        super(BasicVizModule, self).__init__(id="basic")
        
    def init_viz(self):
        self.mode_t = TextMarker('mode', '', (0,0,0))
        self.heading_marker = HeadingMarker('heading', (0,0,0))
        self.vlm = VerticalLineMarker('altitude', (1,1), color=Colors.WHITE + Alpha(0.5))
        self.alt_t = TextMarker('altitude', '0.0', (0,0,0))
        self.pos_t = TextMarker('position', '0.0,0.0', (0,0,0))
        self.trail = TrailMarker('trail', [], colors=[], color=Colors.BLUE + Alpha(0.8))
        self.trail.set_max_points(500)
        self.ground_trail = TrailMarker('ground_track', [], color=Colors.WHITE + Alpha(0.2))
        self.ground_trail.set_max_points(500)
        self.boundary = PolygonMarker('boundary', ((1.5,1.5,0), (-1.5,1.5,0), (-1.5,-1.5,0), (1.5,-1.5,0)), 
                                      color=Colors.RED+Alpha(0.5), width=0.02)
        self.mg = MarkerGroup(VizModuleBase.mapub)
        self.mg.add(self.mode_t, self.vlm, self.alt_t, self.pos_t, self.trail, 
                    self.ground_trail, self.heading_marker, self.boundary)

#    def init_publishers(self):
#        self.mpub = rospy.Publisher('flyer_viz', Marker)
#        self.mapub = rospy.Publisher('flyer_viz_array', MarkerArray)
#        self.tfbr = TransformBroadcaster()
        
    def init_vars(self):
        self.trajectory = np.empty((1000,3))
        self.n_trajectory = 0

    def init_subscribers(self):
        prefix = self.subscriber_topic_prefix
        #self.llstatus_sub = rospy.Subscriber(prefix+'autopilot/LL_STATUS', LLStatus, self.llstatus_callback) # AscTec specific.. how to make more generic?
        self.odom_sub = rospy.Subscriber(prefix+'estimator/output', Odometry, self.odom_callback)
        self.controller_status_sub = rospy.Subscriber(prefix+'controller/status', controller_status, self.controller_status_callback)

    def publish_timer_callback(self, event):
        now = rospy.Time.now()
        if self.latest_controller_status is not None:
            self.mode_t.set(text=self.latest_controller_status.active_mode)
        if self.latest_odom is not None:
            pos = self.latest_odom.pose.pose.position
            loppo = self.latest_odom.pose.pose.orientation
            ori_ypr = tft.euler_from_quaternion((loppo.x, loppo.y, loppo.z, loppo.w), 'rzyx')
            self.vlm.set((pos.x,pos.y), zend=pos.z)
            self.mode_t.set(pos=(pos.x,pos.y,pos.z - 0.1))
            self.alt_t.set(text="%.3f" % -pos.z, pos=(pos.x,pos.y,pos.z/2))
            self.pos_t.set(text="%.2f, %.2f" % (pos.x, pos.y), pos=(pos.x,pos.y,0.02))
            self.heading_marker.set(pos=(pos.x,pos.y,pos.z), heading=degrees(ori_ypr[0]))
            self.tfbr.sendTransform((pos.x,pos.y,pos.z), (loppo.x, loppo.y, loppo.z, loppo.w), now, '/viz/flyer_axes', '/ned')
            self.tfbr.sendTransform((pos.y,pos.x,-pos.z), (0,0,0,1), now, '/viz/chase', '/enu')
            self.tfbr.sendTransform((0,0,0), tft.quaternion_from_euler(0,radians(180),0,'rzyx'), now, '/viz/onboard', '/viz/flyer_axes')
        VizModuleBase.publish(self,now)
        
    def odom_callback(self, msg):
        self.latest_odom = msg
        pos = self.latest_odom.pose.pose.position
        self.trajectory[self.n_trajectory,:] = (pos.x, pos.y, pos.z)
        self.n_trajectory += 1
        if self.n_trajectory == self.trajectory.shape[0]:
            self.trajectory.resize((self.n_trajectory+1000,3))
        self.trail.append_points([(pos.x, pos.y, pos.z)], [alt_color(-pos.z)])
        self.ground_trail.append_points([(pos.x, pos.y, 0)])
        mindist = min([a-b for a,b in ((XMAX, pos.x), (pos.x, XMIN), (YMAX, pos.y), (pos.y, YMIN))])
        if mindist <= 0:
            bcolor = Colors.RED + Alpha(1.0)
        elif mindist <= WARN_DIST:
            bcolor = Colors.YELLOW + Alpha(1.0)
        else:
            bcolor = Colors.GREEN + Alpha(1.0)
        self.boundary.set(color=bcolor)

    def controller_status_callback(self, msg):
        self.latest_controller_status = msg
