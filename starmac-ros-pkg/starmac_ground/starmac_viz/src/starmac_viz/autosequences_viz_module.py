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
    HeadingMarker, PolygonMarker, SphereMarker
    
#messages:
from flyer_controller.msg import control_mode_hover_info, control_mode_autosequence_info, Autosequence, controller_status
#srv:
from flyer_controller.srv import GetAutosequence, GetAutosequenceRequest, GetAutosequenceResponse


class AutosequencesVizModule(VizModuleBase):
    """
    Visualization module for control_mode_autosequence.
    Provides:
    - polygon representing autosequence path
    - markers for autosequence points
    - labels
    - heading indicators at each point
    
    """
    got_autosequence = False
    current_autosequence = None
    latest_autoseq_info = None
    latest_hover_info = None
    loaded_as_name = ""
    
    def __init__(self):
        super(AutosequencesVizModule, self).__init__(id="autoseq")

    def init_viz(self):
        self.as_poly = PolygonMarker('autosequence', (), 
                              color=Colors.BLUE+Alpha(0.5), width=0.02, closed=False)
        self.as_hover_point_sphere = SphereMarker('hover_point', (0,0,0), radius=0.02, color=Colors.BLUE+Alpha(0.5))
        self.mg = MarkerGroup(VizModuleBase.mapub)
        self.mg.add(self.as_poly, self.as_hover_point_sphere)

        
    def init_vars(self):
        pass
    
    def init_subscribers(self):
        prefix = self.subscriber_topic_prefix
        #self.llstatus_sub = rospy.Subscriber(prefix+'autopilot/LL_STATUS', LLStatus, self.llstatus_callback) # AscTec specific.. how to make more generic?
        self.autoseq_info_sub = rospy.Subscriber(prefix+'control_mode_autosequence/autosequence_info', 
                                                 control_mode_autosequence_info, self.autoseq_info_callback)
        self.hover_info_sub = rospy.Subscriber(prefix+'control_mode_autosequence/info', 
                                                 control_mode_hover_info, self.hover_info_callback)
        self.controller_status_sub = rospy.Subscriber(prefix+'controller/status', controller_status, self.controller_status_callback)

    def publish_timer_callback(self, event):
        if self.got_autosequence:
            as_points = [(p.hover_point.x, p.hover_point.y, 0) for p in self.current_autosequence.points]
            self.as_poly.set(points=as_points)
            if self.latest_hover_info is not None:
                slhi = self.latest_hover_info
                self.as_hover_point_sphere.set(origin=(slhi.north_cmd, slhi.east_cmd, 0))
            now = rospy.Time.now()
            VizModuleBase.publish(self,now)

    def autoseq_info_callback(self, msg):
        #rospy.loginfo('got autoseq info: %s' % str(msg))
        self.latest_autoseq_info = msg

    def hover_info_callback(self, msg):
        self.latest_hover_info = msg

    def _new_autosequence(self):
        return (self.latest_autoseq_info is not None 
                and len(self.latest_autoseq_info.defined_autosequences) > 0
                and (self.latest_autoseq_info.current_autosequence != self.loaded_as_name))

    def controller_status_callback(self, msg):
        self.latest_controller_status = msg
        #rospy.loginfo('got controller status: %s' % str(msg))
        if msg.active_mode == 'autosequence':
            if self._new_autosequence():
                self.got_autosequence = False
            if (not self.got_autosequence 
                    and self.latest_autoseq_info is not None 
                    and len(self.latest_autoseq_info.defined_autosequences) > 0 
                    and len(self.latest_autoseq_info.current_autosequence) > 0):
                as_name = self.latest_autoseq_info.current_autosequence
                self.get_autosequence(as_name)
            
    def get_autosequence(self, as_name):
        get_auto_proxy = rospy.ServiceProxy('control_mode_autosequence/get_autosequence', GetAutosequence)
        rospy.loginfo("Asking for autosequence '%s'..." % as_name)
        resp = get_auto_proxy(autosequence_name=as_name)
        if resp.found:
            self.current_autosequence = resp.autosequence
            rospy.loginfo('Got autosequence: %s' % str(self.current_autosequence))
            self.got_autosequence = True
            self.loaded_as_name = as_name
        else:
            rospy.logerr("Service call failed: autosequence '%s' not found" % as_name)
