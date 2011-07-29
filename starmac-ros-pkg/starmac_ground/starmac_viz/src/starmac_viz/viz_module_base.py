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

from math import degrees, radians

#import roslib
#roslib.load_manifest('starmac_viz')
import rospy
from tf import TransformBroadcaster

from visualization_msgs.msg import Marker, MarkerArray

from starmac_roslib.timers import Timer


class VizModuleBase(object):
    """
    Abstract Base Class
    
    Visualization modules should inherit from this class.
    """
    #latest_llstatus = None
    latest_odom = None
    latest_controller_status = None
    modules = {}
    mpub = None
    mapub = None
    tfbr = None
    
    def __init__(self, id=""):
        self.id = id
        if id in VizModuleBase.modules:
            raise Error("Visualization module with id '%s' already exists" % id)
        VizModuleBase.modules[id] = self
        self.init_params()
        self.init_publishers()
        self.init_vars()
        self.init_viz()
        self.init_subscribers()
        self.init_timers()

    def init_params(self):
        self.publish_freq = rospy.get_param("~publish_freq", 15.0)
        self.subscriber_topic_prefix = rospy.get_param("~subscriber_topic_prefix", 'downlink/')
        
    def init_timers(self):
        self.publish_timer = Timer(rospy.Duration(1/self.publish_freq), self.publish_timer_callback)

    def init_publishers(self):
        VizModuleBase.mpub = rospy.Publisher('flyer_viz', Marker)
        VizModuleBase.mapub = rospy.Publisher('flyer_viz_array', MarkerArray)
        VizModuleBase.tfbr = TransformBroadcaster()

    def publish(self, stamp):        
        self.mg.publish(stamp)

    # Following are 'pure virtual' methods -- they _must_ be defined in the derived class
    def init_vars(self):
        raise NotImplementedError 
        
    def init_viz(self):
        raise NotImplementedError 
        
    def init_subscribers(self):
        raise NotImplementedError 
        
    def publish_timer_callback(self):
        raise NotImplementedError 




