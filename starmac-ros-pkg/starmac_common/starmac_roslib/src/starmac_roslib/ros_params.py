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

# Author: Patrick Bouffard <bouffard@eecs.berkeley.edu>

import rospy

class ROSParams(object):
    """
    Helper class to be used for grabbing parameters off the parameter server at startup.
    
    Example usage:
    
    from starmac_roslib.ros_params import ROSParams
    
    class MyNodeParams(ROSParams):
        param_defaults = {'param_one': 2.3, 'param_two': 0.0}
        
    class MyNode:
    ...
        def init_params(self):
            self.params = MyNodeParams()
            
        def some_method(self):
            foo = self.params.param_one * 42
    """
    param_defaults = {}
    
    def __init__(self, get_params=True, quiet=False):
        if get_params:
            self.get_params(quiet=quiet)
        
    def get_params(self,quiet=False):
        """
        Loads parameter values from the parameter server, using defaults from self.param_defaults
        Unless the quiet parameter is set, an info message is printed giving the parameter name,
        the set value of the parameter, its type and default value.
        """  
        for param_name, default_value in self.param_defaults.items():
            value = rospy.get_param("~" + param_name, default_value)
            self.__setattr__(param_name,value)
            if not quiet:
                rospy.loginfo('Parameter %s = %s (%s) (default %s)' % 
                              (param_name, 
                               str(value), 
                               str(type(value)), 
                               str(self.param_defaults[param_name])
                               ))

