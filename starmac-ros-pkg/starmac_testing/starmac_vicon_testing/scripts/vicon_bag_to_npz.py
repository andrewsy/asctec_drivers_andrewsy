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


import sys

import roslib
roslib.load_manifest('starmac_vicon_testing')

from starmac_tools.load_bag import BagLoader
import numpy as np

def load_bag(bagfile):
    bag = BagLoader(bagfile)
    
    t = bag.vicon__recv__direct_output___header__time
    
    (x, y, z) = (bag.vicon__recv__direct_output_transform_translation_x,
                 bag.vicon__recv__direct_output_transform_translation_y,
                 bag.vicon__recv__direct_output_transform_translation_z)
    
    (qx, qy, qz, qw) = ( bag.vicon__recv__direct_output_transform_rotation_x,
                         bag.vicon__recv__direct_output_transform_rotation_y,
                         bag.vicon__recv__direct_output_transform_rotation_z,
                         bag.vicon__recv__direct_output_transform_rotation_w)

    return t, x, y, z, qx, qy, qz, qw

def load_npz(npzfile):
    npzfile = np.load(npzfile)
    return tuple(npzfile[foo] for foo in ('t','x','y','z','qx','qy','qz','qw'))

def save_npz(npzfile,t, x, y, z, qx, qy, qz, qw):
    np.savez(npzfile, **{'x':x,'y':y,'z':z,'t':t,'qx':qx,'qy':qy,'qz':qz,'qw':qw})
    
def bag_to_npz(bag_fname):
    assert bag_fname.endswith('.bag')
    npz_fname = ''.join(bag_fname.split('.')[:-1]) + '.npz'
    print "Converting %s -> %s ..." % (bag_fname, npz_fname)
    save_npz(npz_fname, *load_bag(bag_fname))

if __name__ == "__main__":
    fnames = sys.argv[1:]
    for fname in fnames:
        bag_to_npz(fname)