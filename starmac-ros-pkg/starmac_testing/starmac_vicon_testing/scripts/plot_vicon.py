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
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

from starmac_tools.load_bag import BagLoader

def add_circle(x, y, radius, **kwargs):
    e = Ellipse((x,y), radius*2., radius*2., fill=False, **kwargs)
    plt.gca().add_artist(e)
    return e

def add_title_and_labels(title=None,xlabel=None,ylabel=None):
    if title is not None:
        plt.title(title)
    if xlabel is not None:
        plt.xlabel(xlabel)
    if ylabel is not None:
        plt.ylabel(ylabel)

def newfig(subplot=111,title=None,xlabel=None,ylabel=None):
    plt.figure()
    plt.grid(True)
    plt.subplot(subplot)
    add_title_and_labels(title, xlabel, ylabel)

def subplot_vs_time(s,title=None,ylabel=None):
    subplot(s,title=title,xlabel='t [s]',ylabel=ylabel)

def subplot(s,title=None,xlabel=None,ylabel=None):
    plt.subplot(s)
    plt.grid(True)
    add_title_and_labels(title, xlabel, ylabel)
        
def fig_xyz():
    newfig(subplot=311)
    subplot_vs_time(311,title='x',ylabel='[m]')
    plt.plot(t,x,'.')
    plt.axhline(meanval[0], color='r')
    subplot_vs_time(312,title='y',ylabel='[m]')
    plt.plot(t,y,'.')
    plt.axhline(meanval[1], color='r')
    subplot_vs_time(313,title='z',ylabel='[m]')
    plt.plot(t,z,'.')
    plt.axhline(meanval[2], color='r')

def fig_qxyzw():
    chars = 'xyzw'
    newfig(subplot=221)
    for i, char in zip(range(4),chars):
        subplot_vs_time(220+i,title='q'+char)
        plt.plot(t,globals()['q'+char],'.')

def fig_dt():
    newfig(title="dt[k] := t[k] - t[k-1]", ylabel='dt [s]', xlabel='t [s]')
    plt.plot(t[1:],dt)
    
def fig_xy():
    newfig(title="Top view (y vs. x)",xlabel="x [m]",ylabel="y [m]")
    plt.plot(x,y,'.')
    plt.axis('equal')
    plt.plot(meanval[0],meanval[1],'rx')
    add_circle(meanval[0],meanval[1],0.001)
    add_circle(meanval[0],meanval[1],0.01)
    
def fig_xz():
    newfig(title="Side view (z vs. x)",xlabel="x [m]",ylabel="z [m]")
    plt.plot(z,x,'.')
    plt.axis('equal')
    plt.plot(meanval[2],meanval[0],'rx')
    add_circle(meanval[2],meanval[0],0.001)
    add_circle(meanval[2],meanval[0],0.01)
    
def fig_yz():
    newfig(title="Side view (z vs. y)",xlabel="y [m]",ylabel="z [m]")
    plt.plot(z,y,'.')
    plt.axis('equal')
    plt.plot(meanval[2],meanval[1],'rx')
    add_circle(meanval[2],meanval[1],0.001)
    add_circle(meanval[2],meanval[1],0.01)
    
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

def save_npz(npzfile):
    np.savez(npzfile, **{'x':x,'y':y,'z':z,'t':t,'qx':qx,'qy':qy,'qz':qz,'qw':qw})

if __name__ == '__main__':
    fname = sys.argv[1]
    
    if fname.endswith('.bag'):
        t,x,y,z, qx, qy, qz, qw = load_bag(fname)
    elif fname.endswith('.npz'):
        t,x,y,z, qx, qy, qz, qw = load_npz(fname)
    
    meanval = np.array([np.mean(foo) for foo in (x,y,z)])
    
    dt = t[1:]-t[:-1]
    fig_xyz()
    fig_dt()
    fig_xy()
    fig_xz()
    fig_yz()
    fig_qxyzw()
    
    plt.show()