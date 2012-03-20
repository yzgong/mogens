#!/usr/bin/env python
# *-* encoding: utf-8

"""
Copyright (c) 2011-2012, Rune Havnung Bakken, Dag Stuan and Geir Hauge at
the Faculty of Informatics and e-Learning, Sør-Trøndelag University College.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Sør-Trøndelag University College nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import matplotlib
matplotlib.use('TkAgg')

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, \
                                              NavigationToolbar2TkAgg
import xml.dom.minidom as minidom
from xml.parsers.expat import ExpatError, errors
import math

from matplotlib.figure import Figure
from matplotlib.text import Text
from matplotlib.font_manager import FontProperties
import matplotlib.mlab as mlab

from matplotlib.offsetbox import AnchoredOffsetbox, TextArea, DrawingArea, VPacker

# http://effbot.org/tkinterbook/
import Tkinter as Tk
import tkFileDialog
import tkMessageBox
import os
import sys
import threading
import Queue
import atexit
import signal
import numpy

graphtypes = (
    "Euclidean distance", 
    "X distance", 
    "Y distance", 
    "Z distance", 
    "Box plot"
)
joint_dicts = {}

joint_labels={
    'head':'Head',
    'neck':'Neck',
    'clav':'Chest',
    'lsho':'LSho',
    'rsho':'RSho',
    'lelb':'LElb',
    'relb':'RElb',
    'lwrist':'LHand',
    'rwrist':'Rhand',
    'lfingertip':'LFTip',
    'rfingertip':'RFTip',
    'thorax':'UBack',
    'upperback':'Back',
    'lowerback':'LBack',
    'lhip':'LHip',
    'rhip':'RHip',
    'lkne':'LKnee',
    'rkne':'RKnee',
    'lank':'LAnk',
    'rank':'RAnk',
    'ltoes':'LFoot',
    'rtoes':'RFoot',
    'ltoetip':'LToe ',
    'rtoetip':'RToe '
    }


def read(file):
    """ 
    Reads and parses an XML-file with joint-positions.

    The xml is of the form:
    <?xml version="1.0" ?>
    <xml>
      <frame id="[frame number]">
        <joint ignore="[true|false]" name="[jointname]">
          <pos x="[X-value]" y="[Y-value]" z="[Z-value]"/>
        </joint>
        <joint ...
      </frame>
      <frame ...
    </xml>
    
    Returns a dict of the form 
    {
        'joint_name1': {
            [1]: (x1, y1, z1), 
            [2]: (x2, y2, z2),
            ...,
        },
        'joint_name2': {
            [1]: (x1, y1, z1), 
            [2]: (x2, y2, z2),
            ...,
        },
    }
    """

    joint_dict = {}
    doc = minidom.parse(file)
        
    frames = doc.getElementsByTagName("frame")

    for frame in frames:
        frame_num = int(frame.getAttribute("id"))
        joints = frame.getElementsByTagName("joint")
        for joint in joints:
            if joint.getAttribute("ignore").lower() in ("true", "1", "yes"):
                joint_dict[joint_name][frame_num]=None
            else:
                joint_name = joint.getAttribute("name")
                if joint_name not in joint_dict:
                    joint_dict[joint_name] = {}
                pos = joint.getElementsByTagName("pos")[0]
                joint_pos = map(lambda x:float(pos.getAttribute(x))*1000., ('x','y','z'))
                joint_dict[joint_name][frame_num] = joint_pos
    file.close()
    return joint_dict

def joint_euclidian_diff(joint_pos1, joint_pos2):
    """
    Calculates euclidian distance between two sets of vertices.

    joint_pos1 and joint_pos2 are dicts with frame_number as key, and
    a tuple of xyx-coordinates as value.

    Returns a tuple of two lists, the X-values (frane numbers) and
    Y-values (euclidian distance)
    """
    x_values = []
    y_values = []
    for frame_num in joint_pos1.keys():
        vec1 = joint_pos1[frame_num]
        distance=0.0
        if frame_num in joint_pos2:
            vec2 = joint_pos2[frame_num]
            if vec2:
                distance = math.sqrt(sum(map(lambda x,y: (y-x)**2, vec1, vec2)))
        x_values.append(frame_num)
        y_values.append(distance)
    return x_values, y_values

def joint_dist(joint_pos1, joint_pos2, direction):
    """
    Calculates the distance between either the X, Y or X dimension of vertices.

    joint_pos1 and joint_pos2 are dicts with frame_number as key, and
    a tuple of xyx-coordinates as value.

    direction is the dimension to calculate; 'x', 'y' or 'z'.

    Returns a tuple of two lists, the X-values (frame numbers) and
    Y-values (distance)
    """

    x_values = []
    y_values = []
    
    if direction == 'x':
        dirNum = 0
    elif direction == 'y':
        dirNum = 1
    elif direction == 'z':
        dirNum = 2
    
    for frame_num in joint_pos1.keys():
        distance=0
        dir_1 = joint_pos1[frame_num][dirNum]
        if frame_num in joint_pos2:
            dir_2 = joint_pos2[frame_num][dirNum]
            distance = dir_2 - dir_1
        x_values.append(frame_num)
        y_values.append(distance)
    return x_values, y_values

def joint_average_euclidian(joint_names, pos1, pos2):
    """
    Calculates the average euclidian distance for each frame.

    joint_names is a list of joint names, used as keys for pos1 and pos2,

    pos1 and pos2 are dicts of the form returned by read(file),

    Returns a tuple of two lists, the X-values (frane numbers) and
    Y-values (average euclidian distance)
    """

    x_values = []
    y_values = []
    length = len(joint_names)
    
    for frame_num in pos1[pos1.keys()[0]]:
        frame_sum = 0
        
        for name in joint_names:            
            distance=0
            vec1 = pos1[name][frame_num]
            if frame_num in pos2[name]:
                vec2 = pos2[name][frame_num]
                if vec2:
                    distance = math.sqrt(sum(map(lambda x,y: (y-x)**2, vec1, vec2)))
            frame_sum += distance
        
        x_values.append(frame_num)
        y_values.append(frame_sum / length)
    
    return x_values, y_values
    
def joint_average_dist(joint_names, pos1, pos2, direction):
    """
    Calculates the average distance of a given dimension for each frame.

    joint_names is a list of joint names, used as keys for pos1 and pos2,

    pos1 and pos2 are dicts of the form returned by read(file),

    direction is the dimensions given as either 'x', 'y' or 'z'.

    Returns a tuple of two lists, the X-values (frane numbers) and
    Y-values (average distance)
    """

    x_values = []
    y_values = []
    length = len(joint_names)
    
    if direction == 'x':
        dirNum = 0
    elif direction == 'y':
        dirNum = 1
    elif direction == 'z':
        dirNum = 2
    
    for frame_num in pos1[pos1.keys()[0]]:
        frame_sum = 0
        
        for name in joint_names:                        
            distance=0
            dir_1 = pos1[name][frame_num][dirNum]
            if frame_num in pos2[name]:
                dir_2 = pos2[name][frame_num][dirNum]
                distance = dir_2 - dir_1
            
            frame_sum += distance
        
        x_values.append(frame_num)
        y_values.append(frame_sum / length)
    
    return x_values, y_values
    
def standard_deviation_euclidian(joint_names, pos1, pos2):
    """
    Calculates the standard deviation of the euclidian distance for each
    frame.

    joint_names is a list of joint names, used as keys for pos1 and pos2,

    pos1 and pos2 are dicts of the form returned by read(file),

    Returns a tuple of four lists, the X-values (frame numbers) and
    Y-values (standard deviation), a list of the minimum Y-values and a list
    of the maximum Y-values.
    """
    y_values = []
    y_valuesmin = []
    y_valuesmax = []
    
    x_values, avg_euclidian = joint_average_euclidian(joint_names, pos1, pos2)
    length = len(joint_names)
    
    for frame_num in pos1[pos1.keys()[0]]:
        frame_sum = 0
        
        for name in joint_names:
            distance=0
            vec1 = pos1[name][frame_num]
            if frame_num in pos2[name]:
                vec2 = pos2[name][frame_num]
                if vec2:
                    distance = math.sqrt(sum(map(lambda x,y: (y-x)**2, vec1, vec2)))
            
            variance = (distance - avg_euclidian[frame_num-1])**2
            frame_sum += variance
        
        frame_val = math.sqrt(frame_sum/length)
        
        y_values.append(frame_val)
        y_valuesmin.append(avg_euclidian[frame_num-1] - frame_val)
        y_valuesmax.append(avg_euclidian[frame_num-1] + frame_val)
        
    return x_values, (y_values, y_valuesmin, y_valuesmax)

def standard_deviation_dist(joint_names, pos1, pos2, direction):    
    """
    Calculates the standard deviation of the distance of a given dimension
    for each frame.

    joint_names is a list of joint names, used as keys for pos1 and pos2,

    pos1 and pos2 are dicts of the form returned by read(file).

    direction is the dimensions given as either 'x', 'y' or 'z'.

    Returns a tuple of four lists, the X-values (frame numbers) and
    Y-values (standard deviation), a list of the minumum Y-values and a list
    the maximum Y-values.
    """
    y_values = []
    y_valuesmin = []
    y_valuesmax = [] 
    
    x_values, avg_dist = joint_average_dist(joint_names, pos1, pos2, direction)
    length = len(joint_names)
    
    if direction == 'x':
        dirNum = 0
    elif direction == 'y':
        dirNum = 1
    elif direction == 'z':
        dirNum = 2
    
    for frame_num in pos1[pos1.keys()[0]]:
        frame_sum = 0
        
        for name in joint_names:
            distance=0
            dir_1 = pos1[name][frame_num][dirNum]
            if frame_num in pos2[name]:
                dir_2 = pos2[name][frame_num][dirNum]
                distance = dir_2 - dir_1
            
            variance = (distance - avg_dist[frame_num-1])**2
            frame_sum += variance
        
        frame_val = math.sqrt(frame_sum/length) 
                
        y_values.append(frame_val)
        y_valuesmin.append(avg_dist[frame_num-1] - frame_val)
        y_valuesmax.append(avg_dist[frame_num-1] + frame_val)
    
    return x_values, (y_values, y_valuesmin, y_valuesmax)

def calculate_euclideans(joint_names,pos1,pos2):
    dists=[]
    for frame_num in pos1[pos1.keys()[0]]:
        frame_sum = 0
        
        for name in joint_names:            
            vec1 = pos1[name][frame_num]
            if frame_num in pos2[name]:
                vec2 = pos2[name][frame_num]
                if vec2:
                    dists.append(math.sqrt(sum(map(lambda x,y: (y-x)**2, vec1, vec2))))
    return numpy.array(dists)
    
def avg_across_move_euclidian(joint_names, pos1, pos2):
    """
    Calculates the average of the euclidian distance of the whole motion.

    joint_names is a list of joint names, used as keys for pos1 and pos2,

    pos1 and pos2 are dicts of the form returned by read(file).

    Returns the value as a float.
    """
    # x_values, y_values = joint_average_euclidian(joint_names, pos1, pos2)
    # 
    # num_frames = len(pos1[pos1.keys()[0]])
    # 
    # frame_sum = 0
    # ignored_frames=0
    # for frame_val in y_values:
    #     if frame_val==0:
    #         ignored_frames+=1
    #     frame_sum += frame_val
    # 
    dists=calculate_euclideans(joint_names,pos1,pos2)
    #return frame_sum / (num_frames-ignored_frames)
    return dists.mean()
    
def sd_across_move_euclidian(joint_names, pos1, pos2):
    """
    Calculates the standard deviation of the euclidian distance of the whole
    motion.

    joint_names is a list of joint names, used as keys for pos1 and pos2,

    pos1 and pos2 are dicts of the form returned by read(file).

    Returns the value as a float.
    """
    #avg_across_move = avg_across_move_euclidian(joint_names, pos1, pos2)
    #x_values, y_values = joint_average_euclidian(joint_names, pos1, pos2)

    #print avg_across_move
    #
    #num_frames = len(pos1[pos1.keys()[0]])
    #
    #frame_sum = 0
    #for frame_val in y_values:
    #    frame_sum += (frame_val - avg_across_move)**2
    
    dists=calculate_euclideans(joint_names,pos1,pos2)
    #return math.sqrt(frame_sum / num_frames)
    return dists.std(ddof=1)
    
def avg_across_move_dist(joint_names, pos1, pos2, direction):
    """
    Calculates the average distance of a given dimension for the whole motion.

    joint_names is a list of joint names, used as keys for pos1 and pos2,

    pos1 and pos2 are dicts of the form returned by read(file).

    direction is the dimensions given as either 'x', 'y' or 'z'.

    Returns the value as a float.
    """
    x_values, y_values = joint_average_dist(joint_names, pos1, pos2, direction)
    
    num_frames = len(pos1[pos1.keys()[0]])
    
    frame_sum = 0
    for frame_val in y_values:
        frame_sum += frame_val
    
    return frame_sum / num_frames
    
def sd_across_move_dist(joint_names, pos1, pos2, direction):
    """
    Calculates the standard deviation of the distance of a given dimension
    for the whole motion.

    joint_names is a list of joint names, used as keys for pos1 and pos2,

    pos1 and pos2 are dicts of the form returned by read(file).

    direction is the dimensions given as either 'x', 'y' or 'z'.

    Returns the value as a float.
    """
    avg_across_move = avg_across_move_dist(joint_names, pos1, pos2, direction)
    x_values, y_values = joint_average_dist(joint_names, pos1, pos2, direction)
    
    num_frames = len(pos1[pos1.keys()[0]])
    
    frame_sum = 0
    for frame_val in y_values:
        frame_sum += (frame_val - avg_across_move)**2
    
    return math.sqrt(frame_sum / num_frames)

def _min_max(values, cur_min, cur_max):
    """
    Returns the minimum and maximum values of the given list of values
    """
    if cur_min is None:
        cur_min = min(values)
    else:
        cur_min = min( cur_min, min(values) )

    if cur_max is None:
        cur_max = max(values)
    else:
        cur_max = max( cur_max, max(values) )

    return cur_min, cur_max
        
def make_plot_lines(subplot, dicts, graphtype):
    """ 
    Plots data on a matplotlib canvas

    subplot is the axes to populate.

    dicts is an iterable with two dictionaries of the form returned by
    read(file)

    graphtype is the type of graphs to plot. The value must be one of the
    values from the global graphtypes list.

    Returns a generator of tuples of a string and a list of the matplotlib 
    lines.
    """
    xmin = None
    xmax = None
    ymin = None
    ymax = None
    dict1 = dicts[0]
    dict2 = dicts[1]
    joint_names = set(dict1.keys()).intersection(dict2.keys())
    if graphtype == graphtypes[0]:
        for name in joint_names:
            x_values, y_values = joint_euclidian_diff(dict1[name], dict2[name])
            xmin, xmax = _min_max(x_values, xmin, xmax)
            ymin, ymax = _min_max(y_values, ymin, ymax)
            lines = subplot.plot(x_values, y_values, label=name, visible=False)
            yield name, lines

        x_values, y_values = joint_average_euclidian(joint_names, dict1, dict2)
        xmin, xmax = _min_max(x_values, xmin, xmax)
        ymin, ymax = _min_max(y_values, ymin, ymax)
        lines = subplot.plot(x_values, y_values, label='avg_euclidian', visible=False)
        yield 'average', lines

        x_values, y_values = standard_deviation_euclidian(joint_names, dict1, dict2)
        xmin, xmax = _min_max(x_values, xmin, xmax)
        ymin, ymax = _min_max(y_values[0], ymin, ymax)
        ymin, ymax = _min_max(y_values[1], ymin, ymax)
        ymin, ymax = _min_max(y_values[2], ymin, ymax)
        subplot.set_xlim(xmin, xmax)
        subplot.set_ylim(ymin, ymax)
        lines = subplot.plot(x_values, y_values[0], label='sd_euclidian', visible=False)
        yield 'standard deviation', lines

        lines = []
        lines.extend(subplot.plot(x_values, y_values[1], label='avg - sd_euclidian', visible=False))
        lines.extend(subplot.plot(x_values, y_values[2], label='avg + sd_euclidian', visible=False))
        x, y = mlab.poly_between(x_values, y_values[1], y_values[2])
        lines.extend(subplot.fill(x, y,  alpha=.3, visible=False))
        yield 'average sd.', lines
        
        #yield 'Average %.3f'% (avg_across_move_euclidian(joint_names, dict1, dict2),), None
        yield 'Mean: %.3f'% (avg_across_move_euclidian(joint_names, dict1, dict2),), None
        #yield 'Standard deviation %.3f' % (sd_across_move_euclidian(joint_names, dict1, dict2),), None
        yield 'Std. dev.: %.3f' % (sd_across_move_euclidian(joint_names, dict1, dict2),), None
    elif graphtype == graphtypes[1]:
        for name in joint_names:
            x_values, y_values = joint_dist(dict1[name], dict2[name], 'x')
            xmin, xmax = _min_max(x_values, xmin, xmax)
            ymin, ymax = _min_max(y_values, ymin, ymax)
            lines = subplot.plot(x_values, y_values, label=name, visible=False)
            yield name, lines

        x_values, y_values = joint_average_dist(joint_names, dict1, dict2, 'x')
        xmin, xmax = _min_max(x_values, xmin, xmax)
        ymin, ymax = _min_max(y_values, ymin, ymax)
        subplot.set_xlim(xmin, xmax)
        subplot.set_ylim(ymin, ymax)
        lines = subplot.plot(x_values, y_values, label='avg_x', visible=False)
        yield 'average X', lines
        
        x_values, y_values = standard_deviation_dist(joint_names, dict1, dict2, 'x')
        lines = subplot.plot(x_values, y_values[0], label='sd_euclidian', visible=False)
        yield 'standard deviation X', (lines)

        lines = []
        lines.extend(subplot.plot(x_values, y_values[1], label='avg - sd_euclidian', visible=False))
        lines.extend(subplot.plot(x_values, y_values[2], label='avg + sd_euclidian', visible=False))
        x, y = mlab.poly_between(x_values, y_values[1], y_values[2])
        lines.extend(subplot.fill(x, y, alpha=.3, visible=False))
        yield 'average sd X', lines
        
        #yield 'Average %.3f'% (avg_across_move_dist(joint_names, dict1, dict2, 'x'),), None
        yield 'Mean: %.3f'% (avg_across_move_dist(joint_names, dict1, dict2, 'x'),), None
        #yield 'Standard deviation %.3f' % (sd_across_move_dist(joint_names, dict1, dict2, 'x'),), None
        yield 'Std. dev.: %.3f' % (sd_across_move_dist(joint_names, dict1, dict2, 'x'),), None
    elif graphtype == graphtypes[2]:
        for name in joint_names:
            x_values, y_values = joint_dist(dict1[name], dict2[name], 'y')
            xmin, xmax = _min_max(x_values, xmin, xmax)
            ymin, ymax = _min_max(y_values, ymin, ymax)
            lines = subplot.plot(x_values, y_values, label=name, visible=False)
            yield name, lines

        x_values, y_values = joint_average_dist(joint_names, dict1, dict2, 'y')
        xmin, xmax = _min_max(x_values, xmin, xmax)
        ymin, ymax = _min_max(y_values, ymin, ymax)
        subplot.set_xlim(xmin, xmax)
        subplot.set_ylim(ymin, ymax)
        lines = subplot.plot(x_values, y_values, label='avg_y', visible=False)
        yield 'average Y', lines
        
        x_values, y_values = standard_deviation_dist(joint_names, dict1, dict2, 'y')
        lines = subplot.plot(x_values, y_values[0], label='sd_euclidian', visible=False)
        yield 'standard deviation Y', (lines)

        lines = []
        lines.extend(subplot.plot(x_values, y_values[1], label='avg - sd_euclidian', visible=False))
        lines.extend(subplot.plot(x_values, y_values[2], label='avg + sd_euclidian', visible=False))
        x, y = mlab.poly_between(x_values, y_values[1], y_values[2])
        lines.extend(subplot.fill(x, y, alpha=.3, visible=False))
        yield 'average sd. Y', lines
        
        #yield 'Average %.3f'% (avg_across_move_dist(joint_names, dict1, dict2, 'y'),), None
        yield 'Mean: %.3f'% (avg_across_move_dist(joint_names, dict1, dict2, 'y'),), None
        #yield 'Standard deviation %.3f' % (sd_across_move_dist(joint_names, dict1, dict2, 'y'),), None
        yield 'Std. dev.: %.3f' % (sd_across_move_dist(joint_names, dict1, dict2, 'y'),), None
    elif graphtype == graphtypes[3]:
        for name in joint_names:
            x_values, y_values = joint_dist(dict1[name], dict2[name], 'z')
            xmin, xmax = _min_max(x_values, xmin, xmax)
            ymin, ymax = _min_max(y_values, ymin, ymax)
            lines = subplot.plot(x_values, y_values, label=name, visible=False)
            yield name, lines

        x_values, y_values = joint_average_dist(joint_names, dict1, dict2, 'z')
        xmin, xmax = _min_max(x_values, xmin, xmax)
        ymin, ymax = _min_max(y_values, ymin, ymax)
        subplot.set_xlim(xmin, xmax)
        subplot.set_ylim(ymin, ymax)
        lines = subplot.plot(x_values, y_values, label='avg_z', visible=False)
        yield 'average Z', lines
        
        x_values, y_values = standard_deviation_dist(joint_names, dict1, dict2, 'z')
        lines = subplot.plot(x_values, y_values[0], label='sd_euclidian', visible=False)
        yield 'standard deviation Z', (lines)

        lines = []
        lines.extend(subplot.plot(x_values, y_values[1], label='avg - sd_euclidian', visible=False))
        lines.extend(subplot.plot(x_values, y_values[2], label='avg + sd_euclidian', visible=False))
        x, y = mlab.poly_between(x_values, y_values[1], y_values[2])
        lines.extend(subplot.fill(x, y, alpha=.3, visible=False))
        yield 'average sd. Z', lines
        
        #yield 'Average %.3f'% (avg_across_move_dist(joint_names, dict1, dict2, 'z'),), None
        yield 'Mean: %.3f'% (avg_across_move_dist(joint_names, dict1, dict2, 'z'),), None
        #yield 'Standard deviation %.3f' % (sd_across_move_dist(joint_names, dict1, dict2, 'z'),), None
        yield 'Std. dev.: %.3f' % (sd_across_move_dist(joint_names, dict1, dict2, 'z'),), None

def createBoxPlot(subplot,dicts):
    global joint_labels
    dict1 = dicts[0]
    dict2 = dicts[1]
    joint_names = set(dict1.keys()).intersection(dict2.keys())
    joints=[]
    names=[]
    for name in joint_names:
        _,y_values = joint_euclidian_diff(dict1[name], dict2[name])
        #xmin, xmax = _min_max(x_values, xmin, xmax)
        #ymin, ymax = _min_max(y_values, ymin, ymax)
        joints.append(y_values)
        names.append(joint_labels[name])

    #subplot.clear()
    bplot=subplot.boxplot(joints,sym='b+')
    subplot.get_xaxis().set_ticklabels(names,rotation='vertical')
    subplot.get_xaxis().set_tick_params(labelsize=20,pad=10)
    subplot.set_xlabel(None,visible=False)
    subplot.set_ylabel('Error (mm)',size=20)
    subplot.get_yaxis().set_tick_params(labelsize=20,pad=10)
    return bplot

class Message(object):
    """ A simple struct to hold a type and value. """
    def __init__(self, type, value):
        self.type = type
        self.value = value

class FileReaderThread(threading.Thread):
    """ 
    This is a thread class to handle parsing of the XML-files in a
    separate thread.

    queue must be an instance of Queue.Queue. Messages will be put into the
    queue for given events.

        - "file_progress" for use with progress bar. Argument is a tuple of
          percentage and a message.
        - "gt_file_add" when a ground truth file has been parsed. Argument
          is the basename of the filename.
        - "filelist_add" when a candidate file has been parsed. Argument is
          the basename of a filename.
    """
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.filequeue = Queue.Queue()
        self.running = False

    def process_files(self, files):
        """ Queue files for processing. 
        
        If files is a list of file objects, the files will be treated as
        candidate files.
        If files is a single file object, the file is treated as a ground
        truth file.
        """
        
        self.filequeue.put(files)

    def done(self):
        """ Ends the thread """
        self.running = False

    def run(self):
        self.running = True
        files = None
        while self.running:
            try:
                files = self.filequeue.get(timeout=5)
            except Queue.Empty:
                continue
            if type(files) == file:
                base = os.path.basename(files.name)
                self.queue.put(
                        Message("file_progress", (0, "Reading " + base))
                )
                try:
                    joint_dicts[base] = read(files)
                except ExpatError, e:
                    tkMessageBox.showwarning(
                            "XML parsing failed",
                            "%s: %s" % (files.name, str(e))
                    )
                    self.queue.put(Message("file_progress", (1, "Failed")))
                    continue
                self.queue.put(Message("gt_file_add", base))
                self.queue.put(
                        Message("file_progress", (1, "Done reading " + base))
                )
            else:
                n = len(files)
                for i, f in enumerate(files):
                    base = os.path.basename(f.name)
                    self.queue.put(
                            Message("file_progress", 
                                    (float(i)/n, "Reading " + base)
                            )
                    )
                    try:
                        joint_dicts[base] = read(f)
                    except ExpatError, e:
                        tkMessageBox.showwarning(
                                "XML parsing failed",
                                "%s: %s" % (f.name, str(e))
                        )
                        continue
                    self.queue.put(Message("filelist_add", base))
                self.queue.put(
                        Message("file_progress", (1, "Done reading files"))
                )


class AsyncCalcLines(threading.Thread):
    """ 
    A thread class for calculating graphs. 
    
    subplot is the matplotlib axes to plot in.

    dicts is a tuple of dicts of the form returned by read(file)

    graph is the type of graphs to generate.

    queue will get messages added to it. The messages are
        - lines_clear  to alert the UI that the current graphs should be
          cleared
        - textline_add  A textmessage that should be displayed next to the
          graph.
        - line_add  A graph line has been added.
        - adjust_axis  All done, adjust the axes to show all graphs.
    """
    def __init__(self, subplot, dicts, graph, queue):
        threading.Thread.__init__(self)
        self.subplot = subplot
        self.dicts = dicts
        self.graph = graph
        self.queue = queue
    def run(self):
        self.queue.put(Message('lines_clear', None))
        for name, line in make_plot_lines(self.subplot, self.dicts, 
                                          self.graph):
            if line is None:
                self.queue.put(Message('textline_add', name))
            else:
                self.queue.put(Message('line_add', (name, line)))
        self.queue.put(Message('adjust_axis', None))

class Toolbar(NavigationToolbar2TkAgg):
    """
    A customized version of matplotlib's NavigationToolbar2TkAgg.

    The main difference is that some buttons are removed, and the Home
    button has been changed slightly.
    """
    def __init__(self, canvas, window):
        NavigationToolbar2TkAgg.__init__(self, canvas, window)
    def _init_toolbar(self):
        xmin, xmax = self.canvas.figure.bbox.intervalx
        height, width = 50, xmax-xmin
        Tk.Frame.__init__(self, master=self.window,
                          width=width, height=height,
                          borderwidth=2)

        self.update()  # Make axes menu

        self.bHome = self._Button( text="Home", file="home.ppm",
                                   command=self.home)

        self.bPan = self._Button( text="Pan", file="move.ppm",
                                  command = self.pan)

        self.bZoom = self._Button( text="Zoom",
                                   file="zoom_to_rect.ppm",
                                   command = self.zoom)

        self.bsave = self._Button( text="Save", file="filesave.ppm",
                                   command = self.save_figure)
        self.message = Tk.StringVar(master=self)
        self._message_label = Tk.Label(master=self, textvariable=self.message)
        self._message_label.pack(side=Tk.RIGHT)
        self.pack(side=Tk.BOTTOM, fill=Tk.X)

    def set_homeview(self, minx, maxx, miny, maxy):
        """ set the dimensions that should be used when the home button is
        pressed."""
        self._minx = minx
        self._maxx = maxx
        self._miny = miny
        self._maxy = maxy

    def get_homeview(self):
        """ get the current dimensions for the home view. """
        return self._minx, self._maxx, self._miny, self._maxy

    def home(self, *args):
        """restore original view"""
        NavigationToolbar2TkAgg.home(self, *args)
        for a in self.canvas.figure.get_axes():
            a.set_xlim(self._minx, self._maxx)
            a.set_ylim(self._miny, self._maxy)
        self._update_view()
        


class ColorPicker(object):
    """ A small class to keep track of which graph colors are used, and
    to assign the best fit."""
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    def __init__(self, lines):
        self.lines = lines
        self.in_use = []
    def next_color(self):
        """
        Returns the next available color, or, if all colors are used,
        returns a least used color.
        """
        for c in self.colors:
            if c not in self.in_use:
                self.in_use.append(c)
                return c
        last = self.in_use[-1]
        c = self.colors[ (self.colors.index(last) + 1) % len(self.colors)]
        self.in_use.append(c)
        return c
    def uncolor(self, line):
        """ Tells the ColorPicker that the given line is no longer in use
        (visible).
        """
        self.in_use.reverse()
        self.in_use.remove( line.get_color() )
        self.in_use.reverse()


class ProgressMeter(Tk.Frame):
    """ Custom progress meter widget. """
    def __init__(self, master, width=200, height=20, fillcolor='light blue', 
                 *args, **kwargs):
        Tk.Frame.__init__(self, master, width=width, height=height, *args,
                          **kwargs)
        self.canvas = Tk.Canvas(self, width=width, height=height)
        self.canvas.pack(fill='both', expand=1)
        self.width = self.canvas.winfo_reqwidth()
        self.height = self.canvas.winfo_reqheight()
        self.rect = self.canvas.create_rectangle(0, 0, 0, self.height, fill=fillcolor)
        self.text = self.canvas.create_text(self.width/2, self.height/2)
        self.value = 0.0
    def set(self, value, text=None):
        """ Set the new percentage (as a float between 0 and 1), and an
        optional text. """
        if value < 0.0: value = 0.0
        elif value > 1.0: value = 1.0
        self.value = value
        self.canvas.coords(self.rect, 0, 0, self.width * value, self.height)
        self.canvas.itemconfig(self.text, text=text or "%.f%%" % (value*100,))
        self.canvas.update_idletasks()

        
class MainWindow(object):
    """
    Class to keep track of all the Tkinter widgets
    """
    def __init__(self):
        self.root = Tk.Tk()
        self.root.title("MoGrapher")
        self.root.protocol("WM_DELETE_WINDOW", self.destroy)
        self.root.createcommand("exit",self.destroy)
        self.canvas = None
        self.subplot = None
        self.figure = None
        self.sb_text1 = None
        self.sb_text2 = None
        self.sb_text3 = None
        self.toolbar = None
        self.legend = None
        self.legend_font = FontProperties(size='x-small')
        self.gt_file = Tk.StringVar()
        self.filelist = []
        self.candidate = Tk.StringVar(value="Select a candidate")
        self.xmin = Tk.DoubleVar()
        self.xmax = Tk.DoubleVar()
        self.ymin = Tk.DoubleVar()
        self.ymax = Tk.DoubleVar()
        self.bplot={}

        self.queue = Queue.Queue()

        self.graphtype = Tk.StringVar()
        self.lines = {}
        self.cp = ColorPicker(self.lines)
        self.buttonstates = {}
        self.buttons = {}

        self.xscale = Tk.DoubleVar(value=1.0)
        self.yscale = Tk.DoubleVar(value=1.0)

        # Create the UI components
        self.top = self._top_frame()
        self.middle = self._middle_frame()
        self.bottom = self._bottom_frame()
        self.right = self._right_frame()
        self._menu()

        self.top.grid(row=0, sticky='w')
        self.middle.grid(row=1, sticky='w')
        self.bottom.grid(row=2, sticky='we')
        self.right.grid(row=0, column=1, rowspan=3)

        self.filereader = FileReaderThread(self.queue)
        self.filereader.start()

        self.update()

    def destroy(self):
        """
        Call this when the application exits to ensure that all threads
        gets stopped.
        """
        print "Quitting."
        self.filereader.done()
        self.root.quit()
        
    def update(self):
        """
        update runs itself at regular intervals. It updates the user
        interface based on queued messages.
        """
        try:
            while True:
                if self.gt_file.get() and self.candidate.get() != "Select a candidate":
                    self.change_button.config(state=Tk.NORMAL)
                else:
                    self.change_button.config(state=Tk.DISABLED)

                msg = self.queue.get_nowait()
                if msg.type == 'filelist_add':
                    self.filelist.append(msg.value)
                    self.candidatemenu['menu'].add_command(
                            label=msg.value,
                            command=Tk._setit(self.candidate, msg.value),
                    )
                    self.candidatemenu['state'] = "normal"
                elif msg.type == 'gt_file_add':
                    self.gt_file.set(msg.value)
                elif msg.type == 'lines_clear':
                    self.cb = ColorPicker(self.lines)
                    self.clear_lines()
                elif msg.type == 'line_add':
                    name, lines = msg.value
                    self.add_line(name, lines)
                elif msg.type == 'adjust_axis':
                    a, b = self.subplot.get_xlim()
                    c, d = self.subplot.get_ylim()
                    self.toolbar.set_homeview(a,b,c,d)
                elif msg.type == 'file_progress':
                    self.progress.set(msg.value[0], msg.value[1])
                elif msg.type == 'textline_add':
                    if msg.value.startswith("Mean"):
                        #self.sb_text2.set_text(msg.value)
                        print msg.value
                    elif msg.value.startswith("St"):
                        #self.sb_text3.set_text(msg.value)
                        print msg.value
                    self.canvas.draw()

        except Queue.Empty:
            pass
        self.root.after(100, self.update)
        
    def set_gt_file(self):
        """ Opens a file dialog asking for the ground-truth file and
        processes it."""
        file = tkFileDialog.askopenfile(filetypes=[('XML','*.xml')],
                                        parent=self.root,
                                        title="Select ground truth file")
        if file:
            self.filereader.process_files(file)
    def add_file(self):
        """ Opens a file dialog asking for candidate files and
        processes them."""
        files = tkFileDialog.askopenfiles(filetypes=[('XML','*.xml')],
                                          parent=self.root,
                                          title="Select datafile(s)")
        if files:
            self.filereader.process_files(files)

    def clear_candidates(self):
        """ Clear the candidates from the dropdown menu, and disable the
        dropdown widget. """

        # The Tkinter OptionsMenu isn't really intended to be used dynamically
        # so we have to hack a little bit around it.
        self.candidatemenu['menu'].delete(0, Tk.END)
        self.candidate.set("Select a candidate")
        self.candidatemenu['state'] = Tk.DISABLED
        self.filelist = []
    
    def clear_lines(self):
        """ Clear the canvas. """
        #self.sb_text2.set_text("Mean:")
        #self.sb_text3.set_text("Standard deviation:")
        for name in self.buttons.keys():
            self.buttons[name].grid_forget()
            del self.buttons[name]
        for name in self.lines.keys():
            for line in self.lines[name]:
                line.remove()
            del self.lines[name]
        self.lines.clear()
        for name in self.buttonstates.keys():
            del self.buttonstates[name]
        if self.legend:
            # legend.remove() is not implemented in matplotlib (yet),
            # so we have to hack around that.
            self.legend.set_visible(False)
            self.subplot.legend_ = None
        for name in self.bplot.keys():
            for line in self.bplot[name]:
                line.remove()
            del self.bplot[name]
        self.canvas.draw()

    def add_line(self, name, lines):
        """
        Add lines to a plot, and assign a checkbox for it in the right
        part of the user interface. 
        
        name is the name for the checkbox,
        lines is a list of matplotlib lines.
        """
        self.lines[name] = lines
        self.buttonstates[name] = Tk.BooleanVar(value=False)
        def button_press(name=name):
            color = None
            if self.buttonstates[name].get():
                color = self.cp.next_color()
            else:
                self.cp.uncolor(self.lines[name][0])
            for line in self.lines[name]:
                if color:
                    line.set_color(color)
                line.set_visible(self.buttonstates[name].get())
            linelist=[]
            labellist=[]
            for key in self.buttonstates.keys():
                if self.buttonstates[key].get():
                    labellist.append(key)
                    linelist.append(self.lines[key][0])
            if linelist:
                #self.legend = self.subplot.legend(linelist, labellist, 
                #                                  "upper left",
                #                                  bbox_to_anchor=(1., 1.015),
                #                                  prop=self.legend_font)
                pass
            elif self.legend:
                # legend.remove() is not implemented in matplotlib (yet),
                # so we have to hack around that.
                self.legend.set_visible(False)
                self.subplot.legend_ = None
            self.canvas.draw()
        button = Tk.Checkbutton(self.right, text=name, 
                                variable=self.buttonstates[name],
                                command=button_press)
        n = len(self.lines) - 1
        button.grid(row=n/2, column=n%2, sticky='w')
        self.buttons[name] = button
            
    def change_graph(self):
        """ Generate graphs based on the currently selected ground truth
        file, candidate file and graph type.
        """
        ground_truth = os.path.basename(self.gt_file.get())
        candidate = os.path.basename(self.candidate.get())
        if not joint_dicts.has_key(ground_truth) or \
           not joint_dicts.has_key(candidate):
            tkMessageBox.showwarning(
                    "Generating graphs",
                    "No ground truth or candidate file selected"
            )
            return

        dicts = [
            joint_dicts[ground_truth],
            joint_dicts[candidate],
        ]
        
        graph = self.graphtype.get()
        #self.subplot.clear()
        if graph==graphtypes[4]: # boxplot
          self.clear_lines()
          bplot=createBoxPlot(self.subplot,dicts)
          self.bplot=bplot
          self.canvas.draw()
        else:
          AsyncCalcLines(self.subplot, dicts, graph, self.queue).start()
    
    def _quit(self):
        self.destroy()

    def _menu(self):
        menubar = Tk.Menu(self.root)

        filemenu = Tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Open Ground truth file...", command=self.set_gt_file)
        filemenu.add_command(label="Open Candidate files...", command=self.add_file)
        filemenu.add_command(label="Save as...", command=self.toolbar.save_figure)
        filemenu.add_command(label="Clear candidates", command=self.clear_candidates)
        filemenu.add_separator()
        #filemenu.add_command(label="Quit", command=sys.exit)
        filemenu.add_command(label="Quit", command=self.destroy)
        menubar.add_cascade(label="File", menu=filemenu)

        graphmenu = Tk.Menu(menubar, tearoff=0)
        graphmenu.add_command(label="Clear", command=self.clear_lines)
        menubar.add_cascade(label="Graph", menu=graphmenu)

        self.root.config(menu=menubar)


    def _top_frame(self):
        top_frame = Tk.Frame(self.root)

        groundtruth = Tk.Entry(top_frame, width=20, textvariable=self.gt_file,
                               state=Tk.DISABLED)
        gt_button = Tk.Button(top_frame, text="...", command=self.set_gt_file)

        # A bit hackish. The Tkinter OptionMenu is not really intended to be
        # used dynamically. The constructor requires that we provide a list
        # of all items as well as which one should be currently selected. We 
        # provide a dummy value, then delete it by altering the menu component
        # directly.
        self.candidatemenu = Tk.OptionMenu(top_frame, self.candidate, '')
        self.candidatemenu['menu'].delete(0)
        self.candidatemenu['state'] = Tk.DISABLED
        c_button = Tk.Button(top_frame, text="...", command=self.add_file)

        graphoptions = (graphtypes)
        self.graphtype.set(graphoptions[0])
        graphtypemenu = Tk.OptionMenu(top_frame, self.graphtype, *graphoptions)
        self.progress = ProgressMeter(top_frame)
        self.change_button = Tk.Button(top_frame, text="Generate", command=self.change_graph)

        Tk.Label(top_frame, text="Select ground truth file").grid(row=0, sticky='w')
        groundtruth.grid(row=1, column=0)
        gt_button.grid(row=1, column=1)

        Tk.Label(top_frame, text="Select candidate file").grid(row=2, sticky='w')
        self.candidatemenu.grid(row=3, column=0)
        c_button.grid(row=3, column=1)

        Tk.Label(top_frame, text="Select graphtype").grid(row=0, column=4, sticky='w')
        graphtypemenu.grid(row=1, column=4, sticky='e')
        self.progress.grid(row=3, column=4, columnspan=2)
        self.change_button.grid(row=1, column=5)

        return top_frame

    def _middle_frame(self):
        middle_frame = Tk.Frame(self.root)


        # The graph-canvas
        self.figure = Figure(figsize=(12,7), dpi=100)
        self.subplot = self.figure.add_subplot(1,1,1) #, xlabel='Frame', ylabel='Error (mm)')
        self.subplot.get_xaxis().set_tick_params(labelsize=20,pad=10)
        self.subplot.set_xlabel('Frame',labelpad=40,size=20)
        self.subplot.set_ylabel('Error (mm)',size=20)
        self.subplot.get_yaxis().set_tick_params(labelsize=20,pad=10)
        #self.subplot.set_position((0.11, 0.125, 0.65, 0.8))
        self.subplot.set_position((0.1, 0.25, 0.85, 0.7))

        #drawingArea = DrawingArea(110,25,0,0)
        #ab = AnchoredOffsetbox(loc=2,
        #                       child=drawingArea, 
        #                       pad=0.5, frameon=True, 
        #                       bbox_to_anchor=(1.01, .135),
        #                       bbox_transform=self.subplot.transAxes,
        #                       borderpad=0.)
        #self.subplot.add_artist(ab)
        #self.sb_text1 = TextArea("Statistics", textprops=dict(size=8))
        #self.sb_text2 = TextArea("Average:", textprops=dict(size=8))
        #self.sb_text3 = TextArea("Standard deviation:", textprops=dict(size=8))
        #self.box = VPacker(children=[self.sb_text1, self.sb_text2, self.sb_text3], 
        #              align="left", pad=0, sep=5)
        #self.anchored_box = AnchoredOffsetbox(loc=2,
        #                                 child=self.box,
        #                                 pad=0.5, frameon=False, 
        #                                 bbox_to_anchor=(1.01, .145),
        #                                 bbox_transform=self.subplot.transAxes,
        #                                 borderpad=0.)
        #self.subplot.add_artist(self.anchored_box)

        self.canvas = FigureCanvasTkAgg(self.figure, master=middle_frame)
        self.canvas.show()

        def xlim_changed(subplot):
            min, max = subplot.get_xlim()
            self.xmin.set(min)
            self.xmax.set(max)
        def ylim_changed(subplot):
            min, max = subplot.get_ylim()
            self.ymin.set(min)
            self.ymax.set(max)
        self.subplot.callbacks.connect('xlim_changed', xlim_changed)
        self.subplot.callbacks.connect('ylim_changed', ylim_changed)

        # slider for Y-axis
        slider = Tk.Scale(middle_frame, length=200, from_=6.0, to=0.1, 
                          resolution=0.1, orient=Tk.VERTICAL, 
                          variable=self.yscale)
        def changescale(*args):
            _, _, min, max = self.toolbar.get_homeview()
            c = max - min
            l = (c * self.yscale.get() - c) / 2.
            self.subplot.set_ylim(min - l, max + l)
            self.canvas.draw()

        self.yscale.trace('w', changescale)

        self.canvas.get_tk_widget().grid(row=0, column=0)
        slider.grid(row=0, column=1)

        return middle_frame

    def _bottom_frame(self):
        bottom_frame = Tk.Frame(self.root)

        toolbar_and_slider = Tk.Frame(bottom_frame)

        # slider for X-axis 
        slider = Tk.Scale(toolbar_and_slider, length=300, from_=0.1, to=6.0, 
                          resolution=0.1, orient=Tk.HORIZONTAL,
                          variable=self.xscale)
        def changescale(*args):
            min, max, _, _ = self.toolbar.get_homeview()
            c = max - min
            l = (c * self.xscale.get() - c) / 2.
            self.subplot.set_xlim(min - l, max + l)
            self.canvas.draw()

        self.xscale.trace('w', changescale)

        self.toolbar = Toolbar(self.canvas, toolbar_and_slider)

        slider.grid(row=0, sticky='w')
        self.toolbar.grid(row=1, sticky='w')

        axis_box = Tk.Frame(bottom_frame, width=300)

        xlabel = Tk.Label(axis_box, text="X-axis")
        ylabel = Tk.Label(axis_box, text="Y-axis")

        def callback(event, *args):
            if event.widget in (self.xminbox, self.xmaxbox):
                try:
                    min = float(self.xmin.get())
                    max = float(self.xmax.get())
                    if (min, max) != (self.subplot.get_xlim()):
                        self.subplot.set_xlim( min, max )
                except ValueError:
                    return
            elif event.widget in (self.yminbox, self.ymaxbox):
                try:
                    min = float(self.ymin.get())
                    max = float(self.ymax.get())
                    if (min, max) != (self.subplot.get_ylim()):
                        self.subplot.set_ylim( min, max )
                except ValueError:
                    return
            self.canvas.draw()

        min, max = self.subplot.get_xlim()
        self.xmin.set(min)
        self.xmax.set(max)

        min, max = self.subplot.get_ylim()
        self.ymin.set(min)
        self.ymax.set(max)

        def validfloat(n="0"):
            try:
                float(n)
                return True
            except ValueError:
                return False
        
        self.xminbox = Tk.Entry(axis_box, width=5, textvariable=self.xmin,
                                validate="focusout",
                                validatecommand=validfloat)
        self.xmaxbox = Tk.Entry(axis_box, width=5, textvariable=self.xmax,
                                validate="focusout",
                                validatecommand=validfloat)
        self.yminbox = Tk.Entry(axis_box, width=5, textvariable=self.ymin,
                                validate="focusout",
                                validatecommand=validfloat)
        self.ymaxbox = Tk.Entry(axis_box, width=5, textvariable=self.ymax,
                                validate="focusout", validatecommand=validfloat)

        # Only use the values when the textbox loses focus or the return/enter
        # key is pressed.
        self.xminbox.bind("<FocusOut>", callback)
        self.xmaxbox.bind("<FocusOut>", callback)
        self.yminbox.bind("<FocusOut>", callback)
        self.ymaxbox.bind("<FocusOut>", callback)
        self.xminbox.bind("<Return>", callback)
        self.xmaxbox.bind("<Return>", callback)
        self.yminbox.bind("<Return>", callback)
        self.ymaxbox.bind("<Return>", callback)

        xlabel.grid(row=0, column=0)
        self.xminbox.grid(row=0, column=1)
        self.xmaxbox.grid(row=0, column=2)

        ylabel.grid(row=1, column=0)
        self.yminbox.grid(row=1, column=1)
        self.ymaxbox.grid(row=1, column=2)

        toolbar_and_slider.pack(side=Tk.LEFT,expand=True,anchor='w')
        axis_box.pack(anchor='e', side=Tk.RIGHT)

        return bottom_frame

    def _right_frame(self):
        joint_button_frame = Tk.Frame(self.root, width=200)
        
        return joint_button_frame

def sigint(*args):
    """ trap for SIGINT. Exits with exit status 1. """
    sys.exit(1)

def print_help(to=sys.stdout):
    """ Print command-line help. """
    print >> to, """\
Usage: %s [-h|--help] [gtfile [candidatefile...]]
    
    Start mographer, optionally loading gtfile as ground truth file
    and zero or more candidatefiles.

    -h, --help   Print this help and exit.
    """ % (sys.argv[0],)


if __name__ == '__main__':
    if '--help' in sys.argv[1:] or '-h' in sys.argv[1:]:
        print_help()
        sys.exit(0)
    gt_file = None
    cand_files = []
    try:
        if len(sys.argv) > 1:
            gt_file = open(sys.argv[1])
        if len(sys.argv) > 2:
            cand_files = [open(f) for f in sys.argv[2:]]
    except IOError, e:
        print >> sys.stderr, "%s: %s\n" % (sys.argv[0], e)
        print_help(sys.stderr)
        sys.exit(2)
    win = MainWindow()
    signal.signal(signal.SIGINT, sigint)
    if gt_file:
        win.filereader.process_files(gt_file)
    if cand_files:
        win.filereader.process_files(cand_files)
    
    Tk.mainloop()
