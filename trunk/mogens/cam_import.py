#!BPY
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

"""
Name: 'Camera Import (.xml) ...'
Blender: 249
Group: 'Import'
Tooltip: 'Import cameras from xml'
"""
import Blender
from Blender import Camera
import bpy
import xml.dom.minidom
from xml.dom.minidom import Document
import math

def read(filename):
    """ Reads in a camera setup from the given XML file. 
    
    This is the reverse of cam_export.write(filename).
    
    the xml is of the form:
    <calibration>
        <camera id="[camera name]">
            <fl x="[X-value]" y="[Y-value]" />
            <pp x="[X-value]" y="[Y-value]" />
            <pos x="[X-value]" y="[Y-value]" z="[Z-value]"/>
            <rot x="[X-value]" y="[Y-value]" z="[Z-value]"/>
        </camera>
        <camera ...
    </calibration>
    """
    doc = xml.dom.minidom.parse(filename)
    
    cams = doc.getElementsByTagName("camera")
    scene = Blender.Scene.GetCurrent()
    
    for camera in cams:
        newCam = Camera.New('persp')
        newCamObj = scene.objects.new(newCam)
        pos = camera.getElementsByTagName("pos")[0]
        
        newCamObj.LocX = float(pos.getAttribute("x"))
        newCamObj.LocY = float(pos.getAttribute("y"))
        newCamObj.LocZ = float(pos.getAttribute("z"))
        
        rot = camera.getElementsByTagName("rot")[0]
        
        newCamObj.RotX = float(rot.getAttribute("x"))
        newCamObj.RotY = float(rot.getAttribute("y"))
        newCamObj.RotZ = float(rot.getAttribute("z"))

Blender.Window.FileSelector(read, "Import camera XML")
