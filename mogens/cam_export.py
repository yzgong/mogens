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
Name: 'CamExport (.xml) ...'
Blender: 244
Group: 'Export'
Tooltip: 'Export cameras to xml'
"""
import Blender
import bpy
from xml.dom.minidom import Document
import math

def write(filename):
  """
  Writes the data of the current scene's cameras an xml file given by
  filename.

  A .xml extension is added to the filename if it doesn't already have one.

  The resulting file can be read back in with cam_import.read(filename).
  
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
  if not filename.endswith('.xml'):
    filename+='.xml'
  out=file(filename, "w")
  doc=Document()
  
  scene = Blender.Scene.GetCurrent()

  # create root element
  xml=doc.createElement("calibration")
  doc.appendChild(xml)
    
  cameras = [cam for cam in scene.objects if cam.type == 'Camera']

  print cameras[0].rot
  print cameras[0].mat

  for cam in cameras:
      # create main camera element
      camera=doc.createElement("camera")
      camera.setAttribute("id",cam.name)
      xml.appendChild(camera)

      con=scene.getRenderingContext()

      # create focal length element
      fl=con.imageSizeX()/(2*math.tan(math.radians(cam.getData().angle)/2))
      focalLength=doc.createElement("fl")
      focalLength.setAttribute("x",str(fl))
      focalLength.setAttribute("y",str(fl))
      camera.appendChild(focalLength)

      # create principal point element
      pp=doc.createElement("pp")
      pp.setAttribute("x",str(con.imageSizeX()/2))
      pp.setAttribute("y",str(con.imageSizeY()/2))
      camera.appendChild(pp)

      # create position element
      pos=doc.createElement("pos")

      pos.setAttribute("x",str(cam.LocX))
      pos.setAttribute("y",str(cam.LocY))
      pos.setAttribute("z",str(cam.LocZ))
      camera.appendChild(pos)

      # create rotation element
      rot=doc.createElement("rot")
      rot.setAttribute("x",str(cam.RotX))
      rot.setAttribute("y",str(cam.RotY))
      rot.setAttribute("z",str(cam.RotZ))
      camera.appendChild(rot)

  out.write(doc.toprettyxml(indent="  "))
  out.close()

Blender.Window.FileSelector(write, "Export camera XML")

