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
Name: 'PosExport (.xml) ...'
Blender: 249
Group: 'Export'
Tooltip: 'Export joint positions to xml'
"""

import Blender
import bpy
from xml.dom.minidom import Document
import math

joints = {
(None, 'root'):'root',
('root', 'lowerback'):'lowerback',
('root', 'lhipjoint'):'lhipjoint',
('root', 'rhipjoint'):'rhipjoint',
('root','rfemur'):'rhip',
('root','lfemur'):'lhip',
('rhipjoint', 'rfemur'):'rhip',
('lhipjoint', 'lfemur'):'lhip',
('rfemur', 'rtibia'):'rkne',
('lfemur', 'ltibia'):'lkne',
('rtibia', 'rfoot'):'rank',
('ltibia', 'lfoot'):'lank',
('rfoot', 'rtoes'):'rtoes',
('lfoot', 'ltoes'):'ltoes',
('rtoes', None):'rtoetip',
('ltoes', None):'ltoetip',
('lowerback','upperback'):'upperback',
('upperback', 'thorax'):'thorax',
('thorax', 'rclavicle'):'clav',
('thorax', 'lclavicle'):'clav',
('thorax', 'lowerneck'):'clav',
('rclavicle', 'rhumerus'):'rsho',
('rhumerus', 'rradius'):'relb',
('rradius', 'rwrist'):'rwrist',
('rwrist', 'rhand'):'rhand',
('rwrist', 'rthumb'):'rhand',
('rthumb', None): 'rthumbtip',
('rhand', 'rfingers'):'rfingers',
('rfingers', None):'rfingertip',
('lclavicle', 'lhumerus'):'lsho',
('lhumerus', 'lradius'):'lelb',
('lradius', 'lwrist'):'lwrist',
('lwrist', 'lhand'):'lhand',
('lwrist', 'lthumb'):'lhand',
('lthumb', None): 'lthumbtip',
('lhand', 'lfingers'):'lfingers',
('lfingers', None):'lfingertip',
('lowerneck', 'upperneck'):'neck',
('upperneck', 'head'):'head',
('head', None):'headtop',

#Ignored joints
('rwrist', 'rhandignored'):'rfingers',
('rhandignored', None):'rfingertip',
('lwrist', 'lhandignored'):'lfingers',
('lhandignored', None):'lfingertip',
('rfoot', 'rtoesignored'):'rtoes',
('rtoesignored', None):'rtoetip',
('lfoot', 'ltoesignored'):'ltoes',
('ltoesignored', None):'ltoetip',
}

jointsignore = [
'lhipjoint', 'rhipjoint', 'headtop', 'root'
]

def write(filename, scale=1):
  """
  Writes the positions of all joints for all frames to the given filename.
  And optional scale value can be given.
  """
  scene = Blender.Scene.GetCurrent()
  armatures = [obj for obj in scene.objects if obj.type == 'Armature']
  
  for arm_obj in armatures:
      thisFilename = filename + arm_obj.getName()
      if not thisFilename.endswith('.xml'):
        thisFilename+='.xml'
      out=file(thisFilename, "w")
      doc=Document()

      # create root element
      xml=doc.createElement("xml")
      doc.appendChild(xml)
    
      rest_bones = arm_obj.getData().bones
      pose_bones = arm_obj.getPose().bones
      context = scene.getRenderingContext()
      action = arm_obj.getAction()
      
      numFrames = int(max(action.getFrameNumbers()) * (float(context.newMapValue()) / float(context.oldMapValue())))

      for frame_number in range(1,numFrames+1):
          frame_element = doc.createElement("frame")
          frame_element.setAttribute("id", str(frame_number))
          xml.appendChild(frame_element)
          context.currentFrame(frame_number)
  
          boneSet = set()
          for bone in pose_bones.values():      
              parent = rest_bones[bone.name].parent
              if parent:
                  parent_name = parent.name
              else:
                  parent_name = None

              joint_name = joints[parent_name, bone.name]

              if joint_name in jointsignore:
                continue

              children = rest_bones[bone.name].children
          
              head = bone.head * scale
      
              # because Main.py rotates the pose (asf amc has Y up while Blender has Z up), 
              # the global coordinates come from blender in x, -z, y
              boneSet.add((joint_name, head.x, -head.z, head.y))

              if not children:
                  tail = bone.tail * scale
              
                  joint_name = joints[bone.name, None]
                  boneSet.add((joint_name, tail.x, -tail.z, tail.y))
    
          for name, x,y,z in boneSet:
              joint_element = doc.createElement("joint")
              joint_element.setAttribute("name", name)
              joint_element.setAttribute("ignore", 'false')
              frame_element.appendChild(joint_element)
              pos_element = doc.createElement("pos")
              pos_element.setAttribute("x", str(x))
              pos_element.setAttribute("y", str(y))
              pos_element.setAttribute("z", str(z))
              joint_element.appendChild(pos_element)

      context.currentFrame(1)
      out.write(doc.toprettyxml(indent="  "))
      out.close()

Blender.Window.FileSelector(write, "Export joint poisitions XML")

