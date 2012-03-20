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
Name: 'Motion Capture (.asf, .amc)...'
Blender: 242
Group: 'Import'
Tip: 'Import a (.asf) skeleton and (.amc) motion capture file'

__author__ = "Dag Stuan and Geir Hauge"
"""

import Blender
Matrix = Blender.Mathutils.Matrix
Vector = Blender.Mathutils.Vector
Euler= Blender.Mathutils.Euler
TranslationMatrix = Blender.Mathutils.TranslationMatrix
RotationMatrix = Blender.Mathutils.RotationMatrix
Armature = Blender.Armature
import math
from math import *
from numpy import matrix, cos, sin, array, asmatrix, dot

from mogens.asf import ASF
from mogens.asf import Bone
from mogens.amc import AMC

asf = ASF()
asf2 = ASF()
amc = AMC(asf)

ASFString = ""
AMCString = ""

ignoreOuterJoints=False

animation_origo = Vector(0.,0.,0.)

# Base hierarchy
armature_order = ['root', 'rhipjoint', 'lhipjoint',
                  'lowerback',
                  'upperback',
                  'thorax',
                  'rclavicle', 'lclavicle', 'lowerneck',
                  'upperneck', 'head',
                  'rhumerus', 'rradius', 'rwrist',
                  'lhumerus', 'lradius', 'lwrist',
                  'lfemur', 'ltibia', 'lfoot',
                  'rfemur', 'rtibia', 'rfoot',
                  ]
                  
# Base armature which is created.
base_armature = {
'root': [Vector(0.000, 0.000, 0.000), Vector(0.000, 0.001, 0.000), None],
'rhipjoint': [Vector(0.000, 0.001, 0.000), Vector(-1.637, -1.828, 0.763), 'root'],
'lhipjoint': [Vector(0.000, 0.001, 0.000), Vector(1.666, -1.828, 0.763), 'root'],
'lowerback': [Vector(0.000, 0.001, 0.000), Vector(0.003, 2.302, -0.130), 'root'],
'upperback': [Vector(0.003, 2.302, -0.130), Vector(0.032, 4.609, -0.122), 'lowerback'],
'thorax': [Vector(0.032, 4.609, -0.122), Vector(0.064, 6.918, -0.054), 'upperback'],
'rclavicle': [Vector(0.064, 6.918, -0.054), Vector(-3.346, 7.562, 0.086), 'thorax'],
'lclavicle': [Vector(0.064, 6.918, -0.054), Vector(3.345, 7.613, -0.059), 'thorax'],
'lowerneck': [Vector(0.064, 6.918, -0.054), Vector(0.193, 8.601, -0.047), 'thorax'],
'upperneck': [Vector(0.193, 8.601, -0.047), Vector(-0.051, 10.273, -0.207), 'lowerneck'],
'head': [Vector(-0.051, 10.273, -0.207), Vector(-0.143, 11.982, -0.271), 'upperneck'],
'rhumerus': [Vector(-3.346, 7.562, 0.086), Vector(-9.228, 7.562, 0.086), 'rclavicle'],
'rradius': [Vector(-9.228, 7.562, 0.086), Vector(-12.509, 7.562, 0.086), 'rhumerus'],
'rwrist': [Vector(-12.509, 7.562, 0.086), Vector(-14.150, 7.562, 0.086), 'rradius'],
'lhumerus': [Vector(3.345, 7.613, -0.059), Vector(9.052, 7.613, -0.059), 'lclavicle'],
'lradius': [Vector(9.052, 7.613, -0.059), Vector(12.384, 7.613, -0.059), 'lhumerus'],
'lwrist': [Vector(12.384, 7.613, -0.059), Vector(14.050, 7.613, -0.059), 'lradius'],
'lfemur': [Vector(1.666, -1.828, 0.763), Vector(4.083, -8.470, 0.763), 'lhipjoint'],
'ltibia': [Vector(4.083, -8.470, 0.763), Vector(6.799, -15.933, 0.763), 'lfemur'],
'lfoot': [Vector(6.799, -15.933, 0.763), Vector(6.879, -16.153, 2.588), 'ltibia'],
'rfemur': [Vector(-1.637, -1.828, 0.763), Vector(-4.043, -8.437, 0.763), 'rhipjoint'],
'rtibia': [Vector(-4.043, -8.437, 0.763), Vector(-6.776, -15.946, 0.763), 'rfemur'],
'rfoot': [Vector(-6.776, -15.946, 0.763), Vector(-6.931, -16.374, 2.699), 'rtibia'],
}

outer_joints_order = {'normal':
                      ['rthumb', 'rhand', 'rfingers',
                      'lthumb', 'lhand', 'lfingers',
                      'ltoes', 'rtoes'],
                      'ignore':
                      ['rhandignored', 'lhandignored', 'ltoesignored', 'rtoesignored'],
                      }

# The outer joints, this is saved in a dict because the number of bones varies
# depending on if the outer joints are ignored or not.
outer_joints = {
'normal': {
    'rthumb': [Vector(-14.150, 7.562, 0.086), Vector(-14.857, 7.562, 0.793), 'rwrist'],
    'rhand': [Vector(-14.150, 7.562, 0.086), Vector(-15.013, 7.562, 0.086), 'rwrist'],
    'rfingers': [Vector(-15.013, 7.562, 0.086), Vector(-15.709, 7.562, 0.086), 'rhand'],
    'lthumb': [Vector(14.050, 7.613, -0.059), Vector(14.699, 7.613, 0.590), 'lwrist'],
    'lhand': [Vector(14.050, 7.613, -0.059), Vector(14.843, 7.613, -0.059), 'lwrist'],
    'lfingers': [Vector(14.843, 7.613, -0.059), Vector(15.482, 7.613, -0.059), 'lhand'],
    'ltoes': [Vector(6.879, -16.153, 2.588), Vector(6.879, -16.153, 3.523), 'lfoot'],
    'rtoes': [Vector(-6.931, -16.374, 2.699), Vector(-6.931, -16.374, 3.700), 'rfoot']
    },
'ignore': {
    'rhandignored': [Vector(-14.150, 7.562, 0.086), Vector(-15.684, 7.723, 0.113), 'rwrist'],
    'lhandignored': [Vector(14.050, 7.613, -0.059), Vector(15.449, 7.839, 0.055), 'lwrist'],
    'ltoesignored': [Vector(6.879, -16.153, 2.588), Vector(6.662, -16.153, 4.333), 'lfoot'],
    'rtoesignored': [Vector(-6.931, -16.374, 2.699), Vector(-6.631, -16.159, 4.361), 'rfoot']
    }
}

def get_rotation_matrix(bone_name, frame_vals):
    """
    Returns a rotation matrix for the given bone and frame, rotated around
    the axis given by the asf object, and translated by the offset given by
    the asf object.
    """
    global asf, amc

    if bone_name == 'root':
        asfbone = asf.root
    else:
        asfbone = asf.bones[bone_name]
    
    rvals = (radians(frame_vals.x), radians(frame_vals.y), radians(frame_vals.z))
    
    cx, cy, cz = cos(rvals)
    sx, sy, sz = sin(rvals)

    trans_matrix = array([
        [ cy*cz,            cy*sz,           -sy,    0.],
        [-cx*sz + sx*sy*cz, cx*cz + sx*sy*sz, sx*cy, 0.],
        [ sx*sz + cx*sy*cz,-sx*cz + cx*sy*sz, cx*cy, 0.], 
        [0.,                0.,               0.,    1.],
    ])

    if not asfbone.parent:
        trans_matrix = dot(dot(asfbone.axis_rot, trans_matrix), asfbone.axis_rot_inv)
    else:
        trans_matrix = dot(dot(dot(asfbone.parent.offset_mat, asfbone.axis_rot), trans_matrix), asfbone.axis_rot_inv)

    return Matrix(*trans_matrix.tolist())

def animate_armature(frame, number, armature, armatureObj, pose_bones, child):
    """ Applies the rotations for all the armature's bones for the given
    frame.

    frame is an AMC.Frame object containing the rotations for each bone in
    a frame.

    number is the frame number

    armature is the Blender Armature data object

    armatureObj is the Blender Armature object

    pose_bones is a dict of all the bones in a Blender Pose.

    child is used for recursion. Start off with the root bone, and it'll
    recurse down to the "leaf" bones.
    """
    global asf, amc
    name = child.name
    pose_bone = pose_bones[name]
    rest_bone = armature.bones[name]
    frame_vals = frame.bones.get(name, Vector())
    
    bone_rotation_matrix = get_rotation_matrix(name, frame_vals)

    pose_bone.quat = (child.bone_rest_matrix * bone_rotation_matrix * child.bone_rest_matrix_inv).toQuat()
    pose_bone.insertKey(armatureObj, number, [Blender.Object.Pose.ROT], True)
    
    for child in child.children:
        if ignoreOuterJoints and child.name in outer_joints['normal']:
            continue
        animate_armature(frame, number, armature, armatureObj, pose_bones, child)
        
def getDistance (first, second):
    """ Returns the euclidian distance between two vectors. """
    locx = second[0] - first[0]
    locy = second[1] - first[1]
    locz = second[2] - first[2]

    distance = math.sqrt((locx)**2 + (locy)**2 + (locz)**2)
    return distance
    
def apply_animation(armatureObj, amcObject, prevAMC=None, frameOffset=0, posOffset=None):
    """ Applies animation from an AMC object to a Blender Armature object.

    armatureObj is the Blender Armature object to apply the animation on.

    amcObject is the AMC object containing the motion data.

    prevAMC is used for motion stitching. If it is not None, the end of
    prevAMC and the start of amcObject will be sticthed together

    frameOffset determines the number of frames to use for interpolating
    when stitching.

    posOffset, if set, will translate the motion from origo given a vector.
    """

    # Apply Animation to armature
    armature = armatureObj.getData()
    
    pose = armatureObj.getPose()
    pose_bones = pose.bones
    
    # Set rest_matrix and rest_matrix_inv for all bones, starting with root
    pose_bone = pose_bones['root']
    rest_bone = armature.bones['root']
    
    if prevAMC:
        posTransMatrix = TranslationMatrix(posOffset)
        
        asf.root.bone_rest_matrix = (rest_bone.matrix['ARMATURESPACE']).rotationPart().resize4x4()
    else:
        asf.root.bone_rest_matrix = rest_bone.matrix['ARMATURESPACE'].rotationPart().resize4x4()
    
    asf.root.bone_rest_matrix_inv = Matrix(asf.root.bone_rest_matrix).invert()
    
    # Finding and saving the rest matrix of each bone. This only need to be done once.
    for bone in armature.bones.values():
        name = bone.name
        if name == 'root':
            continue
        pose_bone = pose_bones[name]
        rest_bone = armature.bones[name]
        asf.bones[name].bone_rest_matrix = rest_bone.matrix['ARMATURESPACE'].rotationPart().resize4x4()
        asf.bones[name].bone_rest_matrix_inv = Matrix(asf.bones[name].bone_rest_matrix).invert()

    # Is the motion a new motion or a continuation?
    if not prevAMC:
        action = Blender.Armature.NLA.NewAction("Action")
        action.setActive(armatureObj)
    else:
        action = armatureObj.getAction()
    
    max_x = -9999
    min_x = 9999
    max_y = -9999
    min_y = 9999
    max_z = -9999
    min_z = 9999
    
    if prevAMC:
        startFrameNumber = max(action.getFrameNumbers()) + frameOffset
    
    # Adding keyframe actions to each of the bones recursively
    for number, frame in sorted(amcObject.frames.items()):
        Blender.Window.WaitCursor(True)
        
        if prevAMC:
            number += startFrameNumber
        
        # First adding for root
        pose_bone = pose_bones['root']
        rest_bone = armature.bones['root']
        
        # Adding root rotation
        bone_rotation_matrix = get_rotation_matrix('root', frame.root_rot)
        pose_bone.quat = (asf.root.bone_rest_matrix * bone_rotation_matrix * asf.root.bone_rest_matrix_inv).toQuat()
        
        # Adding root location
        if prevAMC:
            pose_bone.loc = (\
                           TranslationMatrix(Vector(frame.root_pos.x + posOffset.x, frame.root_pos.y + posOffset.y, frame.root_pos.z + posOffset.z)) *\
                           asf.root.bone_rest_matrix_inv).translationPart()
        else:
            pose_bone.loc= (\
                           TranslationMatrix(Vector(frame.root_pos.x, frame.root_pos.y, frame.root_pos.z)) *\
                           asf.root.bone_rest_matrix_inv).translationPart()
        
        # Is done to figure out the origo of the movement. This is done here
        # rather than in Blender because the renderingcontext in Blender is
        # _really_ slow.
        if pose_bone.loc.x > max_x: max_x = pose_bone.loc.x
        if pose_bone.loc.x < min_x: min_x = pose_bone.loc.x
        if pose_bone.loc.y < max_y: max_y = pose_bone.loc.y
        if pose_bone.loc.y > min_y: min_y = pose_bone.loc.y
        if pose_bone.loc.z > max_z: max_z = pose_bone.loc.z
        if pose_bone.loc.z < min_z: min_z = pose_bone.loc.x
        
        # Adding root keyframe
        pose_bone.insertKey(armatureObj, number, [Blender.Object.Pose.ROT, Blender.Object.Pose.LOC], True)
        
        # Recursively adding animation for the rest of the bones in the armature
        for child in asf.root.children:
            animate_armature(frame, number, armature, armatureObj, pose_bones, child)
            
    global animation_origo
    # Sets the animation origo
    animation_origo = ((Vector(max_x, max_y, max_z) - Vector(min_x, min_y, min_z)) / 2) + Vector(min_x, min_y, min_z)
            
def make_bone(arm, name, head, tail, parent=None):
    """ Creates a bone to the armature given by arm, with the given name,
    head vector, tail vector and name of the parent bone."""
    bone = Blender.Armature.Editbone()
    bone.name = name
    bone.head = head
    bone.tail = tail
    if parent:
        bone.parent = arm.bones[parent]
        bone.options = [Armature.CONNECTED]
    arm.bones[name] = bone
        
def make_armature(name="Unnamed"):
    """ Creates an armature with the given name. The armature is created
    given the global asf object."""
    global asf, amc
    scene = Blender.Scene.GetCurrent()
    
    armature = Armature.New()
    armatureObj = scene.objects.new(armature, name)
    armatureObj.RotX = math.pi / 2
    armature.makeEditable()
        
    for name in armature_order:
            make_bone(armature, name, base_armature[name][0], base_armature[name][1], base_armature[name][2])
    
    if ignoreOuterJoints:
        for name in outer_joints_order['ignore']:
            make_bone(armature, name, outer_joints['ignore'][name][0], outer_joints['ignore'][name][1], outer_joints['ignore'][name][2])
            bone = Bone()
            bone.name = name
            bone.length = getDistance(outer_joints['ignore'][name][0], outer_joints['ignore'][name][1])
            asf.bones[name] = bone
    else:
        for name in outer_joints_order['normal']:
            make_bone(armature, name, outer_joints['normal'][name][0], outer_joints['normal'][name][1], outer_joints['normal'][name][2])
    
    armatureObj.makeDisplayList()
    
    armature.update()

    return armatureObj

def add_bones(inbone, armature, parent= None):
    """ Adds a bone given by inbone to the armature given by armature."""
    global asf, amc
    abone = Blender.Armature.Editbone()
    if inbone.name == 'root':
        abone.head = Vector(0., 0., 0.)
        # Setting y-length to 0.001 because Blender automatically deletes
        # bones without lengths.
        abone.tail = Vector(0., 0.001, 0.)
    else:
        abone.head = armature.bones[inbone.parent.name].tail
        abone.tail = abone.head + Vector([float(n) * 1.0 * inbone.length for n in inbone.direction])
        abone.parent = parent
        abone.options = [Armature.CONNECTED]
    abone.name = inbone.name
    armature.bones[abone.name] = abone

    for child in inbone.children:
        add_bones(child, armature, abone)

def get_bones():
    """ Returns a dict bones from the currently loaded ASF. """
    return asf.bones
    
def get_animation_origo():
    """ Returns a vector of the current animation's center. """
    return animation_origo
    
def test():
    #stitch('88.asf', '88_01.amc', ['walk.amc', '88_01.amc'], True)
    load('armatur.asf', 'walk.amc', True)
    
def calcOffsetVec(firstAMC, secAMC):
    """ Returns a vector of the offset between two motions."""
    firstAMC_lastframe = firstAMC.frames[len(firstAMC.frames)-1]
    secAMC_firstframe = secAMC.frames[1]
    
    posOffset = Vector(firstAMC_lastframe.root_pos.x - secAMC_firstframe.root_pos.x, firstAMC_lastframe.root_pos.y - secAMC_firstframe.root_pos.y, firstAMC_lastframe.root_pos.z - secAMC_firstframe.root_pos.z)
    
    return posOffset
    
def stitch(asfFilename, firstAmc, stitchFilenames, ignore=False, transitionTime=120):
    """
    Creates a stitched motion from several amc files.

    asfFilename is the ASF filename for the skeleton used for all the
    motions.

    firstAmc is the AMC filename for the first motion

    stitchFilenames is a list of filenames for the rest of the motions.

    ignore is a boolean specifying whether to ignore outer joints or not.

    transitionTime is an integer specifying how many frames to use for
    interpolation.
    """
    global asf, amc, ignoreOuterJoints
    print 'importing and stitching asf/amc...'
    t1 = Blender.sys.time()
    
    try:
        asf = ASF()
        asf.parse(asfFilename)
        ignoreOuterJoints = ignore

        amc = AMC(asf)
        amc.parse(firstAmc)
        newasf = amc.asf
        newasf.calc_matrices()

        arm = make_armature(newasf.name)
        apply_animation(arm, amc)

        posOffset = Vector(0.,0.,0.)
        rotOffset = Vector(0.,0.,0.)

        prevamc = amc
        for amcFilename in stitchFilenames:
            amc = AMC(asf)
            amc.parse(amcFilename)
    
            newPosOffset = calcOffsetVec(prevamc, amc)
    
            posOffset += newPosOffset
    
            apply_animation(arm, amc, prevAMC=prevamc, frameOffset=transitionTime, posOffset=posOffset)
            prevamc = amc

        print 'done in %.4fs' % (Blender.sys.time()-t1)
        return arm.name
    except:
        raise Exception('Fileexcept')
    
def load(asfFilename, amcFilename, ignore=False):
    """ Loads an ASF/AMC file pair and creates an Animated Armature in
    Blender.

    asfFilename is the ASF file to load

    amcFilename is the AMC file to load

    ignore is a boolean specifying whether to ignore outer joints or not.
    """
    global asf, amc, ignoreOuterJoints
    try:
        asf = ASF()
        amc = AMC(asf)
        ignoreOuterJoints = ignore

        print 'importing asf/amc...'
        t1= Blender.sys.time()
        asf.parse(asfFilename)
        amc.parse(amcFilename)
        asf = amc.asf
        asf.calc_matrices()
        arm = make_armature(asf.name)
        arm.addProperty('num_frames',len(amc.frames),'INT')

        apply_animation(arm, amc)

        print 'done in %.4fs' % (Blender.sys.time()-t1)
        return arm.name
    except:
        raise Exception('Fileexcept')

def main():
    global asf, amc
    #test()
    #Blender.Window.FileSelector(read_asf, 'Import ASF', '*.asf')

if __name__ == "__main__":
    main()
