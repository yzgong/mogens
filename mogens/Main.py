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
Name: 'Mogens'
Blender: 249
Group: 'Motion Capture'
Tooltip: 'A Motion Capture utility'
"""
import os
import sys

import Blender
from Blender import Draw, BGL, Modifier, Armature, Material, Camera, Mesh, Window, Object
from Blender.Scene import Render
from Blender.Object.Pose import ROT, LOC
from Blender.Mathutils import *
import import_obj
import math
import string

from mogens import cam_export
from mogens import cam_import
from mogens import pos_export
from mogens import asfamc_import
import os

TITLE_STRING = "MoGens V0.1, Geir Hauge and Dag Stuan"

mogens_dir = os.path.join(Blender.Get("scriptsdir"), "mogens")
mesh_folder = os.path.join(mogens_dir, "meshes")

hasLoaded = False
ignoreOuterJoints = False
stitch = False

ASFString = ""

AMCString = ""
AMC2String = ""
AMC3String = ""
AMC4String = ""
AMC5String = ""

CAMString = ""
OUTPUTString = ""

MESHString = os.path.join(mesh_folder,"Male.obj")

no_action, ASFBUTTON, AMCBUTTON, AMC2BUTTON,\
AMC3BUTTON, AMC4BUTTON, AMC5BUTTON,\
MESHMENU, OUTERJOINTSBUTTON, STITCHBUTTON,\
CameraAutomaticButton, CameraManualButton,\
CamSelectButton, CameraNumberButton, CameraImportButton,\
CameraExportButton, CameraLatitudeButton, CameraOntopButton,\
CameraRadiusButton, CameraOntopLatitudeButton,\
OutputButton, RenderButton, SimulationButton,\
origFpsButton, outputFpsButton = range(25)

Camera_Setup_Selection = {'Automatic setup': [Draw.Create(1),CameraAutomaticButton,'Automatic setup'],
                          'Pre-saved setup': [Draw.Create(0),CameraManualButton,'Pre-saved setup']}

ASF_button = Draw.Create(0)

AMC_button = Draw.Create(0)
AMC2_button = Draw.Create(0)
AMC3_button = Draw.Create(0)
AMC4_button = Draw.Create(0)
AMC5_button = Draw.Create(0)

MESH_menu = Draw.Create(1)
OUTERJOINTS_toggle = Draw.Create(0)

num_motions_button = Draw.Create(1)
motion_transition_button = Draw.Create(120)

select_all_cameras_button = Draw.Create(0)

camera_number_button = Draw.Create(8)
camera_number_button_step = 80.0
camera_number_button_value = 8.0
camera_number_button_maximum = 32.0
camera_number_button_minimum = 2.0

camera_ontop_button = Draw.Create(0)
camera_latitude_button = Draw.Create(math.pi / 4)
camera_ontop_latitude_button = Draw.Create(math.pi * 0.1)
camera_radius_button = Draw.Create(10)

camera_import_button = Draw.Create(0)
camera_export_button = Draw.Create(0)

camera_latitude = math.pi / 4
camera_longitude = 2 * math.pi
camera_radius = 10
cameras_on_top = 0
cameras_on_top_latitude = math.pi * 0.1

original_fps_button = Draw.Create(120)
output_fps_button = Draw.Create(120)

mesh_name = ""
armature_name = ""

scene = Blender.Scene.GetCurrent()

camera_origo = Vector(0.,0.,0.)

orig_bones = None

# To make sure the scales are in meters
camera_scale = 0.250
scale = (1.0 / 0.45) * 2.54 / 100.0


def selectASF(filename):
    """ Import the ASF file """
    global ASFString
    if os.path.splitext(filename)[1] != '.asf':
        Draw.PupMenu('Invalid file extension! You need a *.asf file')
        return
    ASFString = filename

def selectAMC(filename):
    """ Import the first AMC file """
    global AMCString
    if os.path.splitext(filename)[1] != '.amc':
        Draw.PupMenu('Invalid file extension! You need a *.amc file')
        return
    AMCString = filename

def selectAMC2(filename):
    """ Import the second AMC file """
    global AMC2String
    if os.path.splitext(filename)[1] != '.amc':
        Draw.PupMenu('Invalid file extension! You need a *.amc file')
        return
    AMC2String = filename

def selectAMC3(filename):
    """ Import the third AMC file """
    global AMC3String
    if os.path.splitext(filename)[1] != '.amc':
        Draw.PupMenu('Invalid file extension! You need a *.amc file')
        return
    AMC3String = filename

def selectAMC4(filename):
    """ Import the fourth AMC file """
    global AMC4String
    if os.path.splitext(filename)[1] != '.amc':
        Draw.PupMenu('Invalid file extension! You need a *.amc file')
        return
    AMC4String = filename

def selectAMC5(filename):
    """ Import the fifth AMC file """
    global AMC5String
    if os.path.splitext(filename)[1] != '.amc':
        Draw.PupMenu('Invalid file extension! You need a *.amc file')
        return
    AMC5String = filename

def selectMesh(filename):
    """ Select the (mesh) obj file to use. """
    global MESHString
    MESHString = filename
    Draw.Redraw(1)

def selectOutputLocation(filename):
    """ Set the directory to store rendered data in. """
    global OUTPUTString
    OUTPUTString = filename.rsplit('/', 1)[0] + '/'
    Draw.Redraw(1)

def loadASFAMC(ASFString, AMCString):
    """ Load an ASF/AMC pair. """
    global armature_name, orig_bones, camera_origo
    armature_name = asfamc_import.load(ASFString, AMCString, ignoreOuterJoints)
    orig_bones = asfamc_import.get_bones()
    camera_origo = asfamc_import.get_animation_origo() * scale

def stitchASFAMC(ASFString, AMCString, restAMC):
    """
    Load an ASF/AMC pair plus one or more additional motions to stitch.
    """
    global armature_name, orig_bones, camera_origo
    armature_name = asfamc_import.stitch(ASFString, AMCString, restAMC, ignoreOuterJoints, motion_transition_button.val)
    orig_bones = asfamc_import.get_bones()
    camera_origo = asfamc_import.get_animation_origo() * scale

def redrawScene():
    """ Redraws the Scene. """
    # Hide Armature from 3D-view
    # Disabled because the user might wish to view the armature movement.
    # for ob in scene.objects:
    #     if ob.type == 'Armature':
    #         ob.layers = []
    #     ob.makeDisplayList()
    
    # Set last frame of timeline to the last frame of the motion last imported.
    #frames = [ob for ob in scene.objects if ob.name == armature_name][0].getAction().getFrameNumbers()
    frames = Object.Get(armature_name).getProperty('num_frames').getData()
    scene.getRenderingContext().endFrame(frames)
    
    scene.update(1)

def setWorldColor():
    """ Set the world/background color to black. """
    world = Blender.World.GetCurrent()
    world.setHor([0.0, 0.0, 0.0])

def setMeshMaterial():
    """ Set the material for the mesh to shadeless white. """
    meshes = [ob for ob in scene.objects if ob.type == 'Mesh']
    
    for mesh in meshes:
        materials = mesh.getData(mesh=1).materials
        for mat in materials:
            mat.rgbCol =[1.0, 1.0, 1.0]
            mat.mode |= Material.Modes.SHADELESS

def getDistance (first, second):
    """ Returns the euclidian distance between two points. """
    locx = second[0] - first[0]
    locy = second[1] - first[1]
    locz = second[2] - first[2]
    
    distance = math.sqrt((locx)**2 + (locy)**2 + (locz)**2)
    return distance

def resetScale(pose, pose_bones, rest_bones, orig_lengths, child):
    """
    Resets the length of a bone according to the original length. This is
    nessecary because calling the size()-method on a pose_bone also scales
    all the child bones of the bone, thereby making all bones shorter or longer.
    """
    curLength = getDistance(pose_bones[child].head, pose_bones[child].tail)
    origLength = orig_lengths[child]
    
    scale_value = 1 / (curLength / origLength)
    pose_bones[child].size = Vector(pose_bones[child].size.x, pose_bones[child].size.y * scale_value, pose_bones[child].size.z)
    pose.update()
    
    for child in rest_bones[child].children:
        resetScale(pose, pose_bones, rest_bones, orig_lengths, child.name)

def scaleBone(pose, pose_bones, rest_bones, orig_lengths, bone):
    """
    Helper method for scaleArmature() that scales an individual bone while making
    sure that the lengths of all the other bones in the armature stays the same.
    """
    name = bone.name
    
    child_names = [child.name for child in rest_bones[name].children]
    
    children = {}
    
    for child in child_names:
        children[child] = getDistance(pose_bones[name].head, pose_bones[name].tail)
    
    pose_bone_length = getDistance(bone.head, bone.tail)
    orig_bone_length = orig_bones[name].length
    
    scale_value = 1 / (pose_bone_length / orig_bone_length)
    
    bone.size = Vector(bone.size.x, bone.size.y * scale_value, bone.size.z)
    pose.update()
    
    # Done scaling the bone, first resets the rest of the bone lengths
    # Only children are affected by the scaling of a bone, so parents
    # are the same length. Iterates through the childen and resets their
    # scale recursively
    for child in children:
        resetScale(pose, pose_bones, rest_bones, orig_lengths, child)
    
    # Scale has been reset. Recursively scaling each child.
    for child in rest_bones[name].children:
        scaleBone(pose, pose_bones, rest_bones, orig_lengths, pose_bones[child.name])

def scaleArmature():
    """
    Scales the original armature according to the size of the armature read from the
    .asf file.
    """
    armature = [ob for ob in scene.objects if ob.name == armature_name][0]
    pose = armature.getPose()
    pose_bones = pose.bones
    rest_bones = armature.getData().bones
    
    bone_lengths = {}
    
    for bone in pose_bones.values():
        bone_lengths[bone.name] = getDistance(pose_bones[bone.name].head, pose_bones[bone.name].tail)
    
    root = rest_bones['root']
    
    for bone in root.children:
        scaleBone(pose, pose_bones, rest_bones, bone_lengths, pose_bones[bone.name])
    
    # Sets the size of the armature to the scale given by CMU
    armature.setSize(scale, scale, scale)

def setUpMesh():
    """ Imports the mesh and connects it to the armature. """
    print 'Setting up mesh...'
    t1= Blender.sys.time()
    
    # A little hack because import_obj.py doesnt provide the names of the imported objects
    objectsBefore = [ob for ob in scene.objects]
    
    import_obj.load_obj(MESHString)
    
    names = [ob.name for ob in scene.objects if ob not in objectsBefore]
    
    global mesh_name
    # The mesh created in makehuman is split up in parts, and these need to be
    # joined together before connecting it to the armature. Uses 'body' as the
    # main mesh part and connects all other parts to this.
    mesh_name = [s for s in names if 'body' in s][0]
    restmesh_name = [s for s in names if s is not mesh_name]
    
    armature = [ob for ob in scene.objects if ob.name == armature_name][0]
    armature.RotX = math.pi/2
    
    bodymesh = [ob for ob in scene.objects if ob.name == mesh_name][0]
    restmesh = [ob for ob in scene.objects if ob.name in restmesh_name]
    
    bodymesh.join(restmesh)
    
    for ob in restmesh:
        scene.objects.unlink(ob)
    
    print 'done moving armature and joining mesh parts, it took %.4fs' % (Blender.sys.time()-t1)
    t2 = Blender.sys.time()
    
    editmode = Window.EditMode()    # are we in edit mode?  If so ...
    if editmode: Window.EditMode(0) # leave edit mode
    
    # Actual code for connection to armature. We still need to fix the remaining
    # verts though. This is done in fixVerts()
    armature.makeParent([bodymesh])
    bodymesh.addVertexGroupsFromArmature(armature)
    mods = bodymesh.modifiers
    mod = mods.append(Modifier.Types.ARMATURE)
    mod[Modifier.Settings.OBJECT] = armature
    mod[Modifier.Settings.VGROUPS] = True
    mod[Modifier.Settings.ENVELOPES] = True
    
    print 'done adding mesh using bone heat, it took %.4fs' % (Blender.sys.time()-t2)
    
    if editmode: Window.EditMode(1)  # optional, just being nice
    print 'Setting up mesh took %.4fs' % (Blender.sys.time()-t1)

def fixVerts():
    """
    Make sure all the mesh's vertices are connected to the armature.
    Those that aren't get connected to the closest bone.
    """
    print 'Fixing vertices...'
    t1= Blender.sys.time()
    mesh = [ob for ob in scene.objects if ob.name == mesh_name][0].getData()
    armature = [ob for ob in scene.objects if ob.name == armature_name][0]
    armatureMatrix = armature.mat
    armature = armature.getData()
    
    meshMatrix = [ob for ob in scene.objects if ob.name == mesh_name][0].mat
    
    vGroups = mesh.getVertGroupNames()
    
    # Figuring out which verts were left out of the bone heat method.
    vertsWithGroups = set()
    for group in vGroups:
        for vert, weight in mesh.getVertsFromGroup(group, 1):
            if weight < 0.001:
                mesh.assignVertsToGroup(group, [vert], 0.001, 'replace')
            vertsWithGroups.add(vert)
    
    vertsWithoutGroups = set(range(len(mesh.verts)))-vertsWithGroups
    print 'figured out which vertices are missing a group, it took %.4fs' % (Blender.sys.time()-t1)
    t1 = Blender.sys.time()
    
    # Figured out which vertices are left behind, calculating the closest bone for each vertex
    for vertex in vertsWithoutGroups:
        vert = mesh.verts[vertex]
        vertList = [vertex]
        closestDistance = 999999
        closestBone = None
        coords = vert.co * meshMatrix
        for bone in armature.bones.values():
            if bone.name != 'root':
                worldBoneCoords = (bone.matrix['ARMATURESPACE'] * armatureMatrix).translationPart()
                
                distance = getDistance(coords, worldBoneCoords)
                if distance < closestDistance:
                    closestDistance = distance
                    closestBone = bone
        
        # Assigns the lowest weight possible, since the vertex wasnt picked up in the
        # first place it's obviously far away from the bone
        mesh.assignVertsToGroup(closestBone.name, vertList, 0.001, 'add')
    
    #Function to ignore outer joints
    if ignoreOuterJoints:
        for group in vGroups:
            if group == 'head':
                continue
            if not armature.bones[group].children or 'hand' in group:
                parent = armature.bones[group].parent
                if 'hand' in parent.name:
                    parent = parent.parent
                vertList = []
                for vert in mesh.getVertsFromGroup(group):
                    vertList.append(vert)
                mesh.removeVertsFromGroup(group, vertList)
                mesh.assignVertsToGroup(parent.name, vertList, 0.001, 'add')
    print 'Fixing the rest of the vertices took %.4fs' % (Blender.sys.time()-t1)

def clearCameras():
    """ Removes all cameras from the scene. """
    cameras = [cam for cam in scene.objects if cam.type == 'Camera']
    
    for cam in cameras:
        scene.objects.unlink(cam)
    
    Blender.Redraw()

def addCamera(camObj, camLongitude, ontop=False):
    """ Add a camera object to the scene. """
    camObj.RotZ = camLongitude + math.pi/2

    if not ontop:
        camObj.RotX = camera_latitude
        
        x = camera_radius * math.sin(camera_latitude) * math.cos(camLongitude)
        y = camera_radius * math.sin(camera_latitude) * math.sin(camLongitude)
        z = camera_radius * math.cos(camera_latitude)
    else:
        x = camera_radius * math.sin(cameras_on_top_latitude) * math.cos(camLongitude)
        y = camera_radius * math.sin(cameras_on_top_latitude) * math.sin(camLongitude)
        z = camera_radius * math.cos(camera_latitude) * 1.5
    
    camObj.loc = [x + camera_origo.x, y + camera_origo.y, z + camera_origo.z]
    camObj.setSize(camera_scale, camera_scale, camera_scale)

def setUpCameras(numCams):
    """
    Set up numCams cameras around the origo/animation center, evenly placed
    along a given latitude.
    """
    global CAMString
    CAMString = ""
    clearCameras()
    
    # Reuses already unlinked cameras because Blender never deletes objects from memory,
    # thereby avoiding memory leakage (which isnt really leakage, just works the same way).
    unlinkedCameras = [cam for cam in Blender.Object.Get() if cam not in scene.objects and cam.type == 'Camera']
    
    for cam in unlinkedCameras:
        cam.loc = [0.,0.,0.]
        cam.rot = [0.,0.,0.]
    
    numUnlinkedCameras = len(unlinkedCameras) - 1
    
    for i in range(cameras_on_top):
        camLongitude = (camera_longitude) * (i+1)/cameras_on_top
        if numUnlinkedCameras >= 0:
            camObj = unlinkedCameras[numUnlinkedCameras]
            scene.link(camObj)
            addCamera(camObj, camLongitude, True)
            numUnlinkedCameras -= 1
        else:
            cam = Camera.New('persp')
            camObj = scene.objects.new(cam)
            addCamera(camObj, camLongitude, True)
    
    for i in range(int(numCams) - cameras_on_top):
        camLongitude = camera_longitude * (i+1)/(numCams - cameras_on_top)
        if numUnlinkedCameras  >= 0:
            camObj = unlinkedCameras[numUnlinkedCameras]
            scene.link(camObj)
            addCamera(camObj, camLongitude)
            numUnlinkedCameras -= 1
        
        else:
            cam = Camera.New('persp')
            camObj = scene.objects.new(cam)
            addCamera(camObj, camLongitude)
    
    Select_All_Cameras()
    
    Draw.Redraw(1)
    Blender.Redraw()

def loadCameraSetup(filename):
    """ Load and setup cameras from the given XML file. """
    global CAMString
    CAMString = filename
    
    clearCameras()
    
    cam_import.read(filename)
    
    for cam in [obj for obj in scene.objects if obj.type == 'Camera']:
        cam.setSize(camera_scale, camera_scale, camera_scale)
    
    Blender.Redraw()
    Draw.Redraw(1)

def saveCameraSetup(filename):
    """
    Save all the cameras currently in the scene to an XML file given by
    filename.
    """
    global CAMString, Camera_Setup_Selection
    CAMString = filename
    
    Camera_Setup_Selection['Automatic setup'][0].val = 0
    Camera_Setup_Selection['Pre-saved setup'][0].val = 1
    
    Draw.Redraw(1)
    cam_export.write(filename)

def exportJointPositions(filepath):
    """
    Write current joint posistions to file.
    """
    pos_export.write(os.path.join(filepath, 'jointPositions'), scale)

def renderScene(filepath):
    """
    Start the rendering of the scene to the given filepath.

    For each camera, a directory will be created with the camera's name, and
    all the images taken by that camera will be placed there.
    
    In addition, joint positions and camera positions will be written to
    files, as well as the camera setup.
    """
    context = scene.getRenderingContext()
    
    cameras = [cam for cam in scene.objects if cam.type == 'Camera']
    
    action = [ob for ob in scene.objects if ob.name == armature_name][0].getAction()
    
    for cam in cameras:
        scene.objects.camera = cam
        context.displayMode = 1
        context.extensions = True
        context.renderPath = filepath + '%s/' % cam.name
        context.sFrame = 1
        
        context.eFrame = int(max(action.getFrameNumbers()) * (float(context.newMapValue()) / float(context.oldMapValue())))
        context.renderAnim()
    
    Render.CloseRenderWindow()
    
    outputCamFile = os.path.join(filepath,"calibration.xml")
    saveCameraSetup(outputCamFile)
    
    exportJointPositions(filepath)

def setUpScene():
    """ Sets up the cameras and FPS. """
    for ob in scene.objects: scene.objects.unlink(ob)
    setUpCameras(camera_number_button_value)
    
    context = scene.getRenderingContext()
    context.setImageType(Render.PNG)
    context.fps = 120
    context.oldMapValue(120)
    context.newMapValue(120)

def Select_All_Cameras(evt=None, val=None):
    """ Selects all cameras in the scene, deselects everything else. """
    for ob in scene.objects:
        if ob.type == 'Camera':
            ob.select(True);
        else:
            ob.select(False);
    Blender.Redraw()

# Helper method to make sure that the camera number button has steps in powers of 2
def camButtonClicked(evt,val):
    """ callback for the camera slider button, increasing and decreasing by
    powers of 2 instead of using a static step. """
    global camera_number_button_value, camera_number_button_step
    if val > camera_number_button_value and val <= camera_number_button_maximum:
        camera_number_button.val = val
        camera_number_button_value = val
        camera_number_button_step = camera_number_button.val * 10.0
    elif val < camera_number_button_value and val >= camera_number_button_minimum:
        camera_number_button.val = camera_number_button_value / 2
        camera_number_button_value = camera_number_button_value / 2
        camera_number_button_step = camera_number_button.val * 10.0

def Draw_Border(X1,Y1,X2,Y2): # X1,Y1 = Top Left X2,Y2 = Bottom Right
    """
    Draw a border given a top left corner (X1, X2) and bottom right (X2, Y2)
    """
    INDENT = 3
    
    BGL.glColor3f(1.0,1.0,1.0)
    BGL.glBegin(BGL.GL_LINES)
    BGL.glVertex2i(X1+INDENT,Y1-INDENT)     #top line
    BGL.glVertex2i(X2-INDENT,Y1-INDENT)
    
    BGL.glVertex2i(X1+INDENT,Y1-INDENT)     #left line
    BGL.glVertex2i(X1+INDENT,Y2+INDENT)
    BGL.glEnd()
    
    BGL.glColor3f(0.5,0.5,0.5)
    BGL.glBegin(BGL.GL_LINES)
    BGL.glVertex2i(X2-INDENT,Y1-INDENT)     #Right line
    BGL.glVertex2i(X2-INDENT,Y2+INDENT)
    
    BGL.glVertex2i(X1+INDENT,Y2+INDENT)     #bottom line
    BGL.glVertex2i(X2-INDENT,Y2+INDENT)
    BGL.glEnd()

def Create_Tab(X1,Y1,X2,Y2,Title,Buttons): # X1,Y1 = Top Left X2,Y2 = Bottom Right
    """
    Create a bordered tab/frame/box with the given top left corner (X1,Y1)
    and bottom right corner (X2, Y2) with the given Title and Buttons.
    """
    TITLE_HEIGHT = 15
    INDENT = 6
    BUTTON_GAP = 4
    
    BGL.glColor3f(0.75, 0.75, 0.75)
    BGL.glRecti(X1,Y1,X2,Y2)
    
    Draw_Border(X1,Y1,X2,Y2);
    
    BGL.glColor3f(0.0,0.0,0.0)
    BGL.glRasterPos2d(X1+INDENT,Y1 - TITLE_HEIGHT)
    Draw.Text(Title)
    
    BUTTON_HEIGHT = 18
    
    Button_X = X1 + INDENT
    Button_Y = Y1 - TITLE_HEIGHT - BUTTON_HEIGHT - 8
    
    if (Buttons != 0):
        key= Buttons.keys()
        BUTTON_WIDTH = (X2 - (X1 + INDENT + BUTTON_GAP + INDENT)) / len(key)
        for k in key:
            Buttons[k][0]= Draw.Toggle(k,Buttons[k][1],Button_X,Button_Y, BUTTON_WIDTH,BUTTON_HEIGHT,Buttons[k][0].val,Buttons[k][2])
            Button_X += BUTTON_WIDTH + BUTTON_GAP

def Display_Title_Bar(Y_POS, CONTROL_HEIGHT, CONTROL_WIDTH):
    """ Create the title bar with version and credits. """
    Create_Tab(3, Y_POS, CONTROL_WIDTH, Y_POS - CONTROL_HEIGHT, TITLE_STRING, 0)

def Display_File_Bar(Y_POS, CONTROL_HEIGHT, CONTROL_WIDTH):
    """ Create the File setup box."""
    global num_motions_button
    Create_Tab(3, Y_POS, CONTROL_WIDTH, Y_POS - CONTROL_HEIGHT, "File setup", 0)
    
    RIGHT_LIMIT = 90
    global ASF_button
    ASF_button = Draw.PushButton('ASF-file', ASFBUTTON, 9, (Y_POS - 41), 65, 18,  'ASF-file')
    
    BGL.glRasterPos2i(80, (Y_POS - 37))
    if ASFString:
        tempString = ASFString
        if (Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT)):
            while Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT):
                tempString = tempString[2:]
            Draw.Text('...' + tempString)
        else:
            Draw.Text(ASFString)
    else:
        Draw.Text('Please select a file...')
    
    global AMC_button
    AMC_button = Draw.PushButton('AMC-file', AMCBUTTON, 9, (Y_POS - 63), 65, 18,  'AMC-file')
    
    BGL.glRasterPos2i(80, (Y_POS - 58))
    if AMCString:
        tempString = AMCString
        if (Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT)):
            while Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT):
                tempString = tempString[2:]
            Draw.Text('...' + tempString)
        else:
            Draw.Text(AMCString)
    else:
        Draw.Text('Please select a file...')
    
    if num_motions_button.val > 1:
        global AMC2_button
        AMC2_button = Draw.PushButton('AMC2-file', AMC2BUTTON, 9, (Y_POS - 85), 65, 18,  'AMC-file')
        
        BGL.glRasterPos2i(80, (Y_POS - 80))
        if stitch and AMC2String:
            tempString = AMC2String
            if (Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT)):
                while Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT):
                    tempString = tempString[2:]
                Draw.Text('...' + tempString)
            else:
                Draw.Text(AMC2String)
        else:
            Draw.Text('Please select a file...')
    if num_motions_button.val > 2:
        global AMC3_button
        AMC3_button = Draw.PushButton('AMC3-file', AMC3BUTTON, 9, (Y_POS - 107), 65, 18,  'AMC-file')
        
        BGL.glRasterPos2i(80, (Y_POS - 101))
        if AMC3String:
            tempString = AMC3String
            if (Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT)):
                while Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT):
                    tempString = tempString[2:]
                Draw.Text('...' + tempString)
            else:
                Draw.Text(AMC3String)
        else:
            Draw.Text('Please select a file...')
    if num_motions_button.val > 3:
        global AMC4_button
        AMC4_button = Draw.PushButton('AMC4-file', AMC4BUTTON, 9, (Y_POS - 129), 65, 18,  'AMC-file')
        
        BGL.glRasterPos2i(80, (Y_POS - 123))
        if AMC4String:
            tempString = AMC4String
            if (Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT)):
                while Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT):
                    tempString = tempString[2:]
                Draw.Text('...' + tempString)
            else:
                Draw.Text(AMC4String)
        else:
            Draw.Text('Please select a file...')
    if num_motions_button.val > 4:
        global AMC5_button
        AMC5_button = Draw.PushButton('AMC5-file', AMC5BUTTON, 9, (Y_POS - 151), 65, 18,  'AMC-file')
        
        BGL.glRasterPos2i(80, (Y_POS - 145))
        if AMC5String:
            tempString = AMC5String
            if (Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT)):
                while Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT):
                    tempString = tempString[2:]
                Draw.Text('...' + tempString)
            else:
                Draw.Text(AMC5String)
        else:
            Draw.Text('Please select a file...')
    
    num_motions_button = Draw.Number("Number of motions:", STITCHBUTTON, 9, (Y_POS - 63 - (22*num_motions_button.val)), (CONTROL_WIDTH / 2) - 9, 18, num_motions_button.val, 1, 5, 'The number of motions to be imported')
    
    if num_motions_button.val > 1:
        global motion_transition_button
        motion_transition_button = Draw.Number("Transition time (frames):", no_action, ((CONTROL_WIDTH / 2) + 3), (Y_POS - 63 - (22*num_motions_button.val)), (CONTROL_WIDTH / 2) - 9, 18, motion_transition_button.val, 1, 1000, 'The number of frames between each motion')
    

def Display_Mesh_Bar(Y_POS, CONTROL_HEIGHT, CONTROL_WIDTH):
    """ Create the mesh setup box. """
    Create_Tab(3, Y_POS, CONTROL_WIDTH, Y_POS - CONTROL_HEIGHT, "Mesh setup", 0)
    
    menuItems = "Mesh %t|Male %x1|Female %x2"
    global MESH_menu
    MESH_menu = Draw.Menu(menuItems, MESHMENU, 9, (Y_POS - 41), 130, 18, MESH_menu.val, 'Select mesh (.obj)')
    
    global OUTERJOINTS_toggle
    OUTERJOINTS_toggle = Draw.Toggle('Ignore outer joints', OUTERJOINTSBUTTON, ((CONTROL_WIDTH / 2) + 3), (Y_POS - 41), (CONTROL_WIDTH / 2) - 9, 18, OUTERJOINTS_toggle.val, 'Ignores the outer joints of the animation (fingers and toes)')

def Display_Camera_Bar(Y_POS, CONTROL_HEIGHT, CONTROL_WIDTH):
    """ Create the Camera setup box. """
    Create_Tab(3, Y_POS, CONTROL_WIDTH, Y_POS - CONTROL_HEIGHT, "Camera setup", Camera_Setup_Selection)
    
    global select_all_cameras_button
    select_all_cameras_button = Draw.PushButton('Select all cameras', no_action, ((CONTROL_WIDTH / 2) + 3), (Y_POS - 22), (CONTROL_WIDTH / 2) - 9, 16, 'Selects all cameras in the scene', Select_All_Cameras)
    
    if Camera_Setup_Selection['Automatic setup'][0].val:
        global camera_number_button
        camera_number_button = Draw.Number("Number of cameras:", CameraNumberButton, 9, (Y_POS - 63), (CONTROL_WIDTH / 2) - 9, 18, camera_number_button_value, camera_number_button_minimum, camera_number_button_maximum, 'Number of cameras to be set up in the scene', camButtonClicked, camera_number_button_step)
        
        global camera_latitude_button
        camera_latitude_button = Draw.Number("Latitude:", CameraLatitudeButton, ((CONTROL_WIDTH / 2) + 3), (Y_POS - 63), (CONTROL_WIDTH / 2) - 9, 18, camera_latitude_button.val, 0, math.pi/2, 'General latitude of the cameras')
        
        global camera_ontop_button
        camera_ontop_button = Draw.Number("Ceiling cameras:", CameraOntopButton, 9, (Y_POS - 84), (CONTROL_WIDTH / 2) - 9, 18, camera_ontop_button.val, 0, 4, 'Number of cameras to be positioned in the ceiling', lambda x,y:None, 10)
        
        if camera_ontop_button.val:
            global camera_ontop_latitude_button
            camera_ontop_latitude_button = Draw.Number("Ceiling latitude:", CameraOntopLatitudeButton, ((CONTROL_WIDTH / 2) + 3), (Y_POS - 84), (CONTROL_WIDTH / 2) - 9, 18, camera_ontop_latitude_button.val, 0, math.pi/2, 'Latitude of the cameras in the ceiling')
        
        global camera_radius_button
        camera_radius_button = Draw.Number("Radius:", CameraRadiusButton, ((CONTROL_WIDTH / 2) + 3), (Y_POS - CONTROL_HEIGHT) + 6, (CONTROL_WIDTH / 2) - 9, 18, camera_radius_button.val, 1, 40, 'The radius of the circle the cameras are placed in, in meters', lambda x,y:None, 10)
    
    if Camera_Setup_Selection['Pre-saved setup'][0].val:
        global camera_import_button
        camera_import_button = Draw.PushButton('Select camera setup (.xml)', CameraImportButton, 9, (Y_POS - 63), (CONTROL_WIDTH / 2) - 9, 18,  'Select camera setup (.xml)')
        
        BGL.glRasterPos2i(11, (Y_POS - 80))
        if CAMString:
            RIGHT_LIMIT = 6
            tempString = CAMString
            if (Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT)):
                while Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT):
                    tempString = tempString[2:]
                Draw.Text('...' + tempString)
            else:
                Draw.Text(CAMString)
        else:
            Draw.Text('Please select a file...')
    
    global camera_export_button
    camera_export_button = Draw.PushButton('Export current setup', CameraExportButton, 9, (Y_POS - CONTROL_HEIGHT) + 6, (CONTROL_WIDTH / 2) - 9, 18, 'Press this button to save your camera setup')

def Display_Render_Bar(Y_POS, CONTROL_HEIGHT, CONTROL_WIDTH):
    """ Create the Render setup box."""
    Create_Tab(3, Y_POS, CONTROL_WIDTH, Y_POS - CONTROL_HEIGHT, "Render setup", 0)
    
    global original_fps_button
    original_fps_button = Draw.Number("Original FPS:", origFpsButton, 9, (Y_POS - 41), (CONTROL_WIDTH / 2) - 9, 18, original_fps_button.val, 1, 120, 'Orignal FPS of the motion imported')
    
    global output_fps_button
    output_fps_button = Draw.Number("Output FPS:", outputFpsButton, ((CONTROL_WIDTH / 2) + 3), (Y_POS - 41), (CONTROL_WIDTH / 2) - 9, 18, output_fps_button.val, 1, 120, 'The desired output FPS of the motion')

def Display_Output_Bar(Y_POS, CONTROL_HEIGHT, CONTROL_WIDTH):
    """ Create the Display output box."""
    Create_Tab(3, Y_POS, CONTROL_WIDTH, Y_POS - CONTROL_HEIGHT, "Output setup", 0)
    
    Draw.PushButton('Select output location', OutputButton, 9, (Y_POS - 41), 130, 18, 'Select output location')
    
    BGL.glRasterPos2i(143, (Y_POS - 36))
    if OUTPUTString:
        RIGHT_LIMIT = 144
        tempString = OUTPUTString
        if (Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT)):
            while Draw.GetStringWidth('...' + tempString) > (CONTROL_WIDTH - RIGHT_LIMIT):
                tempString = tempString[2:]
            Draw.Text('...' + tempString)
        else:
            Draw.Text(OUTPUTString)
    else:
        Draw.Text('Please select a folder...')

def drawGUI():
    """ Create and draw the GUI. """
    HEIGHT = 382
    CONTROL_WIDTH = 400
    
    BGL.glClearColor(0.6, 0.6, 0.6, 1.0)
    BGL.glClear(BGL.GL_COLOR_BUFFER_BIT)
    
    BGL.glColor3f(0.75, 0.75, 0.75)
    BGL.glRecti(3,45,CONTROL_WIDTH,3)
    
    stitchOffset = 22
    HEIGHT += stitchOffset * num_motions_button.val
    
    Display_Title_Bar(HEIGHT,25, CONTROL_WIDTH)
    
    Display_File_Bar(HEIGHT - 25, 71 + stitchOffset * num_motions_button.val, CONTROL_WIDTH)
    
    Display_Mesh_Bar(HEIGHT - 96 - stitchOffset * num_motions_button.val, 48, CONTROL_WIDTH)
    
    Display_Camera_Bar(HEIGHT - 144 - stitchOffset * num_motions_button.val, 112, CONTROL_WIDTH)
    
    Display_Render_Bar(HEIGHT - 256 - stitchOffset * num_motions_button.val, 48, CONTROL_WIDTH)
    
    Display_Output_Bar(HEIGHT - 304 - stitchOffset * num_motions_button.val, 48, CONTROL_WIDTH)
    
    Draw.PushButton('Load simulation', SimulationButton, 9, 8, (CONTROL_WIDTH / 2) - 9, 18, 'Load simulation')
    if hasLoaded:
        Draw.PushButton('Render scene', RenderButton, (CONTROL_WIDTH / 2) + 3, 8, (CONTROL_WIDTH / 2) - 9, 18, 'Render scene')

def event(evt,val):
    """
    Handles keyboard events.
    ESC exits.
    """
    if evt == Blender.Draw.ESCKEY:
        Blender.Draw.Exit()
        return

def button(evt):
    """ Callback function for the buttons. """
    global UseBVH, stitch
    if evt == ASFBUTTON:
        Blender.Window.FileSelector (selectASF, 'Load ASF Motion Capture', '*.asf')
    elif evt == AMCBUTTON:
        Blender.Window.FileSelector (selectAMC, 'Load AMC Motion Capture', '*.amc')
    elif evt == AMC2BUTTON:
        Blender.Window.FileSelector (selectAMC2, 'Load AMC Motion Capture', '*.amc')
    elif evt == AMC3BUTTON:
        Blender.Window.FileSelector (selectAMC3, 'Load AMC Motion Capture', '*.amc')
    elif evt == AMC4BUTTON:
        Blender.Window.FileSelector (selectAMC4, 'Load AMC Motion Capture', '*.amc')
    elif evt == AMC5BUTTON:
        Blender.Window.FileSelector (selectAMC5, 'Load AMC Motion Capture', '*.amc')
    elif evt == SimulationButton:
        global hasLoaded
        
        if not ASFString:
            Draw.PupMenu('You need to select an ASF-file!')
            return
        if not AMCString:
            Draw.PupMenu('You need to select an AMC-file!')
            return
        if num_motions_button.val > 1 and not AMC2String:
            Draw.PupMenu('You need to select the second AMC-file!')
            return
        if num_motions_button.val > 2 and not AMC3String:
            Draw.PupMenu('You need to select the third AMC-file!')
            return
        if num_motions_button.val > 3 and not AMC4String:
            Draw.PupMenu('You need to select the fourth AMC-file!')
            return
        if num_motions_button.val > 4 and not AMC5String:
            Draw.PupMenu('You need to select the fifth AMC-file!')
            return
        
        Blender.Window.WaitCursor (True)
        t1= Blender.sys.time()
        
        context = scene.getRenderingContext()
        
        # Saving the fps of the context for later use
        oldfps = context.fps
        oldmapvalue = context.oldMapValue()
        newmapvalue = context.newMapValue()
        
        # Resetting the fps of the scene, this is done to make sure
        # new motions are imported correctly and synched with already
        # imported motions.
        context.fps = 120
        context.oldMapValue(120)
        context.newMapValue(120)
        
        Blender.Window.DrawProgressBar (0.0 / 6.0, 'loading ASF/AMC')
        if stitch:
            try:
                if num_motions_button.val == 2:
                    stitchASFAMC(ASFString, AMCString, [AMC2String])
                elif num_motions_button.val == 3:
                    stitchASFAMC(ASFString, AMCString, [AMC2String, AMC3String])
                elif num_motions_button.val == 4:
                    stitchASFAMC(ASFString, AMCString, [AMC2String, AMC3String, AMC4String])
                elif num_motions_button.val == 5:
                    stitchASFAMC(ASFString, AMCString, [AMC2String, AMC3String, AMC4String, AMC5String])
            except:
                Draw.PupMenu('One of your files doesnt conform to the standards of CMU.')
                return
        else:
            try:
                loadASFAMC(ASFString, AMCString)
            except:
                Draw.PupMenu('Your ASF/AMC pair doesnt conform to the standards of CMU.')
                return
        Blender.Window.DrawProgressBar (1.0 / 6.0, 'loading Mesh')
        setUpMesh()
        Blender.Window.DrawProgressBar (2.0 / 5.0, 'Fixing vertices')
        fixVerts()
        Blender.Window.DrawProgressBar (3.0 / 5.0, 'Scaling armature')
        scaleArmature()
        Blender.Window.DrawProgressBar (4.0 / 5.0, 'Redrawing scene')
        
        # Setting the fps back to the values selected by the user
        context.fps = oldfps
        context.oldMapValue(oldmapvalue)
        context.newMapValue(newmapvalue)
        
        redrawScene()
        Blender.Window.WaitCursor (False)
        Blender.Window.DrawProgressBar (1, 'Done!')
        print 'Simulation loading took %.4fs' % (Blender.sys.time()-t1)
        hasLoaded = True
    elif evt == RenderButton:
        if not OUTPUTString:
            Draw.PupMenu('You need to select an output folder!')
            return
        setWorldColor()
        setMeshMaterial()
        renderScene(OUTPUTString)
    elif evt == OUTERJOINTSBUTTON:
        global ignoreOuterJoints
        ignoreOuterJoints = OUTERJOINTS_toggle.val
    elif evt == STITCHBUTTON:
        if num_motions_button.val > 1:
            stitch = True
        else:
            stitch = False
        Draw.Redraw(1)
    elif evt == CameraAutomaticButton:
        Camera_Setup_Selection['Automatic setup'][0].val = 1
        Camera_Setup_Selection['Pre-saved setup'][0].val = 0
        Draw.Redraw(1)
    elif evt == CameraManualButton:
        Camera_Setup_Selection['Automatic setup'][0].val = 0
        Camera_Setup_Selection['Pre-saved setup'][0].val = 1
        Draw.Redraw(1)
    elif evt == CameraLatitudeButton:
        global camera_latitude
        camera_latitude = camera_latitude_button.val
        setUpCameras(camera_number_button.val)
    elif evt == CameraNumberButton:
        global camera_ontop_button, cameras_on_top
        if(camera_number_button.val < camera_ontop_button.val):
            cameras_on_top = int(camera_number_button.val)
            camera_ontop_button.val = cameras_on_top
        setUpCameras(camera_number_button.val)
    elif evt == CameraOntopButton:
        cameras_on_top = camera_ontop_button.val
        setUpCameras(camera_number_button.val)
    elif evt == CameraOntopLatitudeButton:
        global cameras_on_top_latitude
        cameras_on_top_latitude = camera_ontop_latitude_button.val
        setUpCameras(camera_number_button.val)
    elif evt == CameraRadiusButton:
        global camera_radius
        camera_radius = camera_radius_button.val
        setUpCameras(camera_number_button.val)
    elif evt == CameraImportButton:
        Blender.Window.FileSelector(loadCameraSetup, 'Load Camera Setup (.xml)', '*.xml')
    elif evt == CameraExportButton:
        Blender.Window.FileSelector(saveCameraSetup, 'Save Camera Setup as...', '*.xml')
    elif evt == origFpsButton:
        scene.getRenderingContext().oldMapValue(original_fps_button.val)
    elif evt == outputFpsButton:
        context = scene.getRenderingContext()
        context.newMapValue(output_fps_button.val)
        context.fps = output_fps_button.val
        context.endFrame(Object.Get(armature_name).getProperty('num_frames').getData()*context.fps/120)
    elif evt == OutputButton:
        Blender.Window.FileSelector(selectOutputLocation, 'Select Folder')
    elif evt == MESHMENU:
        global MESHString
        if MESH_menu.val == 1:
            MESHString = os.path.join(mesh_folder, "Male.obj")
        elif MESH_menu.val == 2:
            MESHString = os.path.join(mesh_folder, "Female.obj")
	
	Draw.Redraw(1)
	Blender.Window.RedrawAll()

setUpScene()
Draw.Register(drawGUI,event,button)
