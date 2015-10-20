# Introduction #

User's guide for the Mogens Blender plug-in and MoGrapher analysis tool by Geir Hauge and Dag Stuan.

# Requirements #

The following lists the required packages that need to be installed before the Blender and Graph modules can be installed.

  1. **Blender 2.49b** This was the latest stable version of Blender at the start of the project, and is the only one we support. http://download.blender.org/release/Blender2.49b/
  1. **Python 2.5 or 2.6** Blender uses Python 2.x, and the Graph module is written and tested with Python 2.5 and 2.6. It should work with Python 2.7 too. http://python.org/
  1. **NumPy** NumPy adds efficient computing of matrices, amongst other things, which is a necessity when doing 3D calculations. NumPy is used by the Blender Module and matplotlib. http://numpy.scipy.org/
  1. **matplotlib 1.0** matplotlib is a very versatile python library for plotting advanced graphs and more. This is required by the Graph module. http://matplotlib.sourceforge.net/

# Installation #

This system consists of two modules. A Blender module and a graph-module.

## Installing the Blender module ##

To install the Blender module, copy the folder named <tt>mogens</tt> from the source tree to Blender’s scripts folder. Where this folder is located depends on your system, but in general, you should find it in

  * <tt>C:\Users\<your username>\AppData\Roaming\Blender Foundation\Blender\.blender\scripts</tt> (Windows Vista and newer)
  * <tt>~/Applications/blender/blender.app/Contents/MacOS/.blender/scripts</tt> (Mac OSX)
  * <tt>~/.blender/scripts</tt> (Linux)

Consult the Blender documentation for further help http://wiki.blender.org/index.php/Doc:Manual/Introduction/Installing_Blender/Python.

## Installing MoGrapher ##

MoGrapher doesn’t really need installing. It’s just one python script file. It is located in the <tt>mographer</tt> directory in the source tree. Place it wherever you prefer. On Mac OSX and Linux, a logical place to put it would be <tt>~/bin/</tt> and make sure that directory is listed in your <tt>PATH</tt> environment variable. If you want to make it globally available on your system, then <tt>/usr/local/bin/</tt> is recommended. If you do install it to <tt>~/bin/</tt> or <tt>/usr/local/bin</tt>, remember to make the file executable, and it is recommended that you rename the file from <tt>mographer.py</tt> to <tt>mographer</tt>, after which you can execute it by running <tt>mographer</tt> in your shell.

# Usage #

## Using the Blender module ##

**Fig. 1 Steps to run Mogens**
| ![https://mogens.googlecode.com/svn/wiki/images/blender-select-window.png](https://mogens.googlecode.com/svn/wiki/images/blender-select-window.png) | ![https://mogens.googlecode.com/svn/wiki/images/location.png](https://mogens.googlecode.com/svn/wiki/images/location.png) |
|:----------------------------------------------------------------------------------------------------------------------------------------------------|:--------------------------------------------------------------------------------------------------------------------------|
| (a) Selecting the Scripts Window                                                                                                                    | (b) Finding and starting Mogens                                                                                           |

To use the Blender module, start up Blender and split a window. Make one of the windows a Scripts Window as shown Fig. 1a. In the Scripts Window’s menu bar, select Scripts->Misc->Mogens as shown in Fig. 1b. The Mogens user interface, as shown in Fig. 2, should now appear in this window.

**Fig. 2 The Blender module's user interface**
| ![https://mogens.googlecode.com/svn/wiki/images/gui.png](https://mogens.googlecode.com/svn/wiki/images/gui.png) |
|:----------------------------------------------------------------------------------------------------------------|

The user interface is ordered in several sections, described in the following text.

### File setup ###

**Fig. 3 The file setup section of the Blender UI**
| ![https://mogens.googlecode.com/svn/wiki/images/blenderfilesetup.png](https://mogens.googlecode.com/svn/wiki/images/blenderfilesetup.png) |
|:------------------------------------------------------------------------------------------------------------------------------------------|

In this part of the user interface, you select the skeleton (ASF) file and one or more motion (AMC) files. Clicking the buttons will bring up a blender file dialog where you can browse your filesystems for the files you want to load. By adjusting the spinner labeled _Number of motions_ (from 1 to 5), the number of AMC-file buttons will change, and when the number of motions is set to more than one, a new slider will also appear. With this slider you select the number of frames of transition between each motion, which will affect how the motion stitching is done. Fig. 3 shows the File setup section with _Number of motions_ set to 5, but no files currently selected.

### Mesh setup ###

**Fig. 4 The mesh setup section of the Blender UI**
| ![https://mogens.googlecode.com/svn/wiki/images/blendermeshsetup.png](https://mogens.googlecode.com/svn/wiki/images/blendermeshsetup.png) |
|:------------------------------------------------------------------------------------------------------------------------------------------|

This section, as seen in Fig. 4, determines the appearance of the animated character. In addition, you can choose to ignore outer joints. The outer joints, namely fingers and toes, are often noisy due to the way marker based motion capture is done. Enabling _Ignore outer joints_ will make the motion appear smoother in those cases.

### Camera setup ###

**Fig. 5 The camera setup section of the Blender UI**
| ![https://mogens.googlecode.com/svn/wiki/images/blendercamerasetup.png](https://mogens.googlecode.com/svn/wiki/images/blendercamerasetup.png) |
|:----------------------------------------------------------------------------------------------------------------------------------------------|

As seen in Fig. 5, this section has quite a lot of options compared to the others.

This section determines the number and placement of cameras. When the _Automatic setup_ button is selected, you can choose the number of cameras with the slider labeled _Number of cameras_. The cameras will be placed evenly along the latitude ([0, pi/2>) given by the _Latitude_ slider next to it, and at a distance given by the _Radius_ slider further down. All these cameras will point towards origo, and later when a simulation is loaded, they will point towards the center of the motion.

With the next button row, ceiling cameras can be added. These cameras will be placed above the other cameras, and placed evenly along a given latitude. The latitude is given with the slider labeled _Ceiling latitude_ which will appear as soon as at least one ceiling camera has been added. The ceiling cameras’ placement will also be affected by the _Radius_ slider. The main difference between the ceiling cameras and the other cameras is that the ceiling cameras will point directly downwards (negative _z_-direction) instead of towards the center.

After setting the values described above, you can move the cameras to different positions in the 3D view. There’s a convenience button labeled _Select all cameras_ that will select all cameras in the current scene.

If you wish to store the current camera setup you’ve made, click the _Export current setup_ button to get a save dialog. The camera setup will be saved in an xml-format that can be imported again later.

To import a previously saved camera setup, select the _Pre-saved setup_ button. The camera selection buttons will be replaced by a button to load a pre-saved camera setup file and a label showing the currently loaded file (if any).

### Render setup ###

**Fig. 6 The render setup section of the Blender UI**
| ![https://mogens.googlecode.com/svn/wiki/images/blenderrendersetup.png](https://mogens.googlecode.com/svn/wiki/images/blenderrendersetup.png) |
|:----------------------------------------------------------------------------------------------------------------------------------------------|

In the Render setup section, as seen in Fig. 6, you specify original FPS and output FPS. Original FPS is the framerate of the motion you’ve loaded. Most of the motion capture data from the CMU database is at 120 FPS, but some are lower. Output FPS determines how many frames to render per second.

### Output setup ###


**Fig. 7 The output setup section of the Blender UI**
| ![https://mogens.googlecode.com/svn/wiki/images/blenderoutputsetup.png](https://mogens.googlecode.com/svn/wiki/images/blenderoutputsetup.png) |
|:----------------------------------------------------------------------------------------------------------------------------------------------|

In the Output setup section, as seen in Fig. 7, you specify what folder/directory should be used to store the rendered silhouette images, the joint position files and the file containing camera positions. One subdirectory will be created for each camera, and the images rendered by each camera will be placed in it’s respective subdirectory. I addition, one <tt>calibration.xml</tt> file will be generated that contains the positions of the cameras in the scene, and a <tt>jointPositions-<armature name>.xml</tt> will be generated for each imported armature, containing the joint positions of each armature. In both these files, the _z_-axis will be pointing up.

### Load simulation og Render scene ###

**Fig. 8 The output setup section of the Blender UI**
| ![https://mogens.googlecode.com/svn/wiki/images/blenderloadandrender.png](https://mogens.googlecode.com/svn/wiki/images/blenderloadandrender.png) |
|:--------------------------------------------------------------------------------------------------------------------------------------------------|

At the bottom of the user interface you will find a button row consisisting of one or two buttons. Initally only the _Load simulation_ button will be visible. Pressing this button will load the skeleton and motion(s) given with the ASF and AMC files, and the mesh selected under Mesh Setup. Once loaded you can inspect the avatar and its motion using Blender’s 3D view and Timeline windows.

When the simulation is loaded, the _Render scene_ button becomes visible. Pressing this will start the rendering of silhouette images. Fig. 8 shows how this section looks before the simulation has been loaded.

## Using the graph module ##

**Fig. 9 Empty MoGrapher GUI**
| ![https://mogens.googlecode.com/svn/wiki/images/mographerstart.png](https://mogens.googlecode.com/svn/wiki/images/mographerstart.png) |
|:--------------------------------------------------------------------------------------------------------------------------------------|

To run the graph module, run the <tt>mographer.py</tt> Python script using Python. The window you see in Fig. 9 will appear. This user interface can be seen as consisting of four main sections; The top, the center, the bottom and the right section. The contents of each section are explained below.

### The top section ###

**Fig. 10 Top section of the MoGrapher GUI**
| ![https://mogens.googlecode.com/svn/wiki/images/mographertop.png](https://mogens.googlecode.com/svn/wiki/images/mographertop.png) |
|:----------------------------------------------------------------------------------------------------------------------------------|

In the top section, as shown in Fig. 10, you specify the data files and the graphtype you wish to view. The ground truth file will typically be the joint position file generated by the Blender module, and the candidate file(s) will be the results of your algorithms.

Under _Select ground truth file_, you set the ground truth file by going to <tt>File->Open ground truth file...</tt>, or clicking the button labeled _..._ just next to the textbox. A standard file dialog will open up, allowing you to select the XML file. Once selected, the file will be opened and processed. This may take a few seconds.

Under _Select candidate file_, you can select a candidate file to compare with the ground truth file. The dropdown box will be empty until you add some candidate files. To add candidate files, go to <tt>File->Open candidate files...</tt> or click the button labeled _..._ just next to the dropdown box. A standard file dialog will open up, allowing you to select one or more XML files. Once selected, the file(s) will be opened and processed. This may take a few seconds. When the processing is complete, the files will be added as choices in the dropdown box. You can also empty the dropdown box by going to <tt>File->Clear candidates</tt>.

Under _Select graphtype_, you select the type of graphs that should be shown. The choices are to see the euclidian distance between the joints of the ground truth, and the joints of your algorithm, and then three more choices of showing only the difference in _x_, _y_ and _z_ directions.

Once a ground truth file, a candidate file, and graph type is selected, you can click the button labeled _Generate_ to generate the graphs. This may take a while, and checkboxes will appear in the right section of the user interface.

TODO: box and whisker plots.

### The center section ###

**Fig. 11 Center section of the MoGrapher GUI**
| ![https://mogens.googlecode.com/svn/wiki/images/mographercenter.png](https://mogens.googlecode.com/svn/wiki/images/mographercenter.png) |
|:----------------------------------------------------------------------------------------------------------------------------------------|

This section houses the main component of MoGrapher; the canvas. The canvas consists of a large white box with axes, where the actual graphs will be plotted, a smaller white box on the lower right where the average and standard deviation across is shown, and a legend in the upper right, which becomes visible when graphs become visible. Fig. 11 shows the center section of the MoGrapher GUI. The slider on the right hand side is explained in the next section.

### The bottom section ###

In Fig. 12 you see the bottom section. Here you will find tools to alter the axes of the graph.

Adjusting the slider just below the canvas will zoom in and out on the _x_-axis. Similarly there’s a vertical slider just to the right of the canvas which does the same for the _y_-axis. At the bottom right, there are four textboxes where you can specify the limits of the axes manually.

**Fig. 12 Bottom section of the MoGrapher GUI**
| ![https://mogens.googlecode.com/svn/wiki/images/mographerbottom.png](https://mogens.googlecode.com/svn/wiki/images/mographerbottom.png) |
|:----------------------------------------------------------------------------------------------------------------------------------------|

At the bottom left you’ll find four image buttons. The button with an image of a house on it will reset the axes to the values they were after the graphs were generated. If you click the button with an image of a cross, you can afterwards click and drag in the canvas to move the graphs around. If you click the button with an image of a paper and a magnifying glass, you can afterwards click and drag a box in the canvas to zoom in on that area.

Lastly there’s a button with an image of a diskette on it. If you press this, a standard file dialog will appear, allowing you to save the graph currently viewed in the canvas to a file in one of several formats. You can also save the graph by going to <tt>File->Save as...</tt>.

### The right section ###

**Fig. 13 Right section of the MoGrapher GUI**
| ![https://mogens.googlecode.com/svn/wiki/images/mographerright.png](https://mogens.googlecode.com/svn/wiki/images/mographerright.png) |
|:--------------------------------------------------------------------------------------------------------------------------------------|

The section to the right will initially be empty. Once you’ve clicked _Generate_, this section will be populated by two columns of checkboxes, one checkbox for each joint, plus one for average, one for standard deviation and one for average standard deviation. Fig. 13 shows how this may look when checkboxes has been added. These checkboxes will initially be unselected. When you select one, the corresponding graph will be plotted on the canvas, and deselecting one will make it disappear again. Each graph will be plotted in one of seven colors. When there are seven graphs currently visible, additional graphs will reuse these colors.

# Known issues #

This system has some known issues that, mainly due to limited time, could not be fixed by the deadline. Following is a list of the bugs and limitations we are aware of. The initials BM and GM specifies if the bug is related to the Blender module or the Graph module respectively.

  * BM: Only mocap data from CMU is supported. The CMU database has a vast amount of motion capture data, and we’ve only tested the system with data from this database. Other motion capture data may still work.
  * BM: The rendered data will only be as long as the last imported motion. If you load and render two motions in the Blender module, the rendered data will only be as long as the motion that got loaded last. E.g. if you load a motion of 600 frames, then a motion of 500 frames, the resulting rendered data will only consist of the first 500 frames; the last 100 frames of the first motion will be lost.
  * BM: Automatic camera positioning. For automatic camera setups, the cameras will be moved to point at the center of the last imported motion. This also applies to the last motion imported during motion stitching.
  * BM: Motion stictching limited to five motions. This is mainly due to the limited space of the Blender module’s user interface. A more suitable user interface component should be made to handle an arbitrary ammount without making the user interface too large.
  * BM: Only Blender 2.49 is supported. At the time of this writing, the 2.5x series of Blender is under development and the 2.5x API is currently unstable, so making the Blender module 2.5x compatible was not a viable option. Once 2.5x is stable, the Blender module will likely need changes.
  * GM: Problems running the graph module on Windows. There appears to be an issue with the Tcl/Tk build on Windows. We haven’t managed to figure out what exactly makes it fail.
  * GM: Joint names aren’t sorted. When graphs have been generated in the graph module, the checkboxes in the right section will appear in arbitrary order. This makes it cumbersome to find and select the joints you are interested in.
  * GM: In the save dialog on Mac OSX, filetype must be typed in manually. When saving the graph on Mac OSX, the save dialog does not list the possible fileformats you can save as. The graph module uses file dialogs native to the system, and OSX’s file dialog unfortunately does not have this feature.
  * GM: Basenames used as keys. When reading and parsing files, the parsed data will be identified by the basename of each file. That means if two files in different locations have the same filename, only the last one will be stored.
  * GM: Calculated mean and stdev for entire sequence are currently output to terminal. A new message text area should be added for these values to the right section of the UI.
  * GM: The check boxes used for selecting curves are inelegant, and should be replaced by a list view.
  * GM: The x-axis does not reset properly after generating a box plot.
  * GM: Rendering of graph legend is currently disabled. A check box for turning this on/off should be added.
  * GM: Errors in calculation of mean and stdev for entire sequence have been fixed. Calculations for each frame should be checked thoroughly.
  * GM: There is a memory leak somewhere.