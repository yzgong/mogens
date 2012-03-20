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

import sys
from asf import ASF

class Vec3(object):
    """ A simple struct to hold three values, x, y and z. """
    def __init__(self, x=0.0, y=0.0, z=0.0):
        if type(x) == list:
            self.x, self.y, self.z = x
        else:
            self.x = x
            self.y = y
            self.z = z
    def __str__(self):
        return "[%6f, %6f, %6f]" % (self.x, self.y, self.z)
    def __repr__(self):
        return "[%6f, %6f, %6f]" % (self.x, self.y, self.z)

class Frame(object):
    """ A simple struct to represent a frame. """
    def __init__(self):
        self.number = 0
        self.bones = {}
        self.root_pos = Vec3()
        self.root_rot = Vec3()
        

class AMC(object):
    """
    The AMC class is used to parse and store motion from an AMC file.

    asf is an ASF object containing an skeleton parsed from an ASF file.
    """

    def __init__(self, asf):
        self.asf = asf
        self.use_degrees = False
        self.frames = {}
        pass
    def parse(self, filename):
        """
        Parses and stores the data from the amc file given by filename.
        """
        frame = 0
        file = open(filename)
        i = 0
        for line in file:
            i = i + 1
            if line.startswith("#"):
                continue
            elif line.startswith(":"):
                tok = line[1:].split()
                if tok[0] == "FULLY-SPECIFIED":
                    continue
                elif tok[0] == "DEGREES":
                    self.use_degrees = True
                else:
                    print >>sys.stderr, "%d: Unkown token: %s\n", (i,line,)
                    raise
            elif line.strip().isdigit():
                frame = Frame()
                frame.number = int(line.strip())
                self.frames[frame.number] = frame
            else:
                tok = line.split()
                if tok[0] == "root":
                    for type, value in map(None, self.asf.root.order, tok[1:]):
                        if   type == 'TX': frame.root_pos.x = float(value)
                        elif type == 'TY': frame.root_pos.y = float(value)
                        elif type == 'TZ': frame.root_pos.z = float(value)
                        elif type == 'RX': frame.root_rot.x = float(value)
                        elif type == 'RY': frame.root_rot.y = float(value)
                        elif type == 'RZ': frame.root_rot.z = float(value)
                elif tok[0] not in self.asf.bones:
                    print >>sys.stderr, "%d: Unkown bone: %s\n", (i,line,)
                    raise
                else:
                    name = tok[0]
                    frame.bones[name] = Vec3()
                    for type, value in map(None, self.asf.bones[name].dof, tok[1:]):
                        if   type == 'rx': frame.bones[name].x = float(value)
                        elif type == 'ry': frame.bones[name].y = float(value)
                        elif type == 'rz': frame.bones[name].z = float(value)

                
def main():
    global asf
    global amc
    asf = ASF()
    asf.parse(sys.argv[1])
    amc = AMC(asf)
    amc.parse(sys.argv[2])

if __name__ == '__main__':
    main()
