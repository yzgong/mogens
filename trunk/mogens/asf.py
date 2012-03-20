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
from math import radians

try:
    from numpy import matrix, cos, sin, zeros, array, asmatrix
    use_numpy = True
except ImportError:
    use_numpy = False

class Bone(object):
    """
    A class representing a bone from an ASF file
    """
    def __init__(self):
        self.id = 0
        self.name = None
        self.direction = [ 0., 0., 0., ]
        self.length = 0
        self.axis = [ 0, 0, 0, ]
        self.axisorder = None
        self.children = []
        self.parent = None
        self.dof = []
        self.limits = []
        self.axis_rot = None
        self.axis_rot_inv = None
        self.bone_rest_matrix = None
        self.bone_rest_matrix_inv = None
        self.offset_mat = None

        # only root bone
        self.order = None
        self.position = [ 0, 0, 0, ]
        self.orientation = [ 0, 0, 0 ]

    def _calc_matrices_numpy(self):
        # See the final report of the project for why this is needed.
        raxis = map(radians, self.axis)
        cx, cy, cz = cos(raxis)
        sx, sy, sz = sin(raxis)
        self.axis_rot = array([
              [cy*cz, -cx*sz + sx*sy*cz,  sx*sz + cx*sy*cz, 0.],
              [cy*sz,  cx*cz + sx*sy*sz, -sx*cz + cx*sy*sz, 0.],
              [-sy,    sx*cy,             cx*cy           , 0.],
              [0.,     0.,                0.,               1.],
              ])
        self.axis_rot_inv = asmatrix(self.axis_rot).T
        self.offset_mat = array([
            [1., 0., 0., self.length * self.direction[0]],
            [0., 1., 0., self.length * self.direction[1]],
            [0., 0., 1., self.length * self.direction[2]],
            [0., 0., 0., 1.]
        ])
        return True

    def calc_matrices(self):
        """ Calculate axis and offset matrices. """
        if use_numpy:
            return self._calc_matrices_numpy()
        
    def __str__(self):
        """ Generates a multiline string representation of this Bone object.
        Mainly just for debuging."""
        res = ""
        if self.name == "root":
            res += "root:\n"
            res += "  order: %s\n" % (" ".join(self.order),)
            res += "  axisorder: %s\n" % (self.axisorder,)
            res += "  position: %6f %6f %6f\n" % (
                self.position[0], self.position[1], self.position[2],
            )
            res += "  orientation: %6f %6f %6f\n" % (
                self.orientation[0], self.orientation[1], self.orientation[2],
            )
        else:
            res += "%s:\n" % (self.name,)
            res += "  direction: %6f %6f %6f\n" % (
                self.direction[0], self.direction[1], self.direction[2],
            )
            res += "  length: %6f\n" % (self.length,)
            res += "  axis: %6f %6f %6f\n" % (
                self.axis[0], self.axis[1], self.axis[2],
            )
            res += "  axisorder: %s\n" % (self.axisorder,)
            res += "  dof: %s\n" % (" ".join(self.dof),)
            res += "  limits: %s\n" % (str(self.limits),)
            res += "  parent: %s\n" % (self.parent.name,)
        
        res += "  children: %s\n" % (" ".join(x.name for x in self.children),)
        return res
    def __repr__(self):
        return self.__str__()


class ASF(object):
    """ ASF class for parsing data from an ASF file. """
    def __init__(self):
        self.version = None
        self.name = None
        self.units = {}
        self.documentation = []
        self.root = Bone()
        self.bones = {}
        self.hierarchy = None

    def parse(self, filename):
        """ Opens and parses the ASF file given by filename."""
        file = open(filename)
        read_units = False
        read_documentation = False
        read_root = False
        read_bonedata = False
        read_hierarchy = False
        read_limits = False
        bone = None
        i = 0
        for line in file:
            i = i + 1
            if line.startswith("#"):
                continue
            elif line.startswith(":"):
                read_units = False
                read_documentation = False
                read_root = False
                read_bonedata = False
                read_hierarchy = False
                read_limits = False

                tok = line[1:].split()
                if tok[0] == "version":
                    self.version = float(tok[1])
                elif tok[0] == "name":
                    self.name = tok[1]
                elif tok[0] == "units":
                    read_units = True
                elif tok[0] == "documentation":
                    read_documentation = True
                elif tok[0] == "root":
                    self.root.name = "root"
                    read_root = True
                elif tok[0] == "bonedata":
                    read_bonedata = True
                elif tok[0] == "hierarchy":
                    read_hierarchy = True
            elif read_units:
                tok = line.split()
                if len(tok) != 2:
                    print >> sys.stderr, "%d: Invalid data\n" % (i,)
                    raise
                self.units[tok[0]] = tok[1]
                continue
            elif read_documentation:
                self.documentation.append(line.strip())
            elif read_root:
                tok = line.split()
                if tok[0] == "order":
                    self.root.order = tok[1:]
                elif tok[0] == "axis":
                    self.root.axisorder = tok[1]
                elif tok[0] == "position":
                    self.root.position = map(float, tok[1:])
                elif tok[0] == "orientation":
                    self.root.orientation = map(float, tok[1:])
                else:
                    print >>sys.stderr, "%d: Invalid data\n" % (i,)
                    raise
            elif read_bonedata:
                if bone is None:
                    if line.strip() != "begin":
                        print >>sys.stderr, "%d: Excpected begin\n" % (i,)
                        raise
                    bone = Bone()
                    continue
                if line.strip() == "end":
                    read_limits = False
                    bone = None
                    continue
                tok = line.split()
                if tok[0] == "id":
                    bone.id = int(tok[1])
                elif tok[0] == "name":
                    bone.name = tok[1]
                    self.bones[bone.name] = bone
                elif tok[0] == "direction":
                    bone.direction = map(float, tok[1:])
                elif tok[0] == "length":
                    bone.length = float(tok[1])
                elif tok[0] == "axis":
                    bone.axis = map(float,tok[1:4])
                    bone.axisorder = tok[4]
                elif tok[0] == "dof":
                    bone.dof = tok[1:]
                elif tok[0] == "limits":
                    read_limits = True
                    bone.limits.append( (tok[1].strip('('), tok[2].strip(')')) )
                elif read_limits:
                    bone.limits.append( (tok[0].strip('('), tok[1].strip(')')) )
                else:
                    print >>sys.stderr, "%d: Invalid data\n" % (i,)
                    raise
            elif read_hierarchy:
                if self.hierarchy is None:
                    if line.strip() != "begin":
                        print >>sys.stderr, "%d: Excpected begin\n" % (i,)
                        raise
                    self.hierarchy = {}
                    continue
                tok = line.split()
                if tok[0] == "end":
                    break
                if tok[0] == "root":
                    for name in tok[1:]:
                        child = self.bones[name]
                        self.root.children.append(child)
                        child.parent = self.root
                else:
                    node = self.bones[tok[0]]
                    for name in tok[1:]:
                        child = self.bones[name]
                        node.children.append(child)
                        child.parent = node
        file.close()

    def calc_matrices(self):

        """
        Call this after parse() has been called to also calculate axis
        and offset matrices for each bone.
        """

        self.root.calc_matrices()
        for bone in self.bones.values():
            bone.calc_matrices()



def print_hierarchy(root):
    """ Print a representation of an ASF object's hierarchy. Mainly for
    debugging"""
    print root
    for child in root.children:
        print_hierarchy(child)

def main():
    for arg in sys.argv[1:]:
        print "Parsing «%s»" % (arg,)
        asf = ASF()
        asf.parse(arg)
        asf.calc_matrices()
        print_hierarchy(asf.root)
        print "-"*20, "end",arg,"-"*20

if __name__ == "__main__":
    main()
