#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  robot.py
#  
#  Copyright 2016 Topfpflanze <Topfpflanze@WINTERGARTEN>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

import math
from sympy import *


class Robot():
	
	# set the properties of the default robot
	# flLink1 will be considered connected to topLink1 and so on...
	def __init__(self, flLink1,flLink2,flLink3,flLink4,flLink5,flLink6, topLink1,topLink2,topLink3,topLink4,topLink5,topLink6):
		self.initFloorPlate(flLink1,flLink2,flLink3,flLink4,flLink5,flLink6)
		self.initTopPlate(topLink1,topLink2,topLink3,topLink4,topLink5,topLink6)
	
	# u: 1st coordinate of the floor plate
	# v: 2nd coordinate
	# w: 3rd coordinate == 0 as all points ly in a plane
	def initFloorPlate(self, link1 = [0,0], link2 = [0,0], link3 = [0,0], link4 = [0,0], link5 = [0,0], link6 = [0,0]):
		self.u1 = link1[0]
		self.v1 = link1[1]
		
		self.u2 = link2[0]
		self.v2 = link2[1]
		
		self.u3 = link3[0]
		self.v3 = link3[1]
		
		self.u4 = link4[0]
		self.v4 = link4[1]
		
		self.u5 = link5[0]
		self.v5 = link5[1]
		
		self.u6 = link6[0]
		self.v6 = link6[1]
		
	# x: 1st coordinate of the top plate
	# y: 2nd coordinate
	# z: 3rd coordinate == 0 as all points ly in a plane
	def initTopPlate(self, link1 = [0,0], link2 = [0,0], link3 = [0,0], link4 = [0,0], link5 = [0,0], link6 = [0,0]):
		self.x1 = link1[0]
		self.y1 = link1[1]
		self.z1 = 0
		
		self.x2 = link2[0]
		self.y2 = link2[1]
		self.z2 = 0
		
		self.x3 = link3[0]
		self.y3 = link3[1]
		self.z3 = 0
		
		self.x4 = link4[0]
		self.y4 = link4[1]
		self.z4 = 0
		
		self.x5 = link5[0]
		self.y5 = link5[1]
		self.z5 = 0
		
		self.x6 = link6[0]
		self.y6 = link6[1]
		self.z6 = 0
		
		
		
	def getFloorPlate(self):
		return [[self.u1, self.v1, 0], [self.u2, self.v2, 0], [self.u3, self.v3, 0], [self.u4, self.v4, 0], [self.u5, self.v5, 0], [self.u6, self.v6, 0]]

	def getTopPlate(self):
		return [[self.x1, self.y1, self.z1], [self.x2, self.y2, self.z2], [self.x3, self.y3, self.z3], [self.x4, self.y4, self.z4], [self.x5, self.y5, self.z5], [self.x6, self.y6, self.z6]]

	def getRobot(self):
		return [self.getFloorPlate(), self.getTopPlate()]
	
	# apllies a translation and afterwards a rotation
	# translation = [x,y,z], rotation = [psi, theta, phi]
	# angles are given in radians
	def setMotion(self, translation = [0,0,0], rotation = [0,0,0]):
		self.x1 += translation[0]
		self.y1 += translation[1]
		self.z1 += translation[2]
		
		self.x2 += translation[0]
		self.y2 += translation[1]
		self.z2 += translation[2]
		
		self.x3 += translation[0]
		self.y3 += translation[1]
		self.z3 += translation[2]
		
		self.x4 += translation[0]
		self.y4 += translation[1]
		self.z4 += translation[2]
		
		self.x5 += translation[0]
		self.y5 += translation[1]
		self.z5 += translation[2]
		
		self.x6 += translation[0]
		self.y6 += translation[1]
		self.z6 += translation[2]
		
		psi = rotation[0]
		phi = rotation[1]
		theta = rotation[2]
		
		
		M11 = math.cos(phi)*math.cos(psi)-math.sin(psi)*math.sin(phi)*math.cos(theta)
		M12 = -math.cos(psi)*math.sin(phi)-math.sin(psi)*math.cos(phi)*math.cos(theta)
		M13 = math.sin(psi)*math.sin(theta)
		
		M21 = math.sin(psi)*math.cos(phi)+math.cos(psi)*math.sin(phi)*math.cos(theta)
		M22 = -math.sin(psi)*math.sin(phi)+math.cos(psi)*math.cos(phi)*math.cos(theta)
		M23 = -math.cos(psi)*math.sin(theta)
		
		M31 = math.sin(phi)*math.sin(theta)
		M32 = math.cos(phi)*math.sin(theta)
		M33 = math.cos(theta)
	
		rotMatrix = Matrix(((M11,M12,M13),(M21,M22,M23),(M31,M32,M33)))
		linkMatrixTransp = Matrix(((self.x1, self.y1, self.z1), (self.x2, self.y2, self.z2), (self.x3, self.y3, self.z3), (self.x4, self.y4, self.z4), (self.x5, self.y5, self.z5), (self.x6, self.y6, self.z6)))
		linkMatrix = linkMatrixTransp.T
		linkMatrix = rotMatrix*linkMatrix
		
		self.x1, self.y1, self.z1 = linkMatrix.col(0)
		self.x2, self.y2, self.z2 = linkMatrix.col(1)
		self.x3, self.y3, self.z3 = linkMatrix.col(2)
		self.x4, self.y4, self.z4 = linkMatrix.col(3)
		self.x5, self.y5, self.z5 = linkMatrix.col(4)
		self.x6, self.y6, self.z6 = linkMatrix.col(5)

	# test: all distances of links must stay the same. e.g. sqrt(||L1-L2||) must be constant. Ir is independable of translations.
	# seems to work except of small rounding errors.
	def getDistances(self):
		d1 = math.sqrt((self.x1-self.x2)**2 + (self.y1-self.y2)**2 + (self.z1-self.z2)**2)
		d2 = math.sqrt((self.x3-self.x2)**2 + (self.y3-self.y2)**2 + (self.z3-self.z2)**2)
		d3 = math.sqrt((self.x4-self.x2)**2 + (self.y4-self.y2)**2 + (self.z4-self.z2)**2)
		return [d1,d2,d3]
	
	
def main(args):
	sqrt3halve = math.sqrt(3)/2.0
	# default robot
	R = Robot([1,0],[-0.5,sqrt3halve],[-0.5, sqrt3halve],[-0.5,-sqrt3halve],[-0.5,-sqrt3halve],[1,0], [0.5,sqrt3halve],[0.5,sqrt3halve],[-1,0],[-1,0],[-0.5,sqrt3halve],[-0.5,sqrt3halve])
	R.setMotion([1,1,1], [1,2,3])
	R.setMotion([0,0,0], [2,4,6])
	#print(R.getRobot())
	return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
