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
		
		self.leg1FloorLink = Matrix([link1[0], link1[1], 0])
		self.leg2FloorLink = Matrix([link2[0], link2[1], 0])
		self.leg3FloorLink = Matrix([link3[0], link3[1], 0])
		self.leg4FloorLink = Matrix([link4[0], link4[1], 0])
		self.leg5FloorLink = Matrix([link5[0], link5[1], 0])
		self.leg6FloorLink = Matrix([link6[0], link6[1], 0])
		
		self.legFloorLinks = [self.leg1FloorLink, self.leg2FloorLink, self.leg3FloorLink, self.leg4FloorLink, self.leg5FloorLink, self.leg6FloorLink]
		
	# x: 1st coordinate of the top plate
	# y: 2nd coordinate
	# z: 3rd coordinate == 0 as all points ly in a plane
	def initTopPlate(self, link1 = [0,0], link2 = [0,0], link3 = [0,0], link4 = [0,0], link5 = [0,0], link6 = [0,0]):
		
		self.leg1TopPlateLink = Matrix([link1[0], link1[1], 0])
		self.leg2TopPlateLink = Matrix([link2[0], link2[1], 0])
		self.leg3TopPlateLink = Matrix([link3[0], link3[1], 0])
		self.leg4TopPlateLink = Matrix([link4[0], link4[1], 0])
		self.leg5TopPlateLink = Matrix([link5[0], link5[1], 0])
		self.leg6TopPlateLink = Matrix([link6[0], link6[1], 0])
		
		self.legTopPlateLinks = [self.leg1TopPlateLink, self.leg2TopPlateLink, self.leg3TopPlateLink, self.leg4TopPlateLink, self.leg5TopPlateLink, self.leg6TopPlateLink]
		
		
	def getFloorPlate(self):
		return self.legFloorLinks

	def getTopPlate(self):
		return self.legTopPlateLinks

	def getRobot(self):
		return [self.getFloorPlate(), self.getTopPlate()]
	
	# apllies a translation and afterwards a rotation
	# translation = [x,y,z], rotation = [psi, theta, phi]
	# angles are given in radians
	def setMotion(self, translation = [0,0,0], rotation = [0,0,0]):
		
		# append the translation
		self.leg1TopPlateLink += Matrix(translation)
		self.leg2TopPlateLink += Matrix(translation)
		self.leg3TopPlateLink += Matrix(translation)
		self.leg4TopPlateLink += Matrix(translation)
		self.leg5TopPlateLink += Matrix(translation)
		self.leg6TopPlateLink += Matrix(translation)
		
		# append the rotation
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
	
		rotMatrix = Matrix([[M11,M12,M13],[M21,M22,M23],[M31,M32,M33]])
		linkMatrix = self.leg6TopPlateLink
		linkMatrix = linkMatrix.col_insert(0, self.leg5TopPlateLink)

		linkMatrix = linkMatrix.col_insert(0, self.leg4TopPlateLink)
 
		linkMatrix = linkMatrix.col_insert(0, self.leg3TopPlateLink)

		linkMatrix = linkMatrix.col_insert(0, self.leg2TopPlateLink)

		linkMatrix = linkMatrix.col_insert(0, self.leg1TopPlateLink)
				
		linkMatrix = rotMatrix*linkMatrix
		
		self.leg1TopPlateLink = linkMatrix.col(0)
		self.leg2TopPlateLink = linkMatrix.col(1)
		self.leg3TopPlateLink = linkMatrix.col(2)
		self.leg4TopPlateLink = linkMatrix.col(3)
		self.leg5TopPlateLink = linkMatrix.col(4)
		self.leg6TopPlateLink = linkMatrix.col(5)

	# test: all distances of links must stay the same. e.g. sqrt(||L1-L2||) must be constant. It is independable of translations.
	# seems to work except of small rounding errors.
	# returns a Vecor with all the distances [link1-link1, link1-link2 ... link6-link6]
	def getDistances(self):
		
		results = []
		for link1 in self.legTopPlateLinks:
			for link2 in self.legTopPlateLinks:
				temp = link1-link2
				results.append(mpmath.norm(temp,2))
				
		
		return(Matrix(results))
	

		
	
def main(args):
	sqrt3halve = math.sqrt(3)/2.0
	# default robot
	R = Robot([1,0],[-0.5,sqrt3halve],[-0.5, sqrt3halve],[-0.5,-sqrt3halve],[-0.5,-sqrt3halve],[1,0], [0.5,sqrt3halve],[0.5,sqrt3halve],[-1,0],[-1,0],[-0.5,sqrt3halve],[-0.5,sqrt3halve])
	
	# measure the distance between the links and apply translations and rotations.
	# measure again.
	# the distances must be the same, if everything is properly implemented.
	d1 = R.getDistances()
	R.setMotion([1,0,0], [2,5,2])
	d2 = R.getDistances()
	R.setMotion([3,-1,4], [3,0,3])
	d3 = R.getDistances()
	
	# as expected all of the following entries are 0
	print(d1-d2)
	print(d2-d3)

	return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
