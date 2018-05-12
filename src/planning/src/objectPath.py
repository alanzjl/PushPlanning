#!/usr/bin/python
#########################################################################
# File Name: objectPath.py
# Description: 
# Author: Jialiang Zhao
# Mail: alanzjl@163.com
# Created_Time: 2018-04-10 21:47:00
# Last modified: 2018-04-10 21:47:1523422020
#########################################################################

import numpy as np
from math import pi, sin, cos, asin, acos, atan, atan2, sqrt
from matplotlib import pyplot as plt

INFI = 10000.

class ObjectPath:
    @property
    def target_pose(self):
        raise NotImplementedError()

    @property
    def total_length(self):
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()

    @property
    def is_finished(self):
        raise NotImplementedError()

class ArcObjectPath(ObjectPath):
    def __init__(self, radius, angle, left_turn)
        self.radius = radius
        self.angle = angle
        self.left_turn = left_turn
        self.length = abs((angle / (2*pi)) * (2. * pi * radius))
        self.is_finished = False

    def finish(self):
        self.is_finished = True

    @property
    def target_pos(self):
        return [self.raidus, self.angle, self.left_turn]

    @property
    def total_length(self):
        return self.length

    @property
    def is_finished(self):
        return self.is_finished
    
class ChainObjectPath(ObjectPath):
    def __init__(self, subpaths):
        self.subpaths = subpaths
        self.currentpath = 0
        self.is_finished = False if len(subpaths) > 0 else True

    def finish(self):
       self.subpaths[self.currentpath].finish()
       self.currentpath += 1
       if self.currentpath > len(self.subpaths) - 1:
           self.is_finished = True

    @property
    def target_pos(self):
        return self.subpaths[self.currentpath].target_pos
    
    @property
    def total_length(self):
        l = 0
        for i in self.subpaths:
            l += i.total_length
        return l

    @property
    def is_finished(self):
        return self.is_finished

