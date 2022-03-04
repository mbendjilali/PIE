"""
This code is an attempt at implementing a 3D version of the previously coded RRT-start by Fanjin Zeng
(https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80) using a polygonal representation of the obstacles.

It is parted in 5 different Python files :
ClassDefinition.py -> Defines the most basic structures needed to implement an RRT algorithm, i.e. what a graph,
a line and a window are.
Geometry.py        -> Sets up collision criteria for objects in a 3D space (spheres, cubes, and triangles).
PathPlanning.py    -> Implements the RRT-Star and RRT-Connect (currently not working) algorithms. It also retrieves
the correct path_list from start to goal within a successful tree.
meshRecon.py       -> Uses the Alpha Shape algorithm to generate a triangle set from a point cloud.
main.py            -> Sets up every parameter, loads a point cloud, derives the RRT algorithm and plots the result.

By Bendjilali Moussa 2022. """

import numpy as np
from random import random


# ------------------------------ Class Definition -----------------------------------


class Window:
    """ Defines a search window"""

    def __init__(self, start, end):
        width = end[0] - start[0]
        height = end[1] - start[1]
        depth = end[2] - start[2]
        self.minx = min(start[0], end[0]) - width / 3.
        self.miny = min(start[1], end[1]) - height / 3.
        self.minz = min(start[2], end[2]) - depth / 3.
        self.maxx = max(start[0], end[0]) + width / 3.
        self.maxy = max(start[1], end[1]) + height / 3.
        self.maxz = max(start[2], end[2]) + depth / 3.

    def randomPosition(self):
        """ Shoots a random vertex within the window search """
        posx = random() * (self.maxx - self.minx)
        posy = random() * (self.maxy - self.miny)
        posz = random() * (self.maxz - self.minz)

        return posx, posy, posz


class Line:
    """ Defines the line class """

    def __init__(self, p0, p1):
        self.p = np.array(p0)  # start point
        self.p1 = np.array(p1)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)  # distance between the end and start point
        self.dirn /= self.dist  # normalized vector from p0 to p1

    def ray(self, t):
        """ Returns a point along the line """

        return self.p + t * self.dirn


class Graph:
    """ Defines the graph class """

    def __init__(self, start, end):
        """ Initializes the tree """
        self.start = start
        self.end = end

        self.vertices = [start]
        self.edges = []
        self.success = False

        self.vex2idx = {start: 0}  # Creates a dictionary associating a vextex position to its idx
        self.neighbors = {0: []}  # Creates a dictionary associating an idx to its neighbors idx
        self.distances = {0: 0.}

        self.sx = end[0] - start[0]
        self.sy = end[1] - start[1]
        self.sz = end[2] - start[2]

    def add_vex(self, pos):
        """
        Returns the idx associated to pos if already in the graph, otherwise adds it to the graph and returns the
        newly created idx
        """

        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        """ Adds the edge between idx1 and idx2 in the graph """
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))
