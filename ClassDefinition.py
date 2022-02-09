import numpy as np
from random import random


class Line:
    """ Defines the line class """

    def __init__(self, p0, p1):
        self.p = np.array(p0)  # start point
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)  # distance between the end and start point
        self.dirn /= self.dist  # normalized vector from p0 to p1

    def path(self, t):
        """ Returns a point along the line """

        return self.p + t * self.dirn


class Graph:
    """ Defines the graph class """

    def __init__(self, startpos, endpos):
        """ Initializes the tree """
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos: 0}  # Creates a dictionary associating a vextex position to its Id
        self.neighbors = {0: []}  # Creates a dictionary associating an Id to its neighbors Id
        self.distances = {0: 0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]
        self.sz = endpos[2] - startpos[2]

    def add_vex(self, pos):
        """
        Returns the Id associated to pos if already in the graph, otherwise adds it to the graph and returns the newly created Id
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

    def randomPosition(self):
        """ Shoots a random vertex within the window search """
        rx = random()
        ry = random()
        rz = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2  # This ensures that posx is within the window
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2  # Same for posy
        posz = self.startpos[2] - (self.sz / 2.) + rz * self.sz * 2  # Same for posz
        return posx, posy, posz
