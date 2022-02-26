u"""
This code is an attempt at implementing a 3D version of the previously coded RRT-start by Fanjin Zeng.
https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80

Bendjilali Moussa 2022.
"""

from __future__ import division
from __future__ import absolute_import
from ClassDefinition import *


def intersection(line, center, radius):
    u""" Checks line-sphere (circle) intersection. Works in 3D."""

    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True


def intersectionCube(line, center, radius):
    u"""
    Checks line-cube intersection.
    :param line: Line
    :param center: Center of the cube
    :param radius: Radius of a cube using L1 norm (half of its size)
    :return: True or False depending on whether or not the line crosses the cube
    """
    x0, y0, z0 = line.p
    x1, y1, z1 = line.p1
    xc, yc, zc = center
    if line.dist < 1e-10:
        point = line.p
    else:
        t = -((z0 - zc) * (z1 - z0) + (y0 - yc) * (y1 - y0) + (x0 - xc) * (x1 - x0)) / (line.dist ** 2)
        point = line.ray(t * np.linalg.norm(line.p1 - line.p))
    if center[0] - radius < point[0] < center[0] + radius and \
            center[1] - radius < point[1] < center[1] + radius and \
            center[2] - radius < point[2] < center[2] + radius:
        return True
    return False


def distance(p1, p2):
    u"""Computes the distance between two points"""

    return np.linalg.norm(np.array(p1) - np.array(p2))


def isInObstacle(vex, obstacles, radius):
    u""" Returns True if the point vex is too close (radius) to an obstacle"""

    for obs in obstacles:
        if obs[0] - radius < vex[0] < obs[0] + radius and \
                obs[1] - radius < vex[1] < obs[1] + radius and \
                obs[2] - radius < vex[2] < obs[2] + radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    u""" Returns True if the point vex is located in an obstacle"""

    for obs in obstacles:
        #  if Intersection(line, obs, radius):
        if intersectionCube(line, obs, radius):
            return True
    return False


def nearest(G, vex, obstacles, radius):
    u"""Returns the closest visible vertex in G to vex"""

    Nvex = None
    Nidx = None
    minDist = float(u"inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, radius):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    u""" Shoots a new vertex between randvex and nearvex """

    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min(stepSize, length)  # This ensures that the shot vertex lies between randvex and nearvex
    newvex = (nearvex[0] + dirn[0], nearvex[1] + dirn[1], nearvex[2] + dirn[2])
    if newvex == randvex:
        return newvex, u'Reached'
    else:
        return newvex, u'Advanced'


def window(startpos, endpos):
    u""" Defines search window. This version works in 3D. """

    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    depth = endpos[2] - startpos[2]
    winx = startpos[0] - (width / 3.)
    winy = startpos[1] - (height / 3.)
    winz = startpos[2] - (depth / 3.)
    return winx, winy, winz, width, height, depth


def isInWindow(pos, winx, winy, winz, width, height, depth):
    u""" Restricts new vertex inside the search window. This version works in 3D. """

    if winx < pos[0] < winx + width and \
            winy < pos[1] < winy + height and \
            winz < pos[2] < winz + depth:
        return True
    else:
        return False
