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

from ClassDefinition import *


# ------------------------------ Volume obstacles -----------------------------------

def distance(p1, p2):
    """Computes the distance between two points"""
    return np.linalg.norm(np.array(p1) - np.array(p2))


def intersection(line, center, radius):
    """ Checks line-sphere (circle) intersection. Works in 3D."""
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
    """
    Checks line-cube intersection
    :param line: Line
    :param center: Center of the cube
    :param radius: Radius of a cube using L1 norm (half of its size)
    :return: True or False depending on whether the line crosses the cube
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


def isInObstacle(vex, obstacles, radius):
    """ Returns True if the point vex is too close (rad) to a cubic obstacle"""
    for obs in obstacles:
        if obs[0] - radius < vex[0] < obs[0] + radius and \
                obs[1] - radius < vex[1] < obs[1] + radius and \
                obs[2] - radius < vex[2] < obs[2] + radius:
            return True

    return False


def isThruObstacle(line, obstacles, radius):
    """ Returns True if the point vex is located in an obstacle"""
    for obs in obstacles:
        #  if Intersection(line, obs, rad):
        if intersectionCube(line, obs, radius):
            return True

    return False


# ------------------------------ Surface obstacles ----------------------------------

def intersectionTriangle(line, triangle):
    """
    Checks Line-Triangle intersection using the Möller-Trumbore algorithm
    :param line: Line
    :param triangle: 3D positions of the triangle's edges
    :return: True or False depending on whether the line crosses the triangle
    """
    [v1, v2, v3] = triangle  # i_ème triangle = vertices[triangles[i]]
    eps = 1e-5
    e1 = v2 - v1
    e2 = v3 - v1
    point_vec = np.cross(line.dirn, e2)
    det = e1.dot(point_vec)

    if abs(det) < eps:
        return False
    inv_det = 1. / det
    vec = line.p - v1
    u = vec.dot(point_vec) * inv_det

    if u < 0. or u > 1:
        return False

    vec2 = np.cross(vec, e1)
    v = line.dirn.dot(vec2) * inv_det

    if v < 0. or u + v > 1.:
        return False

    t = e2.dot(vec2) * inv_det

    if t < eps:
        return False

    return True


def isThruTriangle(line, triangles):
    """Returns True if the point vex is hidden by a triangle"""
    for triangle in triangles[1]:
        if intersectionTriangle(line, triangles[0][triangle]):
            return True

    return False
