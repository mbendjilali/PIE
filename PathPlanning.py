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

from Geometry import *


# ------------------------------ RRT_STAR -----------------------------------


def nearest(G, vex, obstacles, radius=None):
    """Returns the closest visible vertex in g to vex"""

    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruTriangle(line, obstacles):
            # if isThruObstacle(line, obstacles, radius):
            continue
        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    """ Shoots a new vertex between randvex and nearvex """

    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min(stepSize, length)  # This ensures that the shot vertex lies between randvex and nearvex
    newvex = (nearvex[0] + dirn[0], nearvex[1] + dirn[1], nearvex[2] + dirn[2])
    if newvex == randvex:
        return newvex, 'Reached'
    else:
        return newvex, 'Advanced'


def RRT_star(startpos, endpos, obstacles, window, n_iter, stepSize, radius=None):
    """
    Implements the RRT Star algorithm
    :param startpos: starting position -> tuple of size 3
    :param endpos: goal position -> tuple of size 3
    :param obstacles: sets of triangles -> tuple of 2 arrays containing edges positions and idx
    :param window: search window
    :param n_iter: maximum number of iterations
    :param stepSize: minimum increment in the tree
    :param radius: obsolete parameter for polygonal representation of the obstacles
    :return: graph
    """

    # ---- INIT TREE
    G = Graph(startpos, endpos)

    for i in range(n_iter):

        if G.success:  # checks if the goal position is not already in the tree
            break

        # ---- RANDOM CONFIG
        randvex = window.randomPosition()
        # while isInObstacle(randvex, obstacles, rad):  # Obsolete for polygonal representation of the obstacles
        #     randvex = window.randomPosition()

        # ---- EXTEND
        nearvex, nearidx = nearest(G, randvex, obstacles,
                                   radius)
        if nearvex is None:  # If none is found, shoots a new randvex
            continue

        newvex, status = newVertex(randvex, nearvex, stepSize)
        dist = distance(newvex, nearvex)

        newidx = G.add_vex(newvex)
        G.add_edge(newidx, nearidx, dist)  # Adds a branch to the tree
        G.distances[newidx] = G.distances[nearidx] + dist  # Computes the cost from the starting position to newvex

        # update nearby vertices distance (if shorter)
        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if dist > radius:
                continue

            line = Line(vex, newvex)
            # if isThruObstacle(line, obstacles, rad):
            if isThruTriangle(line, obstacles):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        # ---- REACH
        line = Line(newvex, G.end)
        dist = distance(newvex, G.end)
        # if not isThruObstacle(line, obstacles, rad):
        if not isThruTriangle(line, obstacles):
            endidx = G.add_vex(G.end)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx] + dist)
            except:
                G.distances[endidx] = G.distances[newidx] + dist
            G.success = True
    return G


# ------------------------------ RRT_CONNECT --------------------------------


def connect(G: Graph, vex, obstacles, radius):  # TODO debug infinite loop
    while True:
        nearvex, nearidx = nearest(G, vex, obstacles,
                                   radius)  # Looks for the nearest visible vertex to vex in the graph
        if nearvex is None:  # If none is found, shoots a new vex
            return G, 'not yet'
        else:
            cidx = G.add_vex(vex)
            dist = distance(vex, nearvex)
            G.add_edge(nearidx, cidx, dist)
    return G, 'Reached'


def RRT_Connect(startpos, endpos, obstacles, n_iter, radius, stepSize):
    """
    Implements the RRT_Connect algorithm
    :param startpos: starting position
    :param endpos: ending position
    :param obstacles: list of tuples representing 3D obstacles
    :param n_iter: number of iterations of the algorithm
    :param radius: L1 norm of the obstacles
    :param stepSize:
    :return: g is a graph representing the tree built from the algorithm
    """

    # ---- INIT TREES
    G_A = Graph(startpos, endpos)
    G_B = Graph(endpos, startpos)
    path_A = []
    path_B = []
    for _ in range(n_iter):  # Operates n_iter times

        # ---- RANDOM CONFIG
        randvex = G_A.randomPosition()
        while isInObstacle(randvex, obstacles, radius):  # Ensures that the random vex is not too close to an obstacle
            randvex = G_A.randomPosition()

        # ---- EXTEND
        nearvex, nearidx = nearest(G_A, randvex, obstacles, radius)  # Looks for the nearest visible vertex to
        # randvex in the graph
        if nearvex is None:  # If none is found, shoots a new randvex
            continue

        newvex, _ = newVertex(randvex, nearvex, stepSize)
        dist = distance(newvex, nearvex)

        newidx = G_A.add_vex(newvex)
        G_A.add_edge(newidx, nearidx, dist)  # Adds a branch to the tree
        G_A.distances[newidx] = G_A.distances[nearidx] + dist  # Computes the cost from the starting position to newvex

        # update nearby vertices distance (if shorter)
        for vex in G_A.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if dist > radius:
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles, radius):
                continue

            cidx = G_A.vex2idx[vex]
            if G_A.distances[newidx] + dist < G_A.distances[cidx]:
                G_A.add_edge(cidx, newidx, dist)
                G_A.distances[cidx] = G_A.distances[newidx] + dist

        # ---- CONNECT
        G_B, status = connect(G_B, newvex, obstacles, radius)
        if status == 'Reached':
            G_A.success = True
            G_B.success = True
            G_A.end = newvex
            G_B.end = newvex
            path_A = findPath(G_A)
            path_B = findPath(G_B)
            return G_A, G_B, path_A, path_B
        G_A, G_B = swap(G_A, G_B)
    return G_A, G_B, path_A, path_B


def swap(G0, G1):
    """
    Just to swap two trees
    """
    G0, G1 = G1, G0
    return G0, G1


# ------------------------- Path Reconstruction -----------------------------


def findPath(G: Graph):
    """
    The way we build the tree makes it really easy to build the path_list from start to end
    Each node is only connected to a single other with an id below its own
    :param G: Our tree
    :return: Path from start to end
    """
    path = [G.end]
    predecessor = G.vertices[G.neighbors[G.vex2idx[path[-1]]][0][0]]
    while predecessor != G.start:
        path.append(predecessor)
        predecessor = G.vertices[G.neighbors[G.vex2idx[path[-1]]][0][0]]
    path.append(G.start)
    path.reverse()  # not necessary but nicer
    return path
