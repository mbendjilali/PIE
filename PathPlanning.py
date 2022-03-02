"""
This code is an attempt at implementing a 3D version of the previously coded RRT-start by Fanjin Zeng.
https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80

Bendjilali Moussa 2022.
"""

from Geometry import *


def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    """
    Implements the RRT Star algorithm.
    This part of the code can be kept unchanged regardless of the workspace used (R2 or R3).
    """

    # ---- INIT TREE
    G = Graph(startpos, endpos)

    for i in range(n_iter):  # Operates n_iter times

        if endpos in G.vex2idx:
            G.success = True
            break

        # ---- RANDOM CONFIG
        randvex = G.randomPosition()
        while isInObstacle(randvex, obstacles, radius):  # Ensures that the random vex is not too close to an obstacle
            randvex = G.randomPosition()

        # ---- EXTEND
        nearvex, nearidx = nearest(G, randvex, obstacles,
                                   radius)  # Looks for the nearest visible vertex to randvex in the graph
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
            if isThruObstacle(line, obstacles, radius):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        # ---- REACH
        line = Line(newvex, G.endpos)
        dist = distance(newvex, G.endpos)
        if not isThruObstacle(line, obstacles, radius):
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx] + dist)
            except:
                G.distances[endidx] = G.distances[newidx] + dist
            G.success = True
    return G


def connect(G: Graph, vex, obstacles, radius, stepSize):
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
    :return: G is a graph representing the tree built from the algorithm
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
        nearvex, nearidx = nearest(G_A, randvex, obstacles, radius)  # Looks for the nearest visible vertex to randvex in the graph
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
        G_B, status = connect(G_B, newvex, obstacles, radius, stepSize)
        if status == 'Reached':
            G_A.success = True
            G_B.success = True
            G_A.endpos = newvex
            G_B.endpos = newvex
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


def findPath(G: Graph):
    """
    The way we build the tree makes it really easy to build the path from startpos to endpos
    :param G: Our tree
    :return: Path from startpos to endpos
    """
    path = [G.endpos]
    predecessor = G.vertices[G.neighbors[G.vex2idx[path[-1]]][0][0]]
    while predecessor != G.startpos:
        path.append(predecessor)
        predecessor = G.vertices[G.neighbors[G.vex2idx[path[-1]]][0][0]]
    path.append(G.startpos)
    path.reverse()  # not necessary but nicer
    return path
