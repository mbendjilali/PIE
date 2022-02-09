"""
This code is an attempt at implementing a 3D version of the previously coded RRT-start by Fanjin Zeng.
https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80

Bendjilali Moussa 2022.
"""

from collections import deque
from Geometry import *


def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    """
    Implements the RRT Star algorithm.
    This part of the code can be kept unchanged regardless of the workspace used (R2 or R3).
    """
    G = Graph(startpos, endpos)

    for _ in range(n_iter):  # Operates n_iter times

        if endpos in G.vex2idx:
            break

        randvex = G.randomPosition()
        while isInObstacle(randvex, obstacles, radius):  # Ensures that the random vex is not too close to an obstacle
            randvex = G.randomPosition()

        nearvex, nearidx = nearest(G, randvex, obstacles,
                                   radius)  # Looks for the nearest visible vertex to randvex in the graph
        if nearvex is None:  # If none is found, shoots a new randex
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

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

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx] + dist)
            except:
                G.distances[endidx] = G.distances[newidx] + dist

            G.success = True
            # print('success')
            # break
    return G


def dijkstra(G):
    """
    Dijkstra algorithm for finding the shortest path from start position to end.
    This part of the code can be kept unchanged regardless of the workspace used (R2 or R3).
    """
    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return np.array(path)


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        return path
