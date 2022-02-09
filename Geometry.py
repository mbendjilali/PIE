from ClassDefinition import *


def Intersection(line, center, radius):
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


def distance(p1, p2):
    """Computes the distance between two points"""

    return np.linalg.norm(np.array(p1) - np.array(p2))


def isInObstacle(vex, obstacles, radius):
    """ Returns True if the point vex is too close (radius) to an obstacle"""

    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    """ Returns True if the point vex is located in an obstacle"""

    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
    return False


def nearest(G, vex, obstacles, radius):
    """Returns the closest visible vertex in G to vex"""

    Nvex = None
    Nidx = None
    minDist = float("inf")

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
    """ Shoots a new vertex between randvex and nearvex """

    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min(stepSize, length)  # This ensures that the shot vertex lies between randvex and nearvex
    newvex = (nearvex[0] + dirn[0], nearvex[1] + dirn[1], nearvex[2] + dirn[2])
    return newvex


def window(startpos, endpos):
    """ Defines search window - 2 times of start to end rectangle. This version works in 3D. """

    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    depth = endpos[2] - startpos[2]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    winz = startpos[2] - (depth / 2.)
    return winx, winy, winz, width, height, depth


def isInWindow(pos, winx, winy, winz, width, height, depth):
    """ Restricts new vertex inside the search window. This version works in 3D. """

    if winx < pos[0] < winx + width and \
            winy < pos[1] < winy + height and \
            winz < pos[2] < winz + depth:
        return True
    else:
        return False
