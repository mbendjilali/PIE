"""
This code is an attempt at implementing a 3D version of the previously coded RRT-start by Fanjin Zeng.
https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80

Bendjilali Moussa 2022.
"""

from PathPlanning import *
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.pyplot as plt


def drawSphere(obs, radius):
    u = np.linspace(0, 2 * np.pi, 10)
    v = np.linspace(0, np.pi, 10)
    x = np.outer(np.cos(u), radius * np.sin(v)) + obs[0]
    y = np.outer(np.sin(u), radius * np.sin(v)) + obs[1]
    z = np.outer(np.ones(np.size(u)), radius * np.cos(v)) + obs[2]
    return x, y, z


def drawCube(obs, radius):
    x = np.array([obs[0] - radius, obs[0] + radius])
    y = np.array([obs[1] - radius, obs[1] + radius])
    z = np.array([obs[2] - radius, obs[2] + radius])
    return x, y, z


def plot(obstacles, radius, G=None, path=None):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for obs in obstacles:
        x, y, z = drawSphere(obs, radius)
        #  x, y, z = drawCube(obs, radius)
        ax.plot_surface(x, y, z, color='blue', linewidth=0.0)

    if G is not None:
        ax.scatter(G.startpos[0], G.startpos[1], G.startpos[2], c='black')
        ax.scatter(G.endpos[0], G.endpos[1], G.endpos[2], c='black')
        lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
        lc = Line3DCollection(lines, colors='green', linewidths=1)
        ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        lc2 = Line3DCollection(paths, colors='red', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()


if __name__ == '__main__':
    """
    Parameters like startpos, end pos, etc. needs to be chosen accordingly to the framework we're in.
    """
    startpos = (0., 0., 0.)
    endpos = (9., 9., 9.)
    obstacles = [[3., 3., 3.], [6., 6., 8.], [2., 4., 6.], [7., 3., 2.], [5., 7.5, 2.], [7.5, 7.5, 7.5]]  # The
    # obstacles are represented by tuples
    n_iter = 1000
    radius = 1.9
    stepSize = 0.7
    #
    # G_A, G_B, path_A, path_B = RRT_Connect(startpos, endpos, obstacles, n_iter, radius, stepSize)
    # plot(obstacles, radius, None, path)

    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = findPath(G)
        plot(obstacles, radius, G, path)
    else:
        print(0)
        plot(obstacles, radius, G)
