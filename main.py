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

from PathPlanning import *
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.pyplot as plt
from meshRecon import *

# ------------------------------ Obsolete plot functions -----------------------------------


def drawSphere(obs, rad):
    u = np.linspace(0, 2 * np.pi, 10)
    v = np.linspace(0, np.pi, 10)
    x = np.outer(np.cos(u), rad * np.sin(v)) + obs[0]
    y = np.outer(np.sin(u), rad * np.sin(v)) + obs[1]
    z = np.outer(np.ones(np.size(u)), rad * np.cos(v)) + obs[2]
    return x, y, z


def drawCube(obs, rad):
    x = np.array([obs[0] - rad, obs[0] + rad])
    y = np.array([obs[1] - rad, obs[1] + rad])
    z = np.array([obs[2] - rad, obs[2] + rad])
    return x, y, z


def plot(obstacles, rad=None, g=None, path_list=None):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    if obstacles is not None:
        for obs in obstacles:
            x, y, z = drawSphere(obs, rad)
            #  x, y, z = drawCube(obs, rad)
            ax.plot_surface(x, y, z, color='blue', linewidth=0.0)

    if g is not None:
        ax.scatter(g.start[0], g.start[1], g.start[2], c='black')
        ax.scatter(g.end[0], g.end[1], g.end[2], c='black')
        line_list = [(g.vertices[edge[0]], g.vertices[edge[1]]) for edge in g.edges]
        lc = Line3DCollection(line_list, colors='green', linewidths=1)
        ax.add_collection(lc)

    if path_list is not None:
        paths = [(path_list[i], path_list[i + 1]) for i in range(len(path_list) - 1)]
        lc2 = Line3DCollection(paths, colors='red', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()

# --------------------------------------- main ---------------------------------------------


if __name__ == '__main__':
    """
    Parameters like start, end, etc. needs to be chosen accordingly to the framework we're in.
    """
    start = (0., 0., 0.)
    end = (4., 1.5, .5)
    window = Window(start, end)
    pcd = list2PointCloud(txt2list('obstacles.txt'))
    mesh = pointCloud2Mesh(pcd)
    triangles = getTriangles(mesh)
    n_iter = 100
    radius = 0.5
    stepSize = 0.5

    # G_A, G_B, path_A, path_B = RRT_Connect(start, end, obstacles, n_iter, rad, stepSize)
    G = RRT_star(start, end, triangles, window, n_iter, stepSize, radius)

    if G.success:
        path = findPath(G)
        # print("success :", time() - t)
        # plot(obstacles, rad, g, path_list)

        lines = [[i, i + 1] for i in range(len(path) - 1)]
        colors = [[1, 0, 0] for i in range(len(lines))]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(path)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([line_set, pcd, mesh], mesh_show_back_face=True)

    else:
        o3d.visualization.draw_geometries([pcd, mesh], mesh_show_back_face=True)
