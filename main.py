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


def plot(G, obstacles, radius, path=None):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(G.startpos[0], G.startpos[1], G.startpos[2], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], G.endpos[2], c='black')

    for obs in obstacles:
        x, y, z = drawSphere(obs, radius)
        ax.plot_surface(x, y, z, color='blue', linewidth=0.0)

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
    endpos = (5., 5., 5.)
    obstacles = [(1., 1., 1.), (3., 2., 2.)]  # Format pour les obstacles : point dans un espace 3D
    n_iter = 200
    radius = 0.5
    stepSize = 0.7

    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)

    if G.success:
        path = dijkstra(G)  # If a path is found, it needs to be transformed into GPS waypoints.
        # print(path)
        plot(G, obstacles, radius, path)
    else:
        print(0)
        plot(G, obstacles, radius)