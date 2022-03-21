# PIE Navigation d'un drone autonome par Path Planning

Python version of the RRT-Star algorithm in a 3D mesh environment. This code is meant to be run in a ROS environment. 


## Description

An in-depth paragraph about your project and overview of use.

## Getting Started

### Dependencies

* Pyhon 3.6 and latest releases
* numpy, random, time, open3d, matplotlib.pyplot, mpl_toolkits.mplot3d.art3d

### Installing

* You can directly download the files from the main branch of this git.
* Some obsolete functions are still implemented, but not called, within the files, they are marked as such.

### Executing program

* You can start off by running main.py, but you can also add these files to your current projects:
* Comment the following line and provide your own point cloud (pcd).
```
    pcd = list2PointCloud(txt2list('obstacles.txt'))
    mesh = pointCloud2Mesh(pcd)
    
```

## Help

This code has been written using the latest Python release at the time, I do not guarantee that it will work with previous ones. Also, almost every function is coded with an added docstring to it,so help yourself and call help():
```
help(RRT_star)
Help on function RRT_star in module PathPlanning:
RRT_star(startpos, endpos, obstacles, window, n_iter, stepSize, radius=None)
    Implements the RRT Star algorithm
    :param startpos: starting position -> tuple of size 3
    :param endpos: goal position -> tuple of size 3
    :param obstacles: sets of triangles -> tuple of 2 arrays containing edges positions and idx
    :param window: search window
    :param n_iter: maximum number of iterations
    :param stepSize: minimum increment in the tree
    :param radius: obsolete parameter for polygonal representation of the obstacles
    :return: graph

```

## Authors

Contributors names and contact info

ex. Moussa Bendjilali at bendjilali.moussa@outlook.fr


## Version History

* 2.0
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 1.0
    * Initial Release


## Acknowledgments

Inspiration, code snippets, etc.
* [Fanjin Zeng](https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80)

