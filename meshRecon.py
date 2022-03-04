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

import numpy as np
import open3d as o3d


def txt2list(filename):  # Soon-to-be obsolete function if point clouds are directly passed from a SLAM
    file_obs = open(filename, 'r')
    obstacles = []  # list of obstacles
    for y in file_obs.read().split('\n'):
        obstacles.append([float(value) for value in y[1:-1].split(', ')])
    return obstacles


def list2PointCloud(obstacles):  # Soon-to-be obsolete function if point clouds are directly passed from a SLAM
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(obstacles)
    # pcd = pcd.voxel_down_sample(voxel_size=0.05)
    return pcd


def pointCloud2Mesh(pcd):
    alpha = .2  # parameter for alpha_shape
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    # o3d.io.write_triangle_mesh("./bpa_mesh.ply", mesh)
    # o3d.visualization.draw_geometries([pcd, mesh], mesh_show_back_face=True)
    return mesh


def getTriangles(mesh):
    return np.asarray(mesh.vertices), np.asarray(mesh.triangles)
