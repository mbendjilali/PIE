import numpy as np
import open3d as o3d


def txt2list(filename):

    file_obs = open(filename, 'r')
    obstacles = []  # list of obstacles
    for y in file_obs.read().split('\n'):
        obstacles.append([float(value) for value in y[1:-1].split(', ')])
    return obstacles


def list2PointCloud(obstacles):

    # Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(obstacles)
    pcd = pcd.voxel_down_sample(voxel_size=0.15)
    return pcd


def pointCloud2Mesh(pcd):

    alpha = .4  # parameter for alpha_shape
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    # Export and visualize
    #
    # o3d.io.write_triangle_mesh("./bpa_mesh.ply", mesh)
    # o3d.visualization.draw_geometries([pcd, mesh], mesh_show_back_face=True)
    return mesh


def getTriangles(mesh):

    return np.asarry(mesh.vertices), np.asarray(mesh.triangles)