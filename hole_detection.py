import open3d as o3d
import numpy as np
import math
import os

class HoleDetection:
    
    def __init__(self, path):
        self.path = path
        
    def readAndConvertObjToPointCloud(self, points):
        """
        A function which reads .obj file and converts it to pointcloud
        Args: points - number of points that need to be generated
        """
        mesh = o3d.io.read_triangle_mesh(self.path, True)
        pcd = mesh.sample_points_poisson_disk(points)
        pcdArray = np.asarray(pcd.points)
        return pcd, pcdArray
    
pointCloud, pointCloudArray = HoleDetection(os.path.abspath(os.getcwd()) + "/models_3d/dragon_with_hole.obj").readAndConvertObjToPointCloud(5000)
o3d.visualization.draw_geometries([pointCloud])