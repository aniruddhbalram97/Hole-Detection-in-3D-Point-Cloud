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
    
    def computeSquaredDistance(self, point1, point2):
        """
        Function to compute squared distances between two points
        Args: point1 is an array of form [x y z]
              point2 is an array of form [x y z]
        Formula: sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2), 2-Norm of a matrix
        """
        x1, y1, z1 = point1[0], point1[1], point1[2]
        x2, y2, z2 = point2[0], point2[1], point2[2]
        squaredDistance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return squaredDistance
    
    def computeCentroid(self, pointArray):
        """
        Function to compute centroid for a given array of points
        Args: pointArray is a 2D array of the form [[x1 y1 z1] [x2 y2 z2] .... [xn, yn, zn]]
        Formula: Centroid = ((x1 + x2 + ... + xn)/n , (y1 + y2 + ... + yn)/n , (z1 + z2 + ... + zn)/n)
        """
        x = 0
        y = 0
        z = 0
        for i in range(0, len(pointArray), 1):
            x = x + pointArray[i][0]
            y = y + pointArray[i][1]
            z = z + pointArray[i][2]
        centroid = [x / len(pointArray), y / len(pointArray), z / len(pointArray)]
        return centroid
    
pointCloud, pointCloudArray = HoleDetection(os.path.abspath(os.getcwd()) + "/models_3d/dragon_with_hole.obj").readAndConvertObjToPointCloud(5000)
o3d.visualization.draw_geometries([pointCloud])