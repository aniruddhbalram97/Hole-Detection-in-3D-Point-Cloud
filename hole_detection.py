import open3d as o3d
import numpy as np
import logging 
import math
import os

class HoleDetection:
    
    def __init__(self, path):
        self.path = path
        self.pcd = []
        self.pcdArray = []
        self.index = []
        
    def readAndConvertObjToPointCloud(self, points):
        """
        A function which reads .obj file and converts it to pointcloud
        Args: points - number of points that need to be generated
        """
        mesh = o3d.io.read_triangle_mesh(self.path, True)
        self.pcd = mesh.sample_points_poisson_disk(points)
        self.pcdArray = np.asarray(self.pcd.points)
        return self.pcd, self.pcdArray
    
    
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
    
    def computeCentroid(self, index):
        """
        Function to compute centroid for a given array of points
        Args: pointArray - 2D array of the form [[x1 y1 z1] [x2 y2 z2] .... [xn, yn, zn]]
        Formula: Centroid = ((x1 + x2 + ... + xn)/n , (y1 + y2 + ... + yn)/n , (z1 + z2 + ... + zn)/n)
        """
        x = 0
        y = 0
        z = 0
        pointArray = np.asarray(self.pcdArray)[index[1:], :]
        for i in range(0, len(pointArray), 1):
            x = x + pointArray[i][0]
            y = y + pointArray[i][1]
            z = z + pointArray[i][2]
        centroid = [x / len(pointArray), y / len(pointArray), z / len(pointArray)]
        return centroid
    
    def findRadialNeighbors(self, point, radius):
        """
        Function to find neighboring points within a given radius
        Args: point - index of point under consideration
              radius - max-radius within which nearest neighbors are defined
        """
        pcdKdTree = o3d.geometry.KDTreeFlann(self.pcd)
        [k, self.index, _] = pcdKdTree.search_radius_vector_3d(self.pcd.points[point], radius)
        return self.index
    
    def extractBoundaryPoints(self, radius, tolerance):
        """
        Function to extract boundary points
        Args: radius - maximum radius to find the radial neighbors for a given point
              tolerance - To detect an outlier point. A point is a boundary point if its distance from centroid is greater than a certain tolerance level
        """
        boundary_points = []
        pcdArray = self.pcdArray
        for i in range(0, len(np.asarray(pcdArray)), 1):
            point = pcdArray[i]
            index = np.asarray(self.findRadialNeighbors(i, radius))
            print(len(index))
            centroid = self.computeCentroid(index)
            if(self.computeSquaredDistance(centroid, point) > tolerance):
                boundary_points.append(list(point))
            logging.info("Iteration...%d", i)
        boundary_points = np.array(boundary_points)
        return boundary_points
        
    

# Boundary Extraction test
hole_detection = HoleDetection(os.path.abspath(os.getcwd()) + "/models_3d/dragon_with_hole.obj")
pcd, pcdArray = hole_detection.readAndConvertObjToPointCloud(50000)

#index = hole_detection.findRadialNeighbors(1500, 0.5)
boundary_points = hole_detection.extractBoundaryPoints(2, 0.4)
boundary_pcd = o3d.geometry.PointCloud()
boundary_pcd.points = o3d.utility.Vector3dVector(boundary_points)
boundary_pcd.paint_uniform_color([0.5, 0.5, 0.5])
#np.asarray(pcd.colors)[index[1:],:] = [0, 0, 1]
o3d.visualization.draw_geometries([boundary_pcd])

