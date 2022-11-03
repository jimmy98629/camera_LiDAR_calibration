from sys import flags
import numpy as np
from PIL import Image
import cv2
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import image
from mpl_toolkits.mplot3d import Axes3D
import copy
import open3d as o3d

print("")

# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("../../TestData/sync.ply")
o3d.visualization.draw_geometries([pcd_load])

# convert Open3D.o3d.geometry.PointCloud to numpy array
xyz_load = np.asarray(pcd_load.points)
print('xyz_load')
print(xyz_load)