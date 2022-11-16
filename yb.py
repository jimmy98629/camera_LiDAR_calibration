from sys import flags
import numpy as np
from PIL import Image
import cv2
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import image
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
from main import projection

cam2pix = np.array([[1852.666, 0, 982.862],

                    [0, 1866.610, 612.790],

                    [0, 0, 1]])

for i in range(50):
        path_lidar = "camera_LiDAR_calibration/calib_checker/lidar/02/lidar_{}.pcd".format(i)
        path_image = "camera_LiDAR_calibration/calib_checker/image/02/image_{}.jpg".format(i)
        proj = np.load("camera_LiDAR_calibration/projection_matrix/01/projection_matrix{}.npy".format(i))
        a = projection(proj, cam2pix, path_lidar, path_image)
        print("projection matrix{}: ".format(i), proj)
    
for i in range(42):
    proj = np.load("camera_LiDAR_calibration/projection_matrix/01/projection_matrix{}.npy".format(i))
    print("projection matrix{}: ".format(i), proj)