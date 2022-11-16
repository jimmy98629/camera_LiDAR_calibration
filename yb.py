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
import statsmodels.api as sm
from sklearn.linear_model import LinearRegression

for i in range(42):
        target_x=[]
        target_x.append()
target_x = #2d의 x좌표 추출해서 순서에 맞게 append
target_y = #2d의 x좌표 추출해서 순서에 맞게 append
target_1 = #2d의 x좌표 추출해서 순서에 맞게 append

coors_3d = # 3d좌표 그대로 순서에 맞게 append
# multi_model_x = sm.OLS(target_x, coors_3d)
# fitted_multi_model_x = multi_model_x.fit()

lr_X = LinearRegression(fit_intercept=False)
lr_X.fit(coors_3d, target_x)

print("회귀 계수: ", lr_X.coef_)

lr_Y = LinearRegression(fit_intercept=False)
lr_Y.fit(coors_3d, target_y)

print("회귀 계수: ", lr_Y.coef_)

lr_1 = LinearRegression(fit_intercept=False)
lr_1.fit(coors_3d, target_1)

print("회귀 계수: ", lr_1.coef_)

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