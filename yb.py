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


d2=[]
for i in range(42):
        # print("-----------------------")
        path = "C:/Users/syb62/Desktop/URP/camera_LiDAR_calibration/ml/01/01_2d/2d_{}.npy".format(i)
        data = np.load(path)
        for j in range(data.shape[0]):
                # print(data[j])
                a = data[j].tolist()
                a.append(1)
                d2.append(a)

d3=[]
for i in range(42):
        # print("-----------------------")
        path = "C:/Users/syb62/Desktop/URP/camera_LiDAR_calibration/ml/01/01_3d/3d_{}.npy".format(i)
        data = np.load(path)
        for j in range(data.shape[0]):
                # print(data[j])
                a = data[j].tolist()
                a.append(1)
                d3.append(a)


d2 = np.array(d2)
d3 = np.array(d3)
# print("d2: ",d2)
# print(d2.shape)
# print("d3: ",d3)
# print(d3.shape)

target_x = d2[:,0]
print("target_x",target_x.shape)
target_y = d2[:,1]
# print("target_y",target_y)
target_1 = d2[:,2]
# print("target_1",target_1)

coors_3d = d3
# print("coors_3d: ", coors_3d)

lr_X = LinearRegression(fit_intercept=False)
lr_X.fit(coors_3d, target_x)
print("회귀 계수: ", lr_X.coef_)
lr_X_coef = lr_X.coef_.reshape(4,1)

lr_Y = LinearRegression(fit_intercept=False)
lr_Y.fit(coors_3d, target_y)
print("회귀 계수: ", lr_Y.coef_)
lr_Y_coef = lr_Y.coef_.reshape(4,1)

lr_1 = LinearRegression(fit_intercept=False)
lr_1.fit(coors_3d, target_1)
print("회귀 계수: ", lr_1.coef_)
lr_1_coef = lr_1.coef_.reshape(4,1)

plt.scatter(coors_3d, target_x)
plt.show()

cam2pix = np.array([[1852.666, 0, 982.862],

                    [0, 1866.610, 612.790],

                    [0, 0, 1]])

lr_proj = np.concatenate((lr_X.coef_,lr_Y.coef_,lr_1.coef_),axis=0)
lr_proj = lr_proj.reshape(3,4)
print(lr_proj)
print(lr_proj.shape)

def projection_LR(lr_proj, intrinsic_matrix,path_lidar,path_img):
#     proj = intrinsic_matrix.dot(projection_matrix)
    proj = lr_proj
    img = image.imread(path_img)

    if(path_lidar[-3:]=="pcd"):
        print("PCD")
        pcd_load = o3d.io.read_point_cloud(path_lidar)
        lidar_data = np.asarray(pcd_load.points)
    elif(path_lidar[-3:]=="npy"):
        print("NPY")
        lidar_data = np.load(path_lidar)

    points_3d = lidar_data
    
    # Filter points in front of camera
    inrange = np.where((points_3d[:, 2] > -5) &
                       (points_3d[:, 2] < 7) &
                       (points_3d[:, 0] < 7) &
                       (points_3d[:, 0] > 0) &
                       (np.abs(points_3d[:, 1]) < 5))
    max_intensity = np.max(points_3d[:, -1])
    points_3d = points_3d[inrange[0]]

    # # Color map for the points
    cmap = matplotlib.cm.get_cmap('jet')
    colors = cmap(points_3d[:, -1] / max_intensity) * 255

    # print("모양 앞에 mat: {0}".format(proj.shape))
    # print(points_3d[:, :3])
    for p in points_3d[:, :3]:
        p = np.append(p, 1)
        # print("뒤에 mat: {0}".format(p.shape))
    # points_3d = [np.append(p,1) for p in points_3d[:, :3]]
    # print(points_3d)
    # Project to 2D and filter points within image boundaries
    points2D = [ proj.dot(np.append(point,1)) for point in points_3d[:, :3] ]
    points2D = np.asarray(points2D)

     # print("변경전:",points2D)
    for i in range(points2D.shape[0]):
        points2D[i,:] = points2D[i,:]/points2D[i,2]
    # print(points2D)
    # print(points2D.shape)
    inrange = np.where((points2D[:, 0] >= 0) &
                       (points2D[:, 1] >= 0) &
                       (points2D[:, 0] < img.shape[1]) &
                       (points2D[:, 1] < img.shape[0]))
    points2D = points2D[inrange[0]].round().astype('int')

    # print("투사된 point 2d: ",points2D)
    points2D = points2D[:,:2]
    # print("변경된 투사된 point 2d: ",points2D)
    # Draw the projected 2D points
    for i in range(len(points2D)):
        cv2.circle(img, tuple(points2D[i]), 2, tuple(colors[i]), -1)
    
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    disp = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2RGB)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('Select 2D Image Points')
    ax.set_axis_off()
    ax.imshow(disp)

for i in range(42):
        path_lidar = "camera_LiDAR_calibration/calib_checker/lidar/01/lidar_{}.pcd".format(i)
        path_image = "camera_LiDAR_calibration/calib_checker/image/01/image_{}.jpg".format(i)
        proj = np.load("camera_LiDAR_calibration/projection_matrix/01/projection_matrix{}.npy".format(i))
        # a = projection(lr_proj, cam2pix, path_lidar, path_image)
        print("projection matrix{}: ".format(i), proj)
        print(lr_proj)
    
# for i in range(42):
#     proj = np.load("camera_LiDAR_calibration/projection_matrix/01/projection_matrix{}.npy".format(i))
#     print("projection matrix{}: ".format(i), proj)