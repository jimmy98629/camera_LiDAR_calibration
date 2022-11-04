
from sys import flags
import numpy as np
from PIL import Image
import cv2
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import image
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
# import PyQt5

cam2pix = np.array([[1852.666, 0, 982.862],

                    [0, 1866.610, 612.790],

                    [0, 0, 1]])

ex_cam_params = np.array([[0,0,0,0.11],
                          [0,0,0,-0.16],
                          [0,0,0,-0.48]])

# 2. 외부 파라미터
# 카메라 위치 = 라이다 위치 + [0.11 -0.16 -0.48]
# 차량 전방이 x축, 왼쪽이 y축, 높이가 z축입니다



def extract_corner_points(path):
    
    # img_data = np.asarray(Image.open("camera/37.png"))
    img = image.imread(path) # need to modify 

    disp = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2RGB)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('Select 2D Image Points')
    ax.set_axis_off()
    ax.imshow(disp)
    
    picked, corners = [], []
    def onclick(event):
        x = event.xdata
        y = event.ydata
        if (x is None) or (y is None): return

        # Display the picked point
        picked.append((x, y))
        corners.append((x, y))
        print('IMG: %s', str(picked[-1]))

        if len(picked) > 1:
            # Draw the line
            temp = np.array(picked)
            ax.plot(temp[:, 0], temp[:, 1])
            ax.figure.canvas.draw_idle()
    
            # Reset list for future pick events
            del picked[0]

    # Display GUI
    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    return corners

def extract_3d_points(path):

    if(path[-3:]=="pcd"):
        print("PCD")
        pcd_load = o3d.io.read_point_cloud(path)
        lidar_data = np.asarray(pcd_load.points)
    elif(path[-3:]=="npy"):
        print("NPY")
        lidar_data = np.load(path)

    # convert Open3D.o3d.geometry.PointCloud to numpy array
    # xyz_load = np.asarray(pcd_load.points)
    inrange = np.where((lidar_data[:, 0] > 0) &
                       (lidar_data[:, 0] < 20) &
                       (np.abs(lidar_data[:, 1]) < 20) &
                       (lidar_data[:, 2] < 20))
    lidar_data = lidar_data[inrange[0]]
    print(lidar_data.shape)
    # print(inrange)

    cmap = matplotlib.cm.get_cmap('hsv')
    colors = cmap(lidar_data[:, -1] / np.max(lidar_data[:, -1]))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Select 3D LiDAR Points')
    ax.set_axis_off()
    ax.set_facecolor((0, 0, 0))
    ax.scatter(lidar_data[:, 0], lidar_data[:, 1], lidar_data[:, 2], c=colors, s=0.07, picker=5)

    # Equalize display aspect ratio for all axes
    max_range = (np.array([lidar_data[:, 0].max() - lidar_data[:, 0].min(), 
        lidar_data[:, 1].max() - lidar_data[:, 1].min(),
        lidar_data[:, 2].max() - lidar_data[:, 2].min()]).max() / 8.0)
    mid_x = (lidar_data[:, 0].max() + lidar_data[:, 0].min()) * 0.5
    mid_y = (lidar_data[:, 1].max() + lidar_data[:, 1].min()) * 0.5
    mid_z = (lidar_data[:, 2].max() + lidar_data[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    picked_3d, corner_3d = [], []
    def onpick(event):
        ind = event.ind[0]
        x, y, z = event.artist._offsets3d

        # Ignore if same point selected again
        if picked_3d and (x[ind] == picked_3d[-1][0] and y[ind] == picked_3d[-1][1] and z[ind] == picked_3d[-1][2]):
            return
        
        # Display picked_3d point
        picked_3d.append((x[ind], y[ind], z[ind]))
        corner_3d.append((x[ind], y[ind], z[ind]))
        # rospy.loginfo('PCL: %s', str(picked_3d[-1]))

        if len(picked_3d) > 1:
            # Draw the line
            temp = np.array(picked_3d)
            ax.plot(temp[:, 0], temp[:, 1], temp[:, 2])
            ax.figure.canvas.draw_idle()

            # Reset list for future pick events
            del picked_3d[0]

    # Display GUI
    fig.canvas.mpl_connect('pick_event', onpick)
    plt.show()
    return corner_3d


def calibrate(points_2d, points_3d):
    intrinsic_cam_matrix= np.array([[1852.666, 0, 982.862],

                    [0, 1866.610, 612.790],

                    [0, 0, 1]])
    
    dist_coeff = np.array([[0.0,0.0,0.0,0.0,0.0]])
    points_2d = np.array(points_2d)
    points_3d = np.array(points_3d)
    print(points_2d.shape)
    print(points_3d.shape)
    success, rotation_matrix, translation_matrix, inlier = cv2.solvePnPRansac(points_3d,
    points_2d, intrinsic_cam_matrix, dist_coeff, flags=cv2.SOLVEPNP_ITERATIVE)
    print("rotation matrix",rotation_matrix)
    print("translation matrix", translation_matrix)

    rotation_matrix = cv2.Rodrigues(rotation_matrix)
    # print("rotation matrix2: ",rotation_matrix[0])
    return rotation_matrix[0]

def projection():
    
    pass

if __name__ == '__main__':
    # path_lidar = "camera_LiDAR_calibration/lidar/0.npy"
    path_lidar = "camera_LiDAR_calibration/calib_checker/lidar/01/lidar_0.pcd"
    path_image = "camera_LiDAR_calibration/camera/37.png"
    corner = extract_corner_points(path_image)
    print("선택된 2D 코너: ",corner)
    print("2D type: ",np.array(corner))
    var = extract_3d_points(path_lidar)
    print("선택된 3D 좌표: ",var)
    # print("선택된 3d 크기: ", var.s)
    rot = calibrate(corner, var)
    print("rotation matrix: ", rot)
    

