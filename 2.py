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



def extract_3d_points(path):

    if(path[-3:]=="pcd"):
        print("PCD")
        pcd_load = o3d.io.read_point_cloud(path)
        lidar_data = np.asarray(pcd_load.points)
    elif(path[-3:]=="npy"):
        print("NPY")
        lidar_data = np.load(path)

    inrange = np.where((lidar_data[:, 0] > 5) &
                       (lidar_data[:, 0] < 7.5) &
                       (np.abs(lidar_data[:, 1]) < 2) &
                       (lidar_data[:, 2] < 20))
    lidar_data = lidar_data[inrange[0]]

    lidar_data = np.random.random((1500, 3))
    print(lidar_data.shape)
    print(lidar_data)
    # print(inrange)

    cmap = matplotlib.cm.get_cmap('hsv')
    colors = cmap(lidar_data[:, -1] / np.max(lidar_data[:, -1]))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Select 3D LiDAR Points')
    ax.set_axis_off()
    ax.set_facecolor((0, 0, 0))
    ax.scatter(lidar_data[:, 0], lidar_data[:, 1], lidar_data[:, 2], c=colors, s=1, picker=5)

    # Equalize display aspect ratio for all axes
    max_range = (np.array([lidar_data[:, 0].max() - lidar_data[:, 0].min(), 
        lidar_data[:, 1].max() - lidar_data[:, 1].min(),
        lidar_data[:, 2].max() - lidar_data[:, 2].min()]).max() / 4.0)
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
        # if picked_3d and (x[ind] == picked_3d[-1][0] and y[ind] == picked_3d[-1][1] and z[ind] == picked_3d[-1][2]):
        #     return
        
        # Display picked_3d point
        picked_3d.append((x[ind], y[ind], z[ind]))
        corner_3d.append((x[ind], y[ind], z[ind]))
        print('IMG: %s', str(picked_3d[-1]))
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

if __name__ == '__main__':
    path_lidar = "camera_LiDAR_calibration/calib_checker/lidar/01/lidar_0.pcd"
    path_image = "camera_LiDAR_calibration/calib_checker/image/01/image_0.jpg"
    
    var = extract_3d_points(path_lidar)
    print("선택된 3D 좌표: ",var)