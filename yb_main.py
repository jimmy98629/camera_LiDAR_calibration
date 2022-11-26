from sys import flags
import numpy as np
from PIL import Image
import cv2
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import image
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
# import image_geometry
# import PyQt5

# cam2pix = np.array([[1852.666, 0, 982.862],

#                     [0, 1866.610, 612.790],

#                     [0, 0, 1]])

# 시뮬레이션용 cam2pix matrix
cam2pix = np.array([[831.3844, 0.0, 480],
                    [0.0, 890.07074, 270.0],
                    [0.0, 0.0, 1.0]])

ex_cam_params = np.array([[0,0,0,0.77],
                          [0,0,0,-0.16],
                          [0,0,0,-0.48]])

# 시뮬레이션용 reference matrix
# R|t:[[ 0.  -1.   0.   0. ]
#  [ 0.   0.  -1.   0. ]
#  [ 1.   0.   0.  -0.5]
#  [ 0.   0.   0.   1. ]]

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
    elif(path[-3:]=="bin"):
        print("BIN")
        lidar_data = np.fromfile(path,dtype=np.float32).reshape(-1,4)

    inrange = np.where((lidar_data[:, 0] > 10) &
                       (lidar_data[:, 0] < 50) &
                       (np.abs(lidar_data[:, 1]) < 20) &
                       (lidar_data[:, 2] < 50))
    lidar_data = lidar_data[inrange[0]]
    # print(lidar_data.shape)
    # print(inrange)

    cmap = matplotlib.cm.get_cmap('hsv')
    colors = cmap(lidar_data[:, -1] / np.max(lidar_data[:, -1]))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Select 3D LiDAR Points')
    ax.set_axis_off()
    ax.set_facecolor((0, 0, 0))
    ax.scatter(lidar_data[:, 0], lidar_data[:, 1], lidar_data[:, 2], c=colors, s=2, picker=5)

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
        if picked_3d and (x[ind] == picked_3d[-1][0] and y[ind] == picked_3d[-1][1] and z[ind] == picked_3d[-1][2]):
            return
        
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


def calibrate(points_2d, points_3d):
    intrinsic_cam_matrix= np.array([[831.3844, 0.0, 480],
                    [0.0, 890.07074, 270.0],
                    [0.0, 0.0, 1.0]])
    
    dist_coeff = np.array([[0.0,0.0,0.0,0.0,0.0]])
    points_2d = np.array(points_2d)
    points_3d = np.array(points_3d)
    print(points_2d.shape)
    print(points_3d.shape)
    success, rotation_matrix, translation_matrix, inlier = cv2.solvePnPRansac(points_3d, points_2d, intrinsic_cam_matrix, dist_coeff, flags=cv2.SOLVEPNP_ITERATIVE)
    print("rotation matrix",rotation_matrix)
    print("translation matrix", translation_matrix)

    rotation_matrix = cv2.Rodrigues(rotation_matrix)
    rot2 = rotation_matrix[0]
    print("rotation matrix2: ",rot2)
    projection_mat = np.concatenate((rotation_matrix[0],translation_matrix),axis=1)
    # return rotation_matrix[0]
    return projection_mat


def projection(projection_matrix, intrinsic_matrix,path_lidar,path_img):
    proj = intrinsic_matrix.dot(projection_matrix)
    img = image.imread(path_img)

    if(path_lidar[-3:]=="pcd"):
        print("PCD")
        pcd_load = o3d.io.read_point_cloud(path_lidar)
        lidar_data = np.asarray(pcd_load.points)
    elif(path_lidar[-3:]=="npy"):
        print("NPY")
        lidar_data = np.load(path_lidar)
    elif(path_lidar[-3:]=="bin"):
        print("BIN")
        lidar_data = np.fromfile(path_lidar,dtype=np.float32).reshape(-1,4)

    points_3d = lidar_data
    
    # Filter points in front of camera
    inrange = np.where((points_3d[:, 2] > -5) &
                       (points_3d[:, 2] < 9) &
                       (points_3d[:, 0] < 50) &
                       (points_3d[:, 0] > 20) &
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

def read_cali(calib_file):
    f = open(calib_file,'r')
    calib = f.readlines()
    P2 = calib[2].split(' ')[1:]
    P2 = np.array(P2, dtype=np.float32).reshape(-1,4)

    R0 = calib[4].split(' ')[1:]
    R0 = np.array(R0, dtype=np.float32).reshape(3, 3)
    R0_ext = np.eye(4, dtype=np.float32)
    R0_ext[:3, :3] = R0

    Tr = calib[5].split(' ')[1:]
    Tr = np.array(Tr, dtype=np.float32).reshape(3, 4)

    velo_to_cam = np.eye(4, dtype=np.float32)
    velo_to_cam[:3, :4] = Tr

    cam_to_pix = np.matmul(P2, R0_ext)

    return cam_to_pix, velo_to_cam

if __name__ == '__main__':
    
    for i in range(51):
        # 경로 너꺼에 맞게 변경하면 될듯?
        if(len(str(i))==1): # 00000뒤에 한자리 수 파일일떄
            path_image = "C:/Users/syb62/Desktop/URP/urp_calib_vtd/urp_calib_vtd/img/00000{}.png".format(i)
            path_lidar = "C:/Users/syb62/Desktop/URP/urp_calib_vtd/urp_calib_vtd/lidar/00000{}.bin".format(i)
            path_calib = "C:/Users/syb62/Desktop/URP/urp_calib_vtd/urp_calib_vtd/calib/00000{}.txt".format(i)
        elif(len(str(i))==2): # 0000뒤에 두자리 수 파일일떄
            path_image = "C:/Users/syb62/Desktop/URP/urp_calib_vtd/urp_calib_vtd/img/0000{}.png".format(i)
            path_lidar = "C:/Users/syb62/Desktop/URP/urp_calib_vtd/urp_calib_vtd/lidar/0000{}.bin".format(i)
            path_calib = "C:/Users/syb62/Desktop/URP/urp_calib_vtd/urp_calib_vtd/calib/0000{}.txt".format(i)
        # path_calib = "C:/Users/syb62/Downloads/test_set/test_set/calib/000000.txt"
        try:
            print("----------------{}번쨰 img, lidar------------".format(i))
            cam_mat, velo_mat = read_cali(path_calib)
            print("cam_mat:{}\nR|t:{}".format(cam_mat, velo_mat))
            corner_2d = extract_corner_points(path_image)
            print("선택된 2D 코너: ",corner_2d)
            print("2D type: ",np.array(corner_2d))
            corner_3d = extract_3d_points(path_lidar)
            print("선택된 3D 좌표: ",corner_3d)
            # print("선택된 3d 크기: ", var.s)
            proj = calibrate(corner_2d, corner_3d)
            # print("rotation matrix: ", rot)
            # print("---------------projection matrix{}: ".format(i),proj)
            print("---------------projection matrix: ",proj)
            a = projection(proj, cam2pix, path_lidar, path_image)
            print(a)
            # 동준아 너가 원하는 경로로 지정하고 저장하면 될듯해
            np.save("urp_calib_proj/projection_matrix{}".format(i), proj)
        except:
            pass
    
    # for i in range(50):
    #     path_lidar = "camera_LiDAR_calibration/calib_checker/lidar/02/lidar_{}.pcd".format(i)
    #     path_image = "camera_LiDAR_calibration/calib_checker/image/02/image_{}.jpg".format(i)
    #     corner_2d = extract_corner_points(path_image)
    #     print("선택된 2D 코너: ",corner_2d)
    #     print("2D type: ",np.array(corner_2d))
    #     corner_3d = extract_3d_points(path_lidar)
    #     print("선택된 3D 좌표: ",corner_3d)
    #     # print("선택된 3d 크기: ", var.s)
    #     proj = calibrate(corner_2d, corner_3d)
    #     # print("rotation matrix: ", rot)
    #     print("---------------projection matrix{}: ".format(i),proj)
    #     a = projection(proj, cam2pix, path_lidar, path_image)
    #     print(a)
    #     np.save("ml/02/02_2d/2d_{}".format(i), corner_2d)
    #     np.save("ml/02/02_3d/3d_{}".format(i), corner_3d)
    #     np.save("ml/02/02_proj/projection_matrix{}".format(i), proj)
    
    # for i in range(50):
    #     corner_2d = np.load("ml/02/02_2d/2d_{}.npy".format(i))
    #     print("2d_{}: ".format(i),corner_2d)
    #     corner_3d = np.load("ml/02/02_3d/3d_{}.npy".format(i))
    #     print("3d_{}: ".format(i),corner_3d)
    #     lidar_data = np.load("ml/02/02_proj/projection_matrix{}.npy".format(i))
    #     print("projection matrix{}: ".format(i),lidar_data)
    
    # file_number = 2
    # path_lidar = "calib_checker/lidar/02/lidar_{}.pcd".format(file_number)
    # path_image = "calib_checker/image/02/image_{}.jpg".format(file_number)
    # corner_2d = extract_corner_points(path_image)
    # print("선택된 2D 코너: ",corner_2d)
    # print("2D type: ",np.array(corner_2d))
    # corner_3d = extract_3d_points(path_lidar)
    # print("선택된 3D 좌표: ",corner_3d)
    # print("선택된 3d 크기: ", var.s)
    # proj = calibrate(corner_2d, corner_3d)
    # print("projection matrix: ", proj)
    # a = projection(proj, cam2pix, path_lidar, path_image)
    # print(a)
# 
    # np.save("./projection_matrix/02/projection_matrix{}.npy".format(file_number), proj)
    # lidar_data = np.load("projection_matrix/02/projection_matrix{}.npy".format(file_number))
    # print("projection matrix1: ", lidar_data)