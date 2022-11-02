
import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
from matplotlib import image
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

lidar_data = np.load("cam_lidar_data/lidar/0.npy")
shape_data = lidar_data.shape

img = np.asarray(Image.open("cam_lidar_data/camera/37.png"))

print("lidar_data")
print(lidar_data)
print("lidar_data size",shape_data)

print(img)
print("img size: ",np.shape(img))
print(cv2.__version__)
disp = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2RGB)

img_data = image.imread("cam_lidar_data/camera/37.png")

class LineBuilder:
    def __init__(self, line):
        self.line = line
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)

    def __call__(self, event):
        print('click', event)
        if event.inaxes!=self.line.axes: return
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        self.line.set_data(self.xs, self.ys)
        self.line.figure.canvas.draw()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title('Select 2D Image Points')
ax.set_axis_off()

line, = ax.plot([0], [0])  # empty line
linebuilder = LineBuilder(line)

# plt.show()
print(disp)
plt.imshow(disp)
# ax.imshow(disp)

# picked, corners = [], []
# def onclick(event):
#     x = event.xdata
#     y = event.ydata
#     if (x is None) or (y is None): return

#     # Display the picked point
#     picked.append((x, y))
#     corners.append((x, y))
#     rospy.loginfo('IMG: %s', str(picked[-1]))

#     if len(picked) > 1:
#         # Draw the line
#         temp = np.array(picked)
#         ax.plot(temp[:, 0], temp[:, 1])
#         ax.figure.canvas.draw_idle()

#         # Reset list for future pick events
#         del picked[0]

# # Display GUI
# fig.canvas.mpl_connect('button_press_event', onclick)
# plt.show()fv

def extract_corner_points():
    pass

def extract_3d_points():
    pass

def calibrate():
    pass

if __name__ == '__main__':
    pass

