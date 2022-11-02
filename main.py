
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

lidar_data = np.load("lidar/0.npy")
shape_data = lidar_data.shape

print("lidar_data")
print(lidar_data)
print("lidar_data size",shape_data)

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

# linebuilder = LineBuilder(line)

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


def extract_3d_points():
    pass

def calibrate():
    pass

if __name__ == '__main__':
    path = 'camera/37.png'
    pass

