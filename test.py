from PIL import Image
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import image


img_data = np.asarray(Image.open("camera/37.png"))
img = image.imread("camera/37.png")
print(img.shape)
disp = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2RGB)
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title('Select 2D Image Points')
ax.set_axis_off()
ax.imshow(disp)
# plt.imshow(disp)
# plt.show()

picked, corners = [], []

def onclick(event):
    x = event.xdata
    y = event.ydata
    if (x is None) or (y is None): return

    # Display the picked point
    picked.append((x, y))
    corners.append((x, y))
    print(picked[-1])

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
print(corners)