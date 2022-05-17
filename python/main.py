import cv2
import numpy as np

i1 = cv2.imread("saved_data/0image.jpg")
i1 = cv2.resize(i1, (640, 480))

depth = np.load("saved_data/0depth.npy")
depth[np.isnan(depth)] = 15
norm_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

norm_depth = cv2.cvtColor(cv2.resize(norm_depth, (640, 480)), cv2.COLOR_GRAY2BGR)

out_image = np.zeros((480, 1281, 3), np.uint8)

out_image[:480, :640] = i1

out_image[:480, 641:1281] = norm_depth

cv2.imshow("XXX", out_image)
key = cv2.waitKey(0)
print(key)

a = 2