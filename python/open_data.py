import numpy as np
import cv2
import os

directory = os.path.join("saved_data", "03_21_20_58_10")

for i in range(0, 1111):
    try:
        path = os.path.join(directory, str(i) + "image.jpg")
        frame = cv2.imread(path)

        path = os.path.join(directory, str(i) + "depth.npy")
        depth = np.load(path)

        path = os.path.join(directory, str(i) + "pos.npy")
        pos = np.load(path)
    except:
        print("data error")
        continue

    if frame is not None:
        cv2.imshow("rgb", cv2.resize(frame, (640, 480)))
    else:
        print("missing data")

    if depth is not None:
        depth[np.isnan(depth)] = 15
        norm_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        norm_depth = cv2.resize(norm_depth, (640, 480))

        cv2.imshow("depth", norm_depth)
    else:
        print("missing data")

    print(pos, i)
    if cv2.waitKey(1) == 113:
        break





