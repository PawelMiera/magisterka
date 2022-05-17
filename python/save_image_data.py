import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import numpy as np
import os
from geometry_msgs.msg import PoseStamped
import queue
from threading import Thread
from datetime import datetime



class DataSaver:
    def __init__(self):
        self.frame = None
        self.depth_frame = None
        self.bridge = CvBridge()
        self.msg = None

        self.local_position = PoseStamped()
        self.local_position_array = None

        rospy.init_node('data_saver')

        rospy.Subscriber("/camera/rgb/image_raw", Image, self.frame_callback)  # /laser/scan

        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)  # /laser/scan

        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)

        self.new_frame = False

        self.q = queue.Queue()

        self.close = False

        self.start_saving = False

        thread = Thread(target=self.saving_thread)
        thread.start()
        print("ok")

        out_image = np.zeros((480, 1281, 3), np.uint8)

        while True:
            if self.frame is not None:
                out_image[:480, :640] = cv2.resize(self.frame, (640, 480))

            if self.depth_frame is not None:
                depth = self.depth_frame.copy()
                depth[np.isnan(depth)] = 15
                norm_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

                norm_depth = cv2.cvtColor(cv2.resize(norm_depth, (640, 480)), cv2.COLOR_GRAY2BGR)
                out_image[:480, 641:1281] = norm_depth

            cv2.imshow("drone view", out_image)

            if self.start_saving:

                if self.new_frame and self.frame is not None and self.depth_frame is not None and \
                        self.local_position_array is not None:
                    self.new_frame = False
                    self.q.put((self.frame.copy(), self.depth_frame.copy(), self.local_position_array.copy()))

            key = cv2.waitKey(1)
            if key == 98:           #b
                self.close = True
                break

            elif key == 114:           #r
                self.start_saving = not self.start_saving
                print("Recording", self.start_saving)


        # rospy.spin()

    def local_position_callback(self, data):
        self.local_position = data
        try:
            self.local_position_array = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z,
                                                  data.pose.orientation.x, data.pose.orientation.y,
                                                  data.pose.orientation.z])

            #print(self.local_position_array)
        except AttributeError:
            pass

    def saving_thread(self):
        ind = 0
        directory = "saved_data"
        d = datetime.now().strftime("%m_%d_%H_%M_%S")
        directory = os.path.join(directory, d)

        try:
            os.mkdir(directory)
        except OSError as error:
            print(error)

        while True:

            if self.q.qsize() > 0:
                print(self.q.qsize())
                item = self.q.get()

                path = os.path.join(directory, str(ind) + "image.jpg")
                cv2.imwrite(path, item[0])

                path = os.path.join(directory, str(ind) + "depth.npy")
                np.save(path, item[1])

                path = os.path.join(directory, str(ind) + "pos.npy")
                np.save(path, item[2])
                ind += 1

                if self.close:
                    print(self.q.qsize())
            else:
                if self.close:
                    print("done saving")
                    break

    def frame_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.new_frame = True
        except CvBridgeError, e:
            print(e)
        else:
            pass

    def depth_callback(self, msg):
        self.msg = msg
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError, e:
            print(e)
        else:
            pass


if __name__ == "__main__":
    dataSaver = DataSaver()
