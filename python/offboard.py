from __future__ import division

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import ParamValue, PositionTarget
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from pynput import keyboard


class Modes:
    POSITION_CONTROL = 0
    VELOCITY_CONTROL = 1


class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.pos = PoseStamped()

        self.vel_global = TwistStamped()

        self.mode = Modes.POSITION_CONTROL

        self.vel_local = PositionTarget()

        self.vel_local.type_mask = PositionTarget.IGNORE_YAW

        self.pos_setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        self.vel_local_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)             # prawdopodobnie da sie wszystkim sterowac

        # send setpoints in seperate thread to better prevent failsafe
        self.drone_control_thread = Thread(target=self.control_drone, args=())
        self.drone_control_thread.daemon = True
        self.drone_control_thread.start()

        self.v_x = 0
        self.v_y = 0
        self.v_z = 0
        self.v_yaw = 0
        self.speed = 0.5

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def control_drone(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        self.vel_local.header = Header()
        self.vel_local.header.frame_id = "base_footprint"

        self.vel_global.header = Header()
        self.vel_global.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            if self.mode == Modes.POSITION_CONTROL:
                self.pos.header.stamp = rospy.Time.now()
                self.pos_setpoint_pub.publish(self.pos)
            elif self.mode == Modes.VELOCITY_CONTROL:
                self.vel_local.header.stamp = rospy.Time.now()
                self.vel_local_pub.publish(self.vel_local)


            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 180  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, 0.5):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    def take_off(self, z, azimuth, timeout, max_error):
        self.set_position(0, 0, z, azimuth, timeout, max_error)

    def set_velocity(self, v_x, v_y, v_z, v_yaw):
        self.mode = Modes.VELOCITY_CONTROL

        self.vel_local.velocity.x = v_x
        self.vel_local.velocity.y = v_y
        self.vel_local.velocity.z = v_z

        self.vel_local.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.vel_local.yaw_rate = v_yaw


    def set_position(self, x, y, z, azimuth, timeout, max_error):
        self.mode = Modes.POSITION_CONTROL
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z

        yaw = math.radians(azimuth)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, max_error):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))
        return reached

    def on_press(self, key):
        try:
            if self.state.armed:
                if key.char == 'w':
                    self.v_x = self.speed
                elif key.char == 's':
                    self.v_x = -self.speed

                if key.char == 'a':
                    self.v_y = self.speed
                elif key.char == 'd':
                    self.v_y = -self.speed

                if key.char == 'z':
                    self.v_z = self.speed
                elif key.char == 'x':
                    self.v_z = - self.speed

                if key.char == 'q':
                    self.v_yaw = self.speed
                elif key.char == 'e':
                    self.v_yaw = - self.speed

                self.set_velocity(self.v_x, self.v_y, self.v_z, self.v_yaw)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key == keyboard.Key.esc:
                return False
            elif key.char == 'm':
                self.increase_speed()
            elif key.char == 'm':
                self.decrease_speed()
            elif key.char == '1':
                self.set_arm(True, 5)
            elif key.char == '2':
                self.set_arm(False, 5)

            if self.state.armed:
                if key.char == 't':
                    self.take_off(1.5, 180, 20, 0.5)
                elif key.char == 'r':
                    self.rtl()
                elif key.char == 'l':
                    self.land()
                elif key.char == 'w' or key.char == 's':
                    self.v_x = 0
                elif key.char == 'a' or key.char == 'd':
                    self.v_y = 0
                elif key.char == 'z' or key.char == 'x':
                    self.v_z = 0
                elif key.char == 'q' or key.char == 'e':
                    self.v_yaw = 0

                self.set_velocity(self.v_x, self.v_y, self.v_z, self.v_yaw)
            #rospy.loginfo(str(self.v_x) + " " + str(self.v_y) + " " + str(self.v_z) + " " + str(self.v_yaw) + " " + str(
            #    self.speed))

        except AttributeError:
            pass


    def land(self):
        self.set_mode("AUTO.LAND", 5)

    def rtl(self):
        self.set_mode("AUTO.RTL", 5)

    def increase_speed(self):
        self.speed += 0.2

    def decrease_speed(self):
        if self.speed > 0.2:
            self.speed -= 0.2


    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)
        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        #self.set_arm(True, 5)

        rospy.loginfo("run mission")
        positions = ((0, 0, 2), (2, 2, 5), (0, 0, 5))

        #for i in xrange(len(positions)):
        #    self.reach_position(positions[i][0], positions[i][1],
        #                        positions[i][2], 30)

        #self.take_off(4, 180, 20, 0.5)

        #self.tearDown()

        #self.set_velocity(0, 0.3, 1, 1)

        #rospy.sleep(5)

        #self.set_position(0, 0, 5, 180, 20, 0.5)

        #self.set_mode("AUTO.LAND", 5)
        #self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        #self.set_arm

        with keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun("magisterka", 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
