#!/usr/bin/python

import rospy
from ackermann_msgs.msg import *
from sensor_msgs.msg import *
import sys, math
from std_msgs.msg import Float32MultiArray

class ObjectDetectorNode:

    def __init__(self):

        self.refresh_count = 0
        self.refresh_count = rospy.get_param('refresh_count', default=self.refresh_count)
        self.desired_distance = 0.4
        self.desired_distance = rospy.get_param('desired_distance', default=self.desired_distance)
        self.threshold = 0.08
        self.desired_distance = rospy.get_param('desired_distance', default=self.desired_distance)

        self.before_e = 0

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)

        self.vision = rospy.Subscriber("/scan", LaserScan, self.drive_control)

        rospy.init_node("object_detector_node")
        self.header = std_msgs.msg.Header()
        self.header.stamp = rospy.Time.now()
        # self.drive_forward = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=0.0, speed=1.5))
        # self.drive_backward = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=0.0, speed=-1.5))
        # self.STOP = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=0.0, speed=0.0))
        rospy.loginfo("Moving forward...")

    def drive_control(self, msg):
        self.refresh_count += 1
	if int(sys.argv[3]) == 1:
            pts = msg.ranges[170:200]
	elif int(sys.argv[3]) == 0:
	    pts = msg.ranges[880:910]
	else:
	    rospy.loginfo("ERROR: INVALID WALL VARIABLE")
        error = self.desired_distance - min(pts)
        if self.refresh_count == 1:
            self.before_e = error
        # new_steering_angle = self.bang_bang_controller(error)
        # new_steering_angle = self.bang_bang_with_threshold(error)
        new_steering_angle = self.pid_controller(error)
        drive_command = AckermannDriveStamped(self.header, AckermannDrive(steering_angle = new_steering_angle, speed = float(sys.argv[4])))
        self.drive_pub.publish(drive_command)

    def bang_bang_controller(self, error):
        rospy.loginfo(error)
        if error < 0:
            return -1
        elif error > 0:
            return 1
        else:
            return 0

    def bang_bang_with_threshold(self, error):
        rospy.loginfo(error)
        if abs(error) > self.threshold:
            if error < -self.threshold:
                return -1
            else:
                return 1
        else:
            return 0

    # ONE POINT
    def pid_controller(self, error):
        rospy.loginfo(error)
	third = int(sys.argv[3])
	is_right = True
	if third == 0:
            is_right = False
        return self.proportion(error,is_right) + self.differential(error, 40.0)

    def proportion(self, error, is_right):
        if is_right:
            return float(sys.argv[1]) * float(error)
        else:
            return -1 * float(sys.argv[1]) * float(error)

    def differential(self, error, hz):
        delta_time = 1 / hz
        edot = float(error - self.before_e) / float(delta_time)
        kd = float(sys.argv[2])
        self.before_e = error
        return (kd / math.fabs(kd)) * edot

    # TWO POINT
    def proportionTwo(self, error, angle, is_right):
        error = (error) * math.cos(angle)
        if is_right:
            return float(sys.argv[1]) * error
        else:
            return -1 * float(sys.argv[1])* error

    def differentialTwo(self, error, angle, hz):
        error = error * math.cos(angle)
        delta_time = 1 / hz
        edot = float(error - self.before_e) / float(delta_time)
        kd = float(sys.argv[2])
        self.before_e = error
        return (kd / math.fabs(kd)) * edot


if __name__ == "__main__":

    node = ObjectDetectorNode()

    rospy.spin()
