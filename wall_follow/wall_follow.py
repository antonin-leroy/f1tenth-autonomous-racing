#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 15
kd = 0.1
ki = 0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 
DESIRED_DISTANCE_RIGHT = 0.9 
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 
CAR_LENGTH = 0.50 

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        #drive_topic = '/nav'
        drive_topic = '/vesc/ackermann_cmd_mux/input/navigation'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)#TODO: Publish to drive



    def getRange(self, data, angle):
        # data: single message from topic /scan 
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement

        
        angle_min = data.angle_min
        incr = data.angle_increment
        ecart= angle - angle_min
        index = int(ecart/incr)
        range = data.ranges[index]
        if math.isinf(range) or math.isnan(range) : 
            return 10.0
        if index < 0 or index >= len(data.ranges):
            return 10.0 
        return range


    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        integral += error 
        derive = error-prev_error
        angle = error * kp + ki * integral + kd * derive
        prev_error=error
        #TODO: Use kp, ki & kd to implement a PID controller 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle 
        if 0<abs(angle) <10*((math.pi)/180) : 
            drive_msg.drive.speed = 1.5
        elif 10*((math.pi)/180) <= abs(angle) <= 20*((math.pi)/180) : 
            drive_msg.drive.speed = 1
        else : 
            drive_msg.drive.speed = 0.5
        self.drive_pub.publish(drive_msg)

    def followRight(self, data):
        L = 1
        theta = 42*(math.pi/180)
        b=self.getRange(data,-90*(math.pi/180))
        a=self.getRange(data,(-90*(math.pi/180))+theta)
        alpha = math.atan((a*math.cos(theta)-b)/(a*math.sin(theta)))
        distance=b * math.cos(alpha)
        distancepredite= distance + L*math.sin(alpha)
        error = DESIRED_DISTANCE_RIGHT - distancepredite
        return error

    def lidar_callback(self, data):
        error = self.followRight(data)
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)