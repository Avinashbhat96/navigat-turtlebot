#!/usr/bin/env python

import rospy
import numpy as np
from math import atan2, sqrt
from sensor_msgs.msg import LaserScan                               # Importing Laser datas
from goal_publisher.msg import PointArray                           # Importing goal points published by goal_publisher
from gazebo_msgs.msg import ModelStates                             # Importing Gazebo_msgs inorder to find positions and orientation
from tf.transformations import euler_from_quaternion                # For converting quaternions to radians
from geometry_msgs.msg import Twist                                 # For publishing velocity to the turtlebot


class MyRobo():

    #Initializing variables, subscriber and publisher
    def __init__(self):
        self.x1 = self.y1 = 0
        self.i = 0
        self.points_to_reach = []
        self.x2_points = []
        self.y2_points = []
        self.theta = self.dist = self.yaw = 0
        self.f = self.l = self.r = False
        self.goals = rospy.Subscriber('/goals', PointArray, self.goal_callback)
        self.points = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_points)
        self.scan_data = rospy.Subscriber('/scan', LaserScan, self.get_data)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.speed = Twist()
#        self.avoid_n_move()
        self.rate = rospy.Rate(10)

    #Callback function to obtain published goal points and sorting it
    def goal_callback(self,goal_points):
        self.points_to_reach = goal_points.goals
        self.points_to_reach.sort(key = lambda x: atan2((x.y - 0),(x.x -0)))
        self.assign_xy()
#        print self.points_to_reach
        return self.points_to_reach

    #assigning sorted points into a seperate array
    def assign_xy(self):
        self.x2_points = []
        self.y2_points = []
        for i in range(20):
            self.x2_points.append(self.points_to_reach[i].x)
            self.y2_points.append(self.points_to_reach[i].y)
        return self.x2_points, self.y2_points

    # callback function to octain present x and y coordinates, orientation of the turtlebot
    def get_points(self,present_point):
        self.x1 = present_point.pose[1].position.x
        self.y1 = present_point.pose[1].position.y
        quat = present_point.pose[1].orientation
        #ModelStates provides orientation in quaternion and we are calculating orientation from it
        #eluer_from_quaternion 3 orientation values and here we need only yaw values
        #Basic idea is obtained from "https://answers.ros.org/question/69754/quaternion-transformations-in-python/"
        (roll, pitch, self.yaw) = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        return self.yaw

    #Callback function for Laserdata and Initializing front, left and right values
    def get_data(self,laser_data):
        las_data = list(laser_data.ranges)
        front = min(min(las_data[0:20]),min(las_data[340:359]),10)
        left = min(min(las_data[80:120]),10)
        right = min(min(las_data[240:300]),10)
        self.f = self.l = self.r = False
        # if obstacle not found in the specified region, then boolean values of front,left and right values will become true
        if front > .5:
            self.f = True#no obstacles in the front
        if left > .5:
            self.l = True#no obstacle in the left
        if right > .5:
            self.r = True#no obstacles in the right
        self.avoid_n_move()
        return self.f, self.l, self.r

    #function to calculate distance between  turtlebot and destination
    def get_distnangle(self,i):
        #print self.x2_points
        x_d = self.x2_points[self.i] - self.x1
        y_d = self.y2_points[self.i] - self.y1
        self.theta = atan2(y_d, x_d)
        self.dist = sqrt(pow(x_d,2) + pow(y_d, 2))
        return self.theta, self.dist

    #function that makes robot to run, when it finds an obstacle turns right and moves around the object
    #In order to frame some conditions, I have referred and got some idea from "https://www.theconstructsim.com/wall-follower-algorithm/"
    def avoid_n_move(self):
        if self.i < 20:
    #        print self.i, self.x2_points[self.i], self.y2_points[self.i]
            self.get_distnangle(self.i)
            angle_to_goal = abs(self.theta - self.yaw)
            theta_degree = np.mod(np.rad2deg(self.theta),360)
            yaw_degree = np.mod(np.rad2deg(self.yaw),360)
    #        print theta_degree, yaw_degree
            if self.dist > 0.2:
                if self.f == True and self.l == True and self.r == True:
                    if angle_to_goal > 0.1:
                        #sufficient angle correction by knowing angle difference
                        if (theta_degree - yaw_degree) > 0 and (theta_degree - yaw_degree) < 180:
                            self.speed.linear.x = 0
                            self.speed.angular.z = 0.2
                            print "Turning towards goal"
                            #print "1"
                        else:
                            self.speed.linear.x = 0
                            self.speed.angular.z = -0.2
                            print "Turning towards goal"
                            #print "2"
                    else:
                        self.speed.linear.x = 0.2
                        self.speed.angular.z = 0
                        print "Moving towards goal"
                        #print "3"
                elif self.f == False and self.l == True and self.r == True:
                    self.speed.linear.x = 0
                    self.speed.angular.z = 0.2
                    print "Found obstacle and turning left"
                    #print "4"
                elif self.f == True and self.l == True and self.r == False:
                    self.speed.linear.x = 0.2
                    self.speed.angular.z = 0
                    print "Moving towards goal or along the wall"
                    #print "5"

                elif self.f == True and self.l == False and self.r == True:
                    if angle_to_goal > 0.1:
                        if (theta_degree - yaw_degree) > 0 and (theta_degree - yaw_degree) < 180:
                            self.speed.linear.x = 0
                            self.speed.angular.z = 0.2
                            print "Turning towards goal"
                            #print "6"
                        else:
                            self.speed.linear.x = 0
                            self.speed.angular.z = -0.2
                            print "Turning towards goal"
                            #print "7"
                    else:
                        self.speed.linear.x = 0.2
                        self.speed.angular.z = 0
                        print "Moving towards goal"
                        #print "8"

                elif self.f == False and self.l == True and self.r == False:
                    self.speed.linear.x = 0
                    self.speed.angular.z = 0.2
                    print "Found obstacle and turning left"
                    #print "9"
                elif self.f == False and self.l == False and self.r == True:
                    self.speed.linear.x = 0
                    self.speed.angular.z = 0.2
                    print "Found obstacle and turning left"
                    #print "10"
                elif self.f == False and self.l == False and self.r == False:
                    self.speed.linear.x = 0
                    self.speed.angular.z = 0.2
                    print "Found obstacle and turning left"
                    #print "11"
                elif self.f == True and self.l == False and self.r == False:
                    if angle_to_goal > 0.1:
                        if (theta_degree - yaw_degree) < 180:
                            self.speed.linear.x = 0
                            self.speed.angular.z = 0.2
                            print "Turning towards goal"
                            #print "12"
                        else:
                            self.speed.linear.x = 0
                            self.speed.angular.z = -0.2
                            print "Turning towards goal"
                            #print "13"
                    else:
                        self.speed.linear.x = 0.2
                        self.speed.angular.z = 0
                        print "Moving towards goal"
                        #print "14"
                self.pub.publish(self.speed)
                self.rate.sleep()
            else:
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                self.i += 1
                print "reached goal number {}".format(self.i)
                if self.i == 20:
                    print "all points are reached"
                self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('turtle_avoiding_object')
    MyRobo = MyRobo()
    rospy.spin()
