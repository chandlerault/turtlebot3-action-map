#!/usr/bin/env python3
"""
 The explore node causes the robot to explore the environment autonomously while mapping the world
 SUBSCRIBERS:
  sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randrange
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import time
import matplotlib.pyplot as plt
import cv2


class Explore:

    def __init__(self):
        """ Initialize environment
        """
        # Initialize rate:
        self.rate = rospy.Rate(1)

        # Simple Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready") 

        self.x = -10
        self.y = -10
        self.width = 384
        self.height = 384
        self.resolution = .05
        #self.robotX
        #self.robotY
        self.completion = 0

        self.tf_listener = tf.TransformListener()
        self.map_frame = 'map'

        try:
            self.tf_listener.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.map_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")


        rospy.logdebug("initialized") 


        # Initialize subscribers:
        self.map = OccupancyGrid()
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.count = 0
        time.sleep(8)


    def map_callback(self, data):
        """ Callback function for map subscriber.
        Subscribes to /map to get the OccupancyGrid of the map.
        """
        rospy.logdebug("map callback") 

        valid = False
        self.resolution = data.info.resolution
        self.height = data.info.height
        self.width = data.info.width
        #self.robotX = data.info.x
        #self.robotY = data.info.y
        print(data.info)

 
        global_map = np.array(data.data).reshape((self.height,self.width))
        global_map = global_map.astype('float64')

        #global_map = np.where(global_map < 50, 255, 0)
        global_map = np.where(global_map <= -1, -1, global_map)
        # saving the final output 
        # as a PNG file

        
        p, r = self.get_odom()

        px_min, py_min = self.cartesian_to_pixel(p[0]-3, p[1]-3)
        px_max, py_max = self.cartesian_to_pixel(p[0]+3, p[1]+3)

        px_min2, py_min2 = self.cartesian_to_pixel(p[0]-2, p[1]-2)
        px_max2, py_max2 = self.cartesian_to_pixel(p[0]+2, p[1]+2)

        viewx = int((px_max2-px_min2)/2)
        viewy = int((py_max2-py_min2)/2)


        print("angle: ")
        print(90-np.degrees(r))

        print(global_map.dtype)
        px, py = self.cartesian_to_pixel(p[0], p[1])
        

        global_map[py-3:py+3, px-3:px+3] = 255   

        local_map = global_map[py_min:py_max, px_min:px_max]
        w, h = local_map.shape
        M = cv2.getRotationMatrix2D((int(h/2),int(w/2)), 90-np.degrees(r), 1)
        w, h = local_map.shape
        local_map = cv2.flip(local_map, 0)
        print(local_map.shape)
        rotated = cv2.warpAffine(local_map, M, (w, h))
        rotated = rotated[int(w/2)-viewy:int(w/2)+viewy, int(h/2)-viewx:int(h/2)+viewx]
        #rotated = cv2.flip(rotated, 1)
        w,h = rotated.shape
        self.dist_map(rotated, int(h/2),int(w/2))
        print(px,py)
        global_map = cv2.flip(global_map, 0)

        plt.imsave('local.jpeg', local_map)
        plt.imsave('rotated.jpeg', rotated)
        plt.imsave('map.jpeg', global_map)


        
        while valid is False:
            map_size = randrange(len(data.data))
            self.map = data.data[map_size]
            
            edges = self.check_neighbors(data, map_size)
            if self.map != -1 and self.map <= 0.2 and edges is True:
                valid = True
            
        row = map_size / self.height
        col = map_size % self.width
        

        self.x, self.y = self.pixel_to_cartesian(col, row)  # column * resolution + origin_x
        # row * resolution + origin_x
        
        if self.completion % 2 == 0:
            self.completion += 1
            # Start the robot moving toward the goal
            #self.set_goal(self.x,self.y)
    

    def set_goal(self, x, y):
        """ Set goal position for move_base.
        """
        rospy.logdebug("Setting goal")

        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        #rospy.logdebug(f'goal: {goal.target_pose.pose.position.x, goal.target_pose.pose.position.y}')
        self.move_base.send_goal(goal)

        self.move_base.wait_for_result(rospy.Duration(5))

        if(self.move_base.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.logdebug("You have reached the destination")
        else:
            rospy.logdebug("Failed")

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (trans, rotation[2])

    def goal_status(self, status, result):
        """ Check the status of a goal - goal reached, aborted,
        or rejected.
        """
        self.completion += 1

        # Goal reached
        if status == 3:
            rospy.loginfo("Goal succeeded")

        # Goal aborted
        if status == 4:
            rospy.loginfo("Goal aborted")

        # Goal rejected
        if status == 5:
            rospy.loginfo("Goal rejected")


    def check_neighbors(self, data, map_size):
        """ Checks neighbors for random points on the map.
        """
        unknowns = 0
        obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                row = x * 384 + y
                try:
                    if data.data[map_size + row] == -1:
                        unknowns += 1
                    elif data.data[map_size + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
        if unknowns > 0 and obstacles < 2:
            return True
        else:
            return False

    def pixel_to_cartesian(self, col, row):
        return col * self.resolution - 10, row * self.resolution - 10  # column * resolution + origin_x

    def cartesian_to_pixel(self, x, y):
        return int((x + 10)/self.resolution), int((y+10)/self.resolution)
    
    def dist_map(self, global_map, px, py):
        map = global_map.copy()
        h,w = map.shape

        for i in range(h):
            for j in range(w):
                if global_map[i,j] != -1:
                    map[i,j] = ((j-py)**2 + (i-px)**2)**.5
                else: map[i,j] = -1
        plt.imsave('distmap.jpeg', map)
        return map

def main():
    """ The main() function """
    rospy.init_node('explore', log_level=rospy.DEBUG)
    Explore()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException