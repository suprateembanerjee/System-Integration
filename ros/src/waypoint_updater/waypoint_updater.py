#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math
import numpy as np
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_lane = None
        self.pose = None
        self.waypoints2D = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        
        self.loop()
        
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()
            
    # Find and return the closest waypoint index
    def get_closest_waypoint_id(self):
        
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_index = self.waypoint_tree.query([x,y], 1)[1]
        
        closest_coord = self.waypoints2D[closest_index]
        prev_coord = self.waypoints2D[closest_index - 1]
        
        closest_vec = np.array(closest_coord)
        prev_vec = np.array(prev_coord)
        pos_vec = np.array([x,y])
        
        val = np.dot(closest_vec - prev_vec, pos_vec - closest_vec)
        
        if val > 0:
            closest_index = (closest_index + 1) % len(self.waypoints2D)
        
        return closest_index
    
    # Publish the waypoints
    def publish_waypoints(self):
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)
    
    # Generate a Lane
    def generate_lane(self):
        lane = Lane()
        closest_index = self.get_closest_waypoint_id()
        farthest_index = closest_index + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_index : farthest_index]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_index):
        	lane.waypoints = base_waypoints
        else:
        	lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_index)

        return lane
    
    # Generate decelerated waypoints when a Red Light is detected
    def decelerate_waypoints(self, waypoints, closest_i):
    	
    	temp = []
    	for i, waypoint in enumerate(waypoints):
    		p = Waypoint()
    		p.pose = waypoint.pose
    		stop_i = max(self.stopline_wp_idx - closest_i - 2, 0)
    		dist = self.distance(waypoints, i, stop_i)
    		vel = math.sqrt(2 * MAX_DECEL * dist)
    		if vel < 1.0:
    			vel = 0.0
    		p.twist.twist.linear.x = min(vel, waypoint.twist.twist.linear.x)
    		temp.append(p)

    	return temp
    
    # Pose Callback, processes PoseStamped messages
    def pose_cb(self, msg):
        self.pose = msg
    
    # Waypoint Callback, processes Lane messages
    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints2D:
            self.waypoints2D = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints2D)
            
    # Traffic Callback, processes Int messages
    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data
    
    # Not Implemented.
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        pass
    
    # Returns velocity of vehicle in a given waypoint
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x
    
    # Sets velocity of a vehicle in a given waypoint
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
    
    # Calculates distance between two points in the waypoint
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
