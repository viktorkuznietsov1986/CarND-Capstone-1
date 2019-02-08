#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np
import math

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

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number
MAX_DECEl = 0.6

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level = rospy.INFO)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) 
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None
	self.max_vel= None
	self.pose  = None
	self.waypoints_2d = None
	self.kd_tree = None
	self.red_light_waypoint = -1
	self.num_waypoints = None			
	
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
	    if self.base_waypoints and self.pose and self.kd_tree:
	        self.set_final_waypoints()
	    rate.sleep()

    def pose_cb(self, msg):
	self.pose = msg
	
	
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
	if not self.max_vel:
	    self.max_vel = self.get_waypoint_velocity(self.base_waypoints.waypoints[0]) 
	self.num_waypoints = len(waypoints.waypoints)
	if not self.waypoints_2d:
	    self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]	   
	    self.kd_tree = KDTree(self.waypoints_2d)
	    

    def find_waypoint_ahead(self):
	dist,ind = self.kd_tree.query([self.pose.pose.position.x,self.pose.pose.position.y])
	direction_waypoint = np.array([self.waypoints_2d[ind][0]-self.pose.pose.position.x,self.waypoints_2d[ind][1]-self.pose.pose.position.y])
	heading_direction = np.array([self.pose.pose.orientation.x, self.pose.pose.orientation.y])
	if np.dot(direction_waypoint,heading_direction)<0:
           ind +=1 
	   ind = ind%len(self.waypoints_2d)
	return ind

    def set_final_waypoints(self):
	lane = self.new_lane()
	self.final_waypoints_pub.publish(lane)

    def new_lane(self):
	lane = Lane()
	next_idx = self.find_waypoint_ahead()
	final_idx = next_idx + LOOKAHEAD_WPS
	base_waypoints = self.base_waypoints.waypoints[next_idx:final_idx+1]
	if self.red_light_waypoint ==-1 or (self.red_light_waypoint >final_idx) or self.red_light_waypoint <next_idx:
#	if there is no red light ahead just use the base_waypoints
	    lane.waypoints = base_waypoints
	else:
	# else slowly decrease velocity to stop
	    lane.waypoints = self.decrease_vel(base_waypoints,next_idx)
	return lane

    def decrease_vel(self,lane_waypoints,next_idx): 
	vel = self.max_vel 
	waypoints = []
	stop_idx = max(self.red_light_waypoint-next_idx-2,0)
	for i in range(LOOKAHEAD_WPS):
	    wp = Waypoint()
	    wp.pose = self.base_waypoints.waypoints[next_idx + i].pose
	    dist = self.distance(lane_waypoints,i,stop_idx) 
	    vel = min(math.sqrt( 2*MAX_DECEl* dist),self.max_vel) 
	    if vel <1:
		vel =0
	    wp.twist.twist.linear.x =vel 
	    waypoints.append(wp)
	return waypoints




    def traffic_cb(self, msg):
        self.red_light_waypoint = msg.data
	

    def obstacle_cb(self, msg):
	#not used...        
	pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
