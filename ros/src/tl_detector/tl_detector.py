#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from scipy.spatial import KDTree
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
	self.kdTree = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
	rate = rospy.Rate(50)
#	while not rospy.is_shutdown():
#	    if self.pose and self.waypoints:
#	    	self.process_traffic_lights()
#	    rate.sleep()

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
	if not self.kdTree: #create a kdTree as in the waypoint updater
	    waypoints2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
	    self.kdTree = KDTree(waypoints2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
	    #rospy.logerr(light_wp)
            light_wp = light_wp if state ==0 else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
#	if self.waypoints:
        #TODO implement
	# as in waypoint_updater.py:
#	    min_ind = 0
#	    min_dist = self.dist(pose.position,self.waypoints.waypoints[0].pose.pose.position)
	ind = -1
	if self.kdTree:
	    _,ind = self.kdTree.query([pose[0], pose[1]],1)
 #           for i in range(len(self.waypoints.waypoints)):
#	        curr_dist = self.dist(pose.position,self.waypoints.waypoints[i].pose.pose.position)
#	        if curr_dist <min_dist:
#		    min_ind = i
#		    min_dist = curr_dist
#
 	#    return min_ind
	return ind

    def get_closest_waypoint(self,x,y):
	ind = -1
	if self.kdTree:
	    ind = self.kdTree.query([x,y],1)[1]
	return ind

#    def dist(self,pos1,pos2):
#	return math.sqrt((pos1.x-pos2.x)**2+(pos1.y-pos2.y)**2) #-waypoint.pose.pose.position.x)**2 + (pose.position.y-waypoint.pose.pose.position.y)**2)  
    
    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
 #       if(not self.has_image):
 #           self.prev_light_loc = None
 #           return False

#        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        #return self.light_classifier.get_classification(cv_image)
	return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
	min_idx = None
	check = False
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoints):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x,self.pose.pose.position.y)
#	    car_position = self.get_closest_waypoint([self.pose.pose.position.x,self.pose.pose.position.y])
	    #rospy.logerr("pose and waypoints")
        #TODO find the closest visible traffic light (if one exists)
	    min_dist = len(self.waypoints.waypoints)
	 #   min_idx =  
#	    rospy.logerr(len(self.lights))
	    for i in range(len(stop_line_positions)):
	        line_pos = self.get_closest_waypoint(stop_line_positions[i][0],stop_line_positions[i][1])
	        curr_dist = (line_pos - car_position) %len(self.waypoints.waypoints)
	        if curr_dist < min_dist:
		    rospy.logerr(curr_dist)
		    min_dist = curr_dist
		    min_idx = i
		    check = True
	    
        if check: #min_idx:
	   # rospy.logerr("state")
            state = self.get_light_state(self.lights[min_idx])
	    #rospy.logerr(state)
            return self.get_closest_waypoint(stop_line_positions[min_idx][0],stop_line_positions[min_idx][1]), state
        #self.waypoints = None
	#colour = self.lights[min_idx] # right way to get the colour?
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
