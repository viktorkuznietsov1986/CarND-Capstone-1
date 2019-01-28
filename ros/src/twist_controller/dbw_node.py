#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from lowpass import LowPassFilter

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
	
        # TODO: Create `Controller` object
        self.controller = Controller(wheel_base, steer_ratio, max_lat_accel, max_steer_angle,accel_limit,decel_limit)
	
	
        # TODO: Subscribe to all the topics you need to
  	rospy.Subscriber('/twist_cmd',TwistStamped,self.twist_cb)
	rospy.Subscriber('/current_velocity',TwistStamped,self.curr_velo_cb)
	rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_enabled_cb) 
	
        self.dbw_enabled = False
	self.twist = None 
	self.curr_velo = None
	self.low_pass = LowPassFilter(0.5,0.02) 
	self.loop(vehicle_mass,wheel_radius)

    def loop(self, vehicle_mass, wheel_radius):
        rate = rospy.Rate(50) # 50Hz
	error = 0 #to determine best values for pid control parameters...
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

	    sample_time = 1./50 # because of the 50Hz; 
            
	    if self.twist and self.curr_velo and self.dbw_enabled :
		#curr_velocity = self.abs_vec3(self.curr_velo.twist.linear)
		curr_velocity = self.curr_velo.twist.linear.x 
		curr_velocity = self.low_pass.filt(curr_velocity)
		linear_vel = self.twist.twist.linear.x
		#linear_vel = self.abs_vec3(self.twist.twist.linear) #linear velocity seems to be in car coordinates so linear.x
		#should be the same
		angular_vel = self.twist.twist.angular.z
		#abs_angular_vel = self.abs_vec3(self.twist.twist.angular) #angular vel seems to be only in z-direction...; 
		#then all the stuff with sign is unneccessary
		#sign = 1 #the direction of the angular velocity must be taken into regard aswell 
		#if abs_angular_vel >0:
		#    sign = self.dir()
		#angular_vel = sign*abs_angular_vel
	    	throttle, brake, steering, error = self.controller.control(linear_vel,angular_vel,curr_velocity,
								 sample_time,vehicle_mass, wheel_radius,self.dbw_enabled) 
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            #		
		
            	self.publish(throttle, brake, steering)
            rate.sleep()
	rospy.logerr(error)
    def twist_cb(self,msg):
	self.twist = msg 

    def abs_vec3(self,vec):
	return math.sqrt(vec.x**2+vec.y**2+vec.z**2)

    def dbw_enabled_cb(self,msg):
	self.dbw_enabled = msg
    def dir(self):
	#projection of angular velocity on x-y-plane and then turning the vector
	#by 90 degrees to use the scalar product with linear_vel to determine 
	#the direction of the angular velocity 
	scalar = -self.twist.twist.linear.x*self.twist.twist.angular.z+self.twist.twist.linear.z*self.twist.twist.angular.x
	#scalar = linear_vel.x*angular_vel.x + linear_vel.y*angular_vel.y
	if scalar >0:
	    return -1
	if scalar <0:
	    return 1
	else:
	    return 0

    def curr_velo_cb(self,msg):
	self.curr_velo= msg


    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
