from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio,max_lat_accel,max_steer_angle): #*args, **kwargs):
        # TODO: Implement
        self.yaw = YawController(wheel_base, steer_ratio, 0., max_lat_accel, max_steer_angle)
 
	
	self.steer = 0.0
	self.throttle = 0.0
	self.brake = 0.0
	self.kp = 0.7
	self.ki = 0.6
	self.kd = 0.1
	self.mn = 0.0
	self.mx = 1.0
	self.pid = PID(self.kp,self.ki,self.kd ,self.mn,self.mx)
	self.accel =None
	

    def control(self,lin_vel,ang_vel,curr_vel,sample_time,vehicle_mass, wheel_radius): # *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	self.steer = self.yaw.get_steering(lin_vel,ang_vel,curr_vel) 	
	error = lin_vel- curr_vel 
	accel_target =self.pid.step(error,sample_time ) #error,sample_time - might be 1/50 s or probably calculated using timestamps
	int_err = self.pid.error()
	#next step: somehow convert target acceleration into throttle and brake values
	if accel_target >=0 :
	    if accel_target <=1: #perhaps some other value? (and the other calculation for self.accel)
		self.throttle = accel_target
	    else:
		self.throttle = 1. 
	    self.brake = 0.0
	else:
	    self.throttle = 0.0
#            if -accel_target >5.:
 #               accel_target = - 5.

	    self.brake = -accel_target*vehicle_mass*wheel_radius
	    
        return self.throttle, self.brake, self.steer, int_err
 
