from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio,max_lat_accel,max_steer_angle, accel_limit,decel_limit): #*args, **kwargs):
        # TODO: Implement
        self.yaw = YawController(wheel_base, steer_ratio, 0., max_lat_accel, max_steer_angle)
 
	
	self.steer = 0.0
	self.throttle = 0.0
	self.brake = 0.0
	self.kp = 0.7
	self.ki = 0.6
	self.kd = 0.1
	self.mn = decel_limit
	self.mx = 0.3
	self.pid = PID(self.kp,self.ki,self.kd ,self.mn,self.mx)
	self.accel =None
	

    def control(self,lin_vel,ang_vel,curr_vel,sample_time,vehicle_mass, wheel_radius,dbw): # *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	self.steer = self.yaw.get_steering(lin_vel,ang_vel,curr_vel) 	
	
	error = lin_vel- curr_vel 
#	if lin_vel == 0 and curr_vel ==0:
#	    self.throttle = 0
#	    self.brake = 700 #prevent rolling forward - different way of implementation might be necessary... 
#	    int_err = 0
	if dbw: #elif dbw:
	    accel_target =self.pid.step(error,sample_time ) #error,sample_time - might be 1/50 s 
	    int_err = self.pid.error() #to determine best values for kp,ki,kd
	#next step: convert target acceleration into throttle and brake values
	    if accel_target >=0 :
	        self.throttle = accel_target
	        self.brake = 0.0
	    else:
	        self.throttle = 0.0
		self.brake = -accel_target*vehicle_mass*wheel_radius
	else:
	   int_err = 0    
        return self.throttle, self.brake, self.steer, int_err
 
