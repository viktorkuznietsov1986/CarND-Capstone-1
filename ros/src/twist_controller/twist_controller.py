from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio,max_lat_accel,max_steer_angle, accel_limit,decel_limit): 
        
        self.yaw = YawController(wheel_base, steer_ratio, 0., max_lat_accel, max_steer_angle)
 	
	self.steer = 0.0
	self.throttle = 0.0
	self.brake = 0.0
	self.kp = 0.9
	self.ki = 0.01
	self.kd = 0.4
	self.mn = decel_limit
	self.mx = .5#accel_limit
	self.pid = PID(self.kp,self.ki,self.kd ,self.mn,self.mx)
	self.accel =None
	

    def control(self,lin_vel,ang_vel,curr_vel,sample_time,vehicle_mass, wheel_radius,dbw): 

	self.steer = self.yaw.get_steering(lin_vel,ang_vel,curr_vel) 	
	
	error = lin_vel- curr_vel 
	if lin_vel == 0 and curr_vel ==0:
	    self.throttle = 0
	    self.brake = 700 #prevent rolling forward 
	    
	if dbw: 
	    accel_target =self.pid.step(error,sample_time ) 
	    
	
	    if accel_target >=0 :
	        self.throttle = accel_target
	        self.brake = 0.0
	    else:
	        self.throttle = 0.0
		self.brake = -accel_target*vehicle_mass*wheel_radius
	   
        return self.throttle, self.brake, self.steer
 
