from yaw_controller import YawController
from pid import PID
import rospy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel,max_steer_angle):
	self.wheel_base = wheel_base
	self.steer_ratio = steer_ratio
	self.max_lat_accel = max_lat_accel
	self.max_steer_angle = max_steer_angle
	self.min_speed = 0

	self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

	self.pid_controller = PID()
	self.prev_time_secs = None

	self.prev_steering_angle = 0.0

    def control(self, linear_velocity, angular_velocity, current_linear_velocity, current_angular_velocity, is_dbw_enabled, time_secs):

	if is_dbw_enabled == False:
		return 0., 0., 0.

        if self.prev_time_secs == None:
		self.prev_time_secs = time_secs
		return 1., 0., 0.
	
	delta_t = (time_secs - self.prev_time_secs) / 1000000000.0
	#rospy.logwarn("delta_t: %s", delta_t)
	self.prev_time_secs = time_secs
	
	desired_steering_angle = self.get_steering_angle(linear_velocity, angular_velocity)
	current_steering_angle = self.get_steering_angle(current_linear_velocity, current_angular_velocity)

	steering_angle = self.yaw_controller.get_steering(linear_velocity.x, angular_velocity.z, current_linear_velocity.x)

	error = current_steering_angle - self.prev_steering_angle
	rospy.logwarn("current, prev, error: %s, %s, %s", current_steering_angle, self.prev_steering_angle, error)
	p_val = self.pid_controller.step(error, 0.02)
	
	
	self.prev_steering_angle = steering_angle

	normalized_angle = self.normalize_angle(steering_angle - p_val)

	# Return throttle, brake, steer
        return 1., 0., normalized_angle



    """
	Calculates steering angle given linear and angular velocity
    """
    def get_steering_angle(self, linear_velocity, angular_velocity):
		radius = linear_velocity.x / angular_velocity.z if abs(angular_velocity.z) > 0. else 0.
		steering_angle = self.yaw_controller.get_angle(radius) if abs(radius) > 0. else 0.

		if steering_angle < -1:
			steering_angle = -1
		elif steering_angle > 1:
			steering_angle = 1

		return steering_angle

    """
	Normalize angle between -1 and 1
    """
    def normalize_angle(self, angle):
	if angle < -0.436332:
		return -0.436332
	elif angle > 0.436332:
		return 0.436332

	return angle
