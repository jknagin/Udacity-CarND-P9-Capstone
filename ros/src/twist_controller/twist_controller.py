import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        # Yaw controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        
        # Throttle PID controller
        kp = 0.3
        ki = 0.1
        kd = 0.0
        min_throttle_value = 0.0
        max_throttle_value = 0.2
        self.throttle_controller = PID(kp, ki, kd, min_throttle_value, max_throttle_value)
        
        # Velocity LPF
        tau = 0.5  # LPF time constant
        dt = 0.02  # sampling period
        self.vel_lpf = LowPassFilter(tau, dt)

    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled, sample_time):
        # Return throttle, brake, steer
        
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        # Calculate steering using low pass filtered current_vel
#         current_vel = self.vel_lpf.filt(current_vel)
        vel_error = linear_vel - current_vel
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # Calculate throttle
        throttle = self.throttle_controller.step(vel_error, sample_time)

        # Calculate brake
        brake = 0
        
        if linear_vel == 0.0 and current_vel < 0.1:  # Want to stop moving
            throttle = 0
            brake = 700  # N*m
        elif throttle < 0.1 and vel_error < 0:  # Moving faster than want to be
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)  # Decel is a function of vel error
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
        
        return throttle, brake, steering
    