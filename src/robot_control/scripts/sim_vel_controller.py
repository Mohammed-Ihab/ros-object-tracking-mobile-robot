#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point

# -----------------------------
# PID CLASS
# -----------------------------
class PID:
    def __init__(self, kp, ki, kd, max_val):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_val = max_val
        self.prev_error, self.integral = 0.0, 0.0
        self.last_time = None

    def compute(self, error):
        current_time = rospy.Time.now()
        if self.last_time is None:
            self.last_time = current_time
            return 0.0
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        if dt <= 0: return 0.0

        P = self.kp * error
        self.integral = max(min(self.integral + error * dt, 0.5), -0.5) 
        I = self.ki * self.integral
        D = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        output = P + I + D
        # Note: We still clamp inside PID, but the final sum (PID+FF) needs clamping too
        return max(min(output, self.max_val), -self.max_val)

# -----------------------------
# CONFIGURATION
# -----------------------------
DESIRED_DISTANCE = 0.5
LIN_DEADBAND = 0.08
ANG_DEADBAND = 0.05

# 1. HARDWARE LIMITS (The absolute max your robot can handle)
MAX_LINEAR_SPEED = 0.5  # m/s
MAX_ANGULAR_SPEED = 1.0 # rad/s

# 2. ACCELERATION LIMITS (Smoothness)
MAX_LINEAR_ACCEL = 0.02 # per loop (20Hz) -> 0.4 m/s^2

# 3. GAINS
ANGULAR_PID = (0.8, 0.0, 0.05, 1.0) 
LINEAR_PID  = (0.4, 0.0, 0.02, 0.5) 
KV_LINEAR = 0.7   
KV_ANGULAR = 0.6  

# -----------------------------
# MAIN NODE
# -----------------------------
class SmoothRampBallFollower:
    def __init__(self):
        rospy.init_node("smooth_ramp_follower")
        
        self.sub_pose = rospy.Subscriber("/camera1/ball_pose", Point, self.pose_callback)
        self.sub_vel  = rospy.Subscriber("/camera1/ball_velocity", Point, self.vel_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        self.pid_angular = PID(*ANGULAR_PID)
        self.pid_linear = PID(*LINEAR_PID)
        
        self.last_seen_time = rospy.Time.now()
        self.ball_detected = False
        self.target_x = 0.0
        self.target_dist = 0.0
        
        self.smooth_vx = 0.0
        self.smooth_vz = 0.0
        self.alpha = 0.2
        
        self.current_linear_speed = 0.0
        self.last_known_direction = -1

    def pose_callback(self, msg):
        self.last_seen_time = rospy.Time.now()
        self.ball_detected = True
        self.target_x = msg.x
        self.target_dist = msg.z
        
        if msg.x != 0:
            self.last_known_direction = 1 if msg.x > 0 else -1

    def vel_callback(self, msg):
        self.smooth_vx = (self.alpha * msg.x) + ((1 - self.alpha) * self.smooth_vx)
        self.smooth_vz = (self.alpha * msg.z) + ((1 - self.alpha) * self.smooth_vz)

    def run(self):
        rate = rospy.Rate(20)
        twist = Twist()
        
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.last_seen_time).to_sec() > 1.0:
                self.ball_detected = False
                self.current_linear_speed = 0.0 

            if self.ball_detected:
                # --- 1. Calculate Raw Targets (PID + Feedforward) ---
                
                # Angular
                err_ang = -self.target_x
                if abs(err_ang) < ANG_DEADBAND: err_ang = 0
                ff_rot = (self.smooth_vx / max(self.target_dist, 0.1)) * KV_ANGULAR
                target_angular = self.pid_angular.compute(err_ang) + ff_rot

                # Linear
                err_lin = self.target_dist - DESIRED_DISTANCE
                if abs(err_lin) < LIN_DEADBAND: err_lin = 0
                ff_lin = self.smooth_vz * KV_LINEAR
                target_linear = self.pid_linear.compute(err_lin) + ff_lin

                # --- 2. Acceleration Ramp (Smooth Start) ---
                diff = target_linear - self.current_linear_speed
                if diff > MAX_LINEAR_ACCEL:
                    self.current_linear_speed += MAX_LINEAR_ACCEL
                elif diff < -MAX_LINEAR_ACCEL:
                    self.current_linear_speed -= MAX_LINEAR_ACCEL
                else:
                    self.current_linear_speed = target_linear

                # --- 3. SAFETY CLAMPS (Hard Limits) ---
                # This fixes the "Sum Trap" (PID + Feedforward > Max)
                
                # Clamp Linear
                if self.current_linear_speed > MAX_LINEAR_SPEED:
                    self.current_linear_speed = MAX_LINEAR_SPEED
                elif self.current_linear_speed < -MAX_LINEAR_SPEED:
                    self.current_linear_speed = -MAX_LINEAR_SPEED
                
                # Clamp Angular
                if target_angular > MAX_ANGULAR_SPEED:
                    target_angular = MAX_ANGULAR_SPEED
                elif target_angular < -MAX_ANGULAR_SPEED:
                    target_angular = -MAX_ANGULAR_SPEED

                # Assign final safe values
                twist.linear.x = self.current_linear_speed
                twist.angular.z = target_angular

            else:
                # Search Mode
                twist.linear.x = 0.0
                twist.angular.z = 0.3 * self.last_known_direction
                self.current_linear_speed = 0.0
                
            self.pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    node = SmoothRampBallFollower()
    node.run()