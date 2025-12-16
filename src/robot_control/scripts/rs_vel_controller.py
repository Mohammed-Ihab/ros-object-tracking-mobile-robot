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
        return max(min(output, self.max_val), -self.max_val)

    def reset_state(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = rospy.Time.now()

# -----------------------------
# CONFIGURATION
# -----------------------------
DESIRED_DISTANCE = 0.5
LIN_DEADBAND = 0.12  # Meters (Tolerance)
ANG_DEADBAND = 0.05  # Radians (Tolerance)

# 1. HARDWARE LIMITS
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.5  # Increased for faster turning

# 2. ACCELERATION LIMITS
# Increased from 0.02 to 0.1 for faster reaction
MAX_LINEAR_ACCEL = 0.1  

# 3. GAINS
# Stiffer Angular PID (1.5) to keep ball centered
ANGULAR_PID = (1.5, 0.0, 0.1, 1.5) 
# Stronger Linear PID
LINEAR_PID  = (0.6, 0.0, 0.05, 0.5) 

# Feedforward disabled (0.0) to remove jitter from noisy velocity data
KV_LINEAR = 0.0   
KV_ANGULAR = 0.0  

# -----------------------------
# MAIN NODE
# -----------------------------
class SmoothRampBallFollower:
    def __init__(self):
        rospy.init_node("smooth_ramp_follower")
        
        self.sub_pose = rospy.Subscriber("/cv_camera/ball_pose", Point, self.pose_callback)
        self.sub_vel  = rospy.Subscriber("/cv_camera/ball_velocity", Point, self.vel_callback)
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
            self.last_known_direction = -1 if msg.x > 0 else 1

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
                
                # --- 1. Angular Control (X) ---
                err_ang = -self.target_x
                
                # TOLERANCE CHECK (Anti-Wiggle)
                if abs(err_ang) < ANG_DEADBAND:
                    target_angular = 0.0
                    self.pid_angular.reset_state() 
                else:
                    ff_rot = (self.smooth_vx / max(self.target_dist, 0.1)) * KV_ANGULAR
                    target_angular = self.pid_angular.compute(err_ang) + ff_rot

                # --- 2. Linear Control (Z) ---
                err_lin = self.target_dist - DESIRED_DISTANCE

                # TOLERANCE CHECK (Anti-Wiggle)
                if abs(err_lin) < LIN_DEADBAND:
                    target_linear = 0.0
                    self.pid_linear.reset_state() 
                else:
                    ff_lin = self.smooth_vz * KV_LINEAR
                    target_linear = self.pid_linear.compute(err_lin) + ff_lin

                # --- 3. Asymmetric Ramp (Fast Reaction) ---
                diff = target_linear - self.current_linear_speed
                
                if diff > MAX_LINEAR_ACCEL:
                    # Accelerating: Smooth Ramp
                    self.current_linear_speed += MAX_LINEAR_ACCEL
                elif diff < -MAX_LINEAR_ACCEL:
                    # Decelerating: BRAKE FAST (3x limit)
                    # This prevents the robot from crashing into the ball or overshooting
                    self.current_linear_speed -= (MAX_LINEAR_ACCEL * 3.0)
                    
                    # Don't overshoot the brake
                    if self.current_linear_speed < target_linear:
                        self.current_linear_speed = target_linear
                else:
                    self.current_linear_speed = target_linear

                # --- 4. Safety Clamps ---
                if self.current_linear_speed > MAX_LINEAR_SPEED:
                    self.current_linear_speed = MAX_LINEAR_SPEED
                elif self.current_linear_speed < -MAX_LINEAR_SPEED:
                    self.current_linear_speed = -MAX_LINEAR_SPEED
                
                if target_angular > MAX_ANGULAR_SPEED:
                    target_angular = MAX_ANGULAR_SPEED
                elif target_angular < -MAX_ANGULAR_SPEED:
                    target_angular = -MAX_ANGULAR_SPEED

                twist.linear.x = self.current_linear_speed
                twist.angular.z = target_angular

            else:
                # Search Mode
                twist.linear.x = 0.0
                twist.angular.z = 0.3 * self.last_known_direction
                self.current_linear_speed = 0.0
                self.pid_linear.reset_state()
                self.pid_angular.reset_state()
                
            self.pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    node = SmoothRampBallFollower()
    node.run()