#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge

# -----------------------------
# CONFIGURATION
# -----------------------------
D_real = 0.08        # Ball diameter in meters
f_pixels = 570   # Effective focal length

# Blue + Light Blue HSV range
lower_blue = np.array([95, 120, 70])
upper_blue = np.array([130, 255, 255])
# lower_blue = np.array([85, 50, 50])
# upper_blue = np.array([140, 255, 255])

# -----------------------------
# ROS PUBLISHERS
# -----------------------------
bridge = CvBridge()
pub_detection = rospy.Publisher("/camera1/detection", Image, queue_size=1)
pub_mask = rospy.Publisher("/camera1/mask", Image, queue_size=1)
pub_pose = rospy.Publisher("/camera1/ball_pose", Point, queue_size=1)
pub_vel = rospy.Publisher("/camera1/ball_velocity", Vector3, queue_size=1)

# -----------------------------
# VELOCITY MEMORY
# -----------------------------
prev_X = prev_Y = prev_Z = None
prev_time = time.time()

# -----------------------------
# IMAGE CALLBACK
# -----------------------------
def image_callback(msg):
    global prev_X, prev_Y, prev_Z, prev_time

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"CV Bridge conversion failed: {e}")
        return

    frame_height, frame_width = frame.shape[:2]
    c_x = frame_width / 2
    c_y = frame_height / 2

    # Blur + HSV + Mask
    blur = cv2.GaussianBlur(frame, (9, 9), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.medianBlur(mask, 7)

    # Publish mask
    mask_msg = bridge.cv2_to_imgmsg(mask, encoding="mono8")
    pub_mask.publish(mask_msg)

    # Contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_radius = 0
    center_coords = None
    for cnt in contours:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if radius > best_radius:
            best_radius = radius
            center_coords = (int(x), int(y))

    if best_radius > 0:
        u, v = center_coords
        D_pixel = 2 * best_radius

        # 3D position
        Z = (D_real * f_pixels) / D_pixel
        X = (u - c_x) * Z / f_pixels
        Y = (v - c_y) * Z / f_pixels

        
        # Time delta
        current_time = time.time()
        dt = current_time - prev_time if prev_X is not None else 0.0

        # Velocities
        if prev_X is not None and dt > 0:
            Vx = (X - prev_X) / dt
            Vy = (Y - prev_Y) / dt
            Vz = (Z - prev_Z) / dt
        else:
            Vx = Vy = Vz = 0.0

        prev_X, prev_Y, prev_Z = X, Y, Z
        prev_time = current_time

        # Optional smoothing
        alpha = 0.5
        Vx *= alpha
        Vy *= alpha
        Vz *= alpha

        # Draw detection
        cv2.circle(frame, (u, v), int(best_radius), (0, 255, 0), 3)
        cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)

        # Publish position
        point_msg = Point()
        point_msg.x = X
        point_msg.y = Y
        point_msg.z = Z
        pub_pose.publish(point_msg)

        # Publish velocity
        vel_msg = Vector3()
        vel_msg.x = Vx
        vel_msg.y = Vy
        vel_msg.z = Vz
        pub_vel.publish(vel_msg)

    # Publish detection image
    det_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    pub_detection.publish(det_msg)

# -----------------------------
# MAIN
# -----------------------------
def main():
    rospy.init_node("ball_detector_node")
    rospy.Subscriber("/camera1/image_raw", Image, image_callback, queue_size=1)
    rospy.loginfo("Ball Detector Node Started")
    rospy.spin()

if __name__ == "__main__":
    main()
