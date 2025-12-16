#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import time
from collections import deque
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge

D_real = 0.08  # Ball diameter in meters (8cm)
lower_blue = np.array([95, 120, 70])
upper_blue = np.array([130, 255, 255])

# Smoothing Factors
ALPHA = 0.6 
BUFFER_SIZE = 5 

# GLOBALS
bridge = CvBridge()
fx = fy = cx = cy = None
intrinsics_received = False
img_height, img_width = 0, 0

# State Memory
prev_time = None
image_callback_last_pos = None # Stores (X, Y, Z) from previous frame
filt_Vx = filt_Vy = filt_Vz = 0.0

# Position buffers
buf_X = deque(maxlen=BUFFER_SIZE)
buf_Y = deque(maxlen=BUFFER_SIZE)
buf_Z = deque(maxlen=BUFFER_SIZE)

def camera_info_callback(msg):
    global fx, fy, cx, cy, intrinsics_received, img_width, img_height
    fx = msg.K[0]
    fy = msg.K[4]
    cx = msg.K[2]
    cy = msg.K[5]
    img_width = msg.width
    img_height = msg.height
    intrinsics_received = True
    sub_info.unregister()
    rospy.loginfo(f"Intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")

def image_callback(msg):
    global prev_time, filt_Vx, filt_Vy, filt_Vz, image_callback_last_pos
    
    if not intrinsics_received:
        return

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        return

    # Image Processing
    blur = cv2.GaussianBlur(frame, (9, 9), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Clean noise
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_radius = 0
    center_coords = None
    is_touching_border = False

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 400: continue
        
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        
        # Check if contour touches the image border
        bx, by, bw, bh = cv2.boundingRect(cnt)
        touching_border = (bx <= 1 or by <= 1 or (bx+bw) >= img_width-1 or (by+bh) >= img_height-1)
        
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0: continue
        circularity = 4 * np.pi * (area / (perimeter * perimeter))

        # Strict check for small/far objects, relaxed check for large/close objects
        valid_shape = False
        if touching_border and radius > 50:
             valid_shape = True
             is_touching_border = True
        elif 0.6 < circularity < 1.3:
             valid_shape = True

        if valid_shape and radius > best_radius:
            best_radius = radius
            center_coords = (int(x), int(y))

    # Physics Calculation
    if best_radius > 0:
        u, v = center_coords
        D_pixel = 2 * best_radius

        # Z Calculation
        Z_raw = (D_real * fx) / D_pixel
        X_raw = (u - cx) * Z_raw / fx
        Y_raw = (v - cy) * Z_raw / fy

        # Smoothing Position
        buf_X.append(X_raw)
        buf_Y.append(Y_raw)
        buf_Z.append(Z_raw)

        X = sum(buf_X) / len(buf_X)
        Y = sum(buf_Y) / len(buf_Y)
        Z = sum(buf_Z) / len(buf_Z)

        # Velocity Calculation
        current_time = time.time()
        
        if prev_time is not None:
            dt = current_time - prev_time
            if dt > 0:
                if image_callback_last_pos is None:
                    Vx = Vy = Vz = 0
                else:
                    last_X, last_Y, last_Z = image_callback_last_pos
                    Vx = (X - last_X) / dt
                    Vy = (Y - last_Y) / dt
                    Vz = (Z - last_Z) / dt

                # Low Pass Filter
                filt_Vx = (ALPHA * Vx) + ((1 - ALPHA) * filt_Vx)
                filt_Vy = (ALPHA * Vy) + ((1 - ALPHA) * filt_Vy)
                filt_Vz = (ALPHA * Vz) + ((1 - ALPHA) * filt_Vz)

        # Update state
        prev_time = current_time
        image_callback_last_pos = (X, Y, Z)

        # Visualization
        cv2.circle(frame, (u, v), int(best_radius), (0, 255, 0), 2)
        cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)

        color = (0, 255, 255)
        if is_touching_border:
            cv2.putText(frame, "EDGE DETECTED", (u, v-40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            color = (0, 165, 255)

        cv2.putText(frame, f"X: {X:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame, f"Z: {Z:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame, f"Vx: {filt_Vx:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(frame, f"Vz: {filt_Vz:.2f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        # Publish
        pub_pose.publish(Point(x=X, y=Y, z=Z))
        pub_vel.publish(Vector3(x=filt_Vx, y=filt_Vy, z=filt_Vz))

    pub_detection.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    pub_mask.publish(bridge.cv2_to_imgmsg(mask, "mono8"))

if __name__ == "__main__":
    rospy.init_node("ball_detector_node")
    
    pub_detection = rospy.Publisher("/cv_camera/detection", Image, queue_size=1)
    pub_mask = rospy.Publisher("/cv_camera/mask", Image, queue_size=1)
    pub_pose = rospy.Publisher("/cv_camera/ball_pose", Point, queue_size=1)
    pub_vel = rospy.Publisher("/cv_camera/ball_velocity", Vector3, queue_size=1)

    global sub_info

    sub_info = rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_callback)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback, queue_size=1)

    rospy.loginfo("Node Started...")
    rospy.spin()