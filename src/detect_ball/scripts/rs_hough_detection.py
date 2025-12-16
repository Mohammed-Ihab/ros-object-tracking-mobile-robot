#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import time
from collections import deque
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge

D_real = 0.08 
lower_blue = np.array([95, 120, 70])
upper_blue = np.array([130, 255, 255])

MAX_DISTANCE = 2.5  

ALPHA = 0.7
BUFFER_SIZE = 5

HOUGH_PARAM1 = 100
HOUGH_PARAM2 = 30 
MIN_RADIUS = 15
MAX_RADIUS = 200

# GLOBALS
bridge = CvBridge()
fx = fy = cx = cy = None
intrinsics_received = False
img_height, img_width = 0, 0

# State
prev_time = None
image_callback_last_pos = None 
filt_Vx = filt_Vy = filt_Vz = 0.0
frames_tracked = 0 

buf_X = deque(maxlen=BUFFER_SIZE)
buf_Y = deque(maxlen=BUFFER_SIZE)
buf_Z = deque(maxlen=BUFFER_SIZE)

def verify_circle_dynamic(mask, x, y, r, width, height, is_already_tracking):
    if r < MIN_RADIUS: return False, "Too Small"

    dist_to_edge = min(x, width - x, y, height - y)
    
    if dist_to_edge > r:
        required_circularity = 0.6
        required_fill = 0.6
    else:
        factor = max(0, dist_to_edge / r)
        required_circularity = 0.2 + (0.4 * factor)
        required_fill = 0.4 + (0.2 * factor)

    if is_already_tracking:
        required_circularity *= 0.9
        required_fill *= 0.9

    circle_mask = np.zeros_like(mask)
    cv2.circle(circle_mask, (int(x), int(y)), int(r), 255, -1)
    intersection = cv2.bitwise_and(mask, circle_mask)
    fill_ratio = cv2.countNonZero(intersection) / (np.pi * r * r)

    if fill_ratio < required_fill:
        return False, f"Low Fill {fill_ratio:.2f}"

    if dist_to_edge > 0:
        contours, _ = cv2.findContours(intersection, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            perimeter = cv2.arcLength(cnt, True)
            area = cv2.contourArea(cnt)
            if perimeter > 0:
                circ = 4 * np.pi * (area / (perimeter**2))
                if circ < required_circularity:
                    return False, f"Not Round {circ:.2f}"

    return True, "Valid"

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
    rospy.loginfo(f"Intrinsics: fx={fx:.1f}, fy={fy:.1f}")

def image_callback(msg):
    global prev_time, filt_Vx, filt_Vy, filt_Vz, image_callback_last_pos, frames_tracked
    
    if not intrinsics_received:
        return

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        return

    # 1. Preprocessing
    blur = cv2.GaussianBlur(frame, (9, 9), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 2. Hough Detection
    mask_blurred = cv2.GaussianBlur(mask, (9, 9), 2)
    circles = cv2.HoughCircles(
        mask_blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=100,
        param1=HOUGH_PARAM1, param2=HOUGH_PARAM2, 
        minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS
    )

    best_circle = None
    max_r = 0
    status_text = "Searching..."
    is_tracking = frames_tracked > 5

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            x, y, r = i[0], i[1], i[2]
            
            is_valid, reason = verify_circle_dynamic(mask, x, y, r, img_width, img_height, is_tracking)
            
            if is_valid:
                if r > max_r:
                    max_r = r
                    best_circle = (x, y, r)
                    status_text = reason
            else:
                cv2.circle(frame, (x, y), r, (0, 0, 255), 1)

    # Physics & Publishing
    valid_detection = False
    
    if best_circle is not None:
        u, v, r = best_circle
        D_pixel = 2 * r

        # Calculate Distance
        Z_raw = (D_real * fx) / D_pixel

        # --- DISTANCE CHECK ---
        if Z_raw > MAX_DISTANCE:
            status_text = f"Too Far ({Z_raw:.1f}m)"
            # We treat this as NO BALL found
            best_circle = None 
        else:
            valid_detection = True

    if valid_detection:
        frames_tracked += 1
        
        X_raw = (u - cx) * Z_raw / fx
        Y_raw = (v - cy) * Z_raw / fy

        buf_X.append(X_raw)
        buf_Y.append(Y_raw)
        buf_Z.append(Z_raw)

        X = sum(buf_X) / len(buf_X)
        Y = sum(buf_Y) / len(buf_Y)
        Z = sum(buf_Z) / len(buf_Z)

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

                filt_Vx = (ALPHA * Vx) + ((1 - ALPHA) * filt_Vx)
                filt_Vy = (ALPHA * Vy) + ((1 - ALPHA) * filt_Vy)
                filt_Vz = (ALPHA * Vz) + ((1 - ALPHA) * filt_Vz)

        prev_time = current_time
        image_callback_last_pos = (X, Y, Z)

        # Visuals
        color = (0, 255, 0)
        dist_to_edge = min(u, img_width-u, v, img_height-v)
        if dist_to_edge < r: 
            color = (0, 255, 255)
            status_text = "Edge Mode"

        cv2.circle(frame, (u, v), r, color, 2)
        cv2.circle(frame, (u, v), 2, color, -1)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"X: {X:.2f}m", (10, 30), font, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"Z: {Z:.2f}m", (10, 60), font, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"Vx: {filt_Vx:.2f}", (10, 90), font, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Vz: {filt_Vz:.2f}", (10, 120), font, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Status: {status_text}", (10, 150), font, 0.6, color, 2)

        pub_pose.publish(Point(x=X, y=Y, z=Z))
        pub_vel.publish(Vector3(x=filt_Vx, y=filt_Vy, z=filt_Vz))
    else:
        frames_tracked = 0
        cv2.putText(frame, f"NO BALL {status_text}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    pub_detection.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    pub_mask.publish(bridge.cv2_to_imgmsg(mask, "mono8"))

if __name__ == "__main__":
    rospy.init_node("hough_verified_node")
    
    pub_detection = rospy.Publisher("/cv_camera/detection", Image, queue_size=1)
    pub_mask = rospy.Publisher("/cv_camera/mask", Image, queue_size=1)
    pub_pose = rospy.Publisher("/cv_camera/ball_pose", Point, queue_size=1)
    pub_vel = rospy.Publisher("/cv_camera/ball_velocity", Vector3, queue_size=1)

    global sub_info
    sub_info = rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_callback)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback, queue_size=1)

    rospy.loginfo("Node Started...")
    rospy.spin()