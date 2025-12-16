#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def move_ball_circle():
    rospy.init_node('ball_mover_node', anonymous=True)
    
    # Wait for the service to be available
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    rate = rospy.Rate(10) # 10 Hz
    t = 0.0
    
    while not rospy.is_shutdown():
        state_msg = ModelState()
        state_msg.model_name = 'moving_ball' # REPLACE with your ball's name in Gazebo
        
        # Circular Motion Math
        radius = 1.6
        state_msg.pose.position.x = radius * math.cos(t)
        state_msg.pose.position.y = radius * math.sin(t)
        state_msg.pose.position.z = 0.04 # Keep it slightly above ground
        
        # Reset orientation
        state_msg.pose.orientation.w = 1.0

        try:
            resp = set_state(state_msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        
        t += 0.01 # Increment time/angle
        rate.sleep()

if __name__ == '__main__':
    try:
        move_ball_circle()
    except rospy.ROSInterruptException:
        pass