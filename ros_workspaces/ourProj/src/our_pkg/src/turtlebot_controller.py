import rospy
import tf2_ros
import math
import time

from geometry_msgs.msg import Twist
from occupancy_grid import OccupancyGrid

import numpy as np

def controller():
    try:
        #TODO: get correct topic to publish to
        pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        time.sleep(1)
        vel_msg = Twist()
    
    except (rospy.ROSInterruptException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("")

    r = rospy.Rate(20)

    occupancy_grid = OccupancyGrid()
    occupancy_grid.Initialize()

    grids_traversed = []
    rot_angle = 0.5 * math.pi

    while not rospy.is_shutdown():
        try:
            tb_tf_pos = tf_buffer.lookup_transform("world", "tb", rospy.Time(0))
            tb_pos = np.array([tb_tf_pos.transform.translation.x, tb_tf_pos.transform.translation.y])
            tb_grid = occupancy_grid.getGrid(tb_pos) # tb grid number [x y] from occupancy_grid
            tb_grid = tb_grid.tolist()

        except (rospy.ROSInterruptException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("")

        if len(grids_traversed) == 5:
            current_angle = 0

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 1
            t0 = rospy.Time.now().to_sec()
            while (current_angle < rot_angle):
                pub.publish(vel_msg)
                t1 = rospy.Time.now().to_sec()
                current_angle = t1 - t0
            
            vel_msg.angular.z = 0 #stop rotation
            pub.publish(vel_msg)
            del grids_traversed[:]
        
        elif tb_grid in grids_traversed:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            pub.publish(vel_msg)
        
        else:
            grids_traversed.append(tb_grid)
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            pub.publish(vel_msg)

if __name__ == "__main__":
    try:
        controller()
    except:
        pass
        


