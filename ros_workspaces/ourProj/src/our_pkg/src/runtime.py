#!/usr/bin/env python
from obstacle_creation import Obstacle, ObstacleManager
from track_tb import TrackTurtlebot
from occupancy_grid import OccupancyGrid
from position_helper import PositionHelper
from our_pkg.srv import obstacle_create_srv
import rospy
import subprocess
import time
import numpy as np
import tf2_ros
import tf
import roslaunch

obstacle_manager = ObstacleManager()

def handle_create_request(req):
    params = [req.id_, req.x_pos, req.y_pos, req.x_length, req.y_length, req.z_height]
    obstacle_manager.createObstacle(params)
    return "done"

def main():
    # Instantiate obstacle manager

    try:
        s = rospy.Service('create_obstacle', obstacle_create_srv, handle_create_request)
        rospy.init_node('runtime', anonymous=True)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        time.sleep(1)
        cf_tf_pos = tf_buffer.lookup_transform("world", "cf", rospy.Time(0))
        cf_pos = np.array([cf_tf_pos.transform.translation.x, cf_tf_pos.transform.translation.y]) # cf [x y] position from motive
        # subprocess.call(["roslaunch", "crazyflie_examples","hw_hover.launch", "takeoff_hover_x:=" + str(cf_pos[0]), "takeoff_hover_y:=" + str(cf_pos[1]), "takeoff_hover_z:=1.5"])



    except (rospy.ROSInterruptException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("")
        # rospy.logerr("No takeoff position info, using defaults...")
        # subprocess.call(["roslaunch", "crazyflie_examples","hw_hover.launch"])

    # Build occupancy grid

    r = rospy.Rate(10)

    occupancy_grid = OccupancyGrid()
    occupancy_grid.Initialize()

    turtlebot_tracker = TrackTurtlebot()
    turtlebot_tracker.Initialize()
    track_flag = False

    # position_helper = PositionHelper()
    prev_cf_pos = [0,0]

    mydata = raw_input("Press any key to start: ")

    rospy.wait_for_service('/takeoff')
    subprocess.call(["rosservice", "call","/takeoff"])
    returnGrid = np.array([0,0])
    time.sleep(3)
    
    while not rospy.is_shutdown():

        try:
            occupancy_grid.Update(obstacle_manager.getObstacles())
            

        except rospy.ROSInterruptException:
            rospy.loginfo("")

        # Get current crazyflie grid pos to increase cost of returning to same grid
        try:
            mod_occupied_grid = occupancy_grid.getOccupancy()
            # mod_occupied_grid[returnGrid[0]][returnGrid[1]] = 1
            occupancy_grid.Visualize()

        except rospy.ROSInterruptException:
            rospy.loginfo("")

        # Track turtlebot
        try:
            
            track_flag, returnGrid = turtlebot_tracker.Track(occupancy_grid, obstacle_manager, mod_occupied_grid)

        except rospy.ROSInterruptException:
            rospy.loginfo("")

        if (track_flag == True):
            break
        
        r.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
