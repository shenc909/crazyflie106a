#!/usr/bin/env python
from obstacle_creation import Obstacle, ObstacleManager
from track_tb import TrackTurtlebot
from occupancy_grid import OccupancyGrid
from our_pkg.srv import obstacle_create_srv
import rospy


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


    except rospy.ROSInterruptException:
        rospy.loginfo("")

    # Build occupancy grid

    r = rospy.Rate(10)

    occupancy_grid = OccupancyGrid()
    occupancy_grid.Initialize()

    turtlebot_tracker = TrackTurtlebot()
    turtlebot_tracker.Initialize()

    while not rospy.is_shutdown():

        try:
            occupancy_grid.Update(obstacle_manager.getObstacles())

        except rospy.ROSInterruptException:
            rospy.loginfo("")

        # Track turtlebot
        try:
            
            track_flag = turtlebot_tracker.Track(occupancy_grid.getOccupancy())

        except rospy.ROSInterruptException:
            rospy.loginfo("")

        if (track_flag == True):
            break
        
        r.sleep()

if __name__ == '__main__':
    main()
