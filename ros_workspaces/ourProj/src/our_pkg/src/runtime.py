from obstacle_creation import Obstacle, ObstacleManager
from track_tb import TrackTurtlebot
from occupancy_grid import OccupancyGrid
import rospy

if __name__ == '__main__':

    rospy.init_node('runtime', anonymous=True)

    # Create obstacle

    try:
        obstacle_manager = ObstacleManager()
        obstacle_manager.createObstacle()

    except rospy.ROSInterruptException:
        rospy.loginfo("")

    # Build occupancy grid

    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        try:
            occupancy_grid = OccupancyGrid(obstacle_manager.getObstacles())
            occupancy_grid.Initialize()

        except rospy.ROSInterruptException:
            rospy.loginfo("")

        # Track turtlebot
        try:
            turtlebot_tracker = TrackTurtlebot()
            turtlebot_tracker.Initialize()
            track_flag = turtlebot_tracker.Track(occupancy_grid.getOccupancy())

        except rospy.ROSInterruptException:
            rospy.loginfo("")

        if (track_flag = True):
            break
        
        r.sleep()

    