#!/usr/bin/env python
import rospy
import tf2_ros
import tf

import numpy as np
from crazyflie_msgs.msg import PositionVelocityStateStamped
import tf_listener

class TrackTurtlebot(object): 
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/track_turtlebot"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        return True

    def Track(self,occupancy_grid):

        track_flag = False

        while not rospy.is_shutdown():
            cf_tf_pos = self._tf_buffer.lookup_transform('cf', 'World', rospy.Time())
            tb_tf_pos = self._tf_buffer.lookup_transform('tb', 'World', rospy.Time())

            occupied_grid = occupancy_grid.getOccupancy() #occupied = 1, not occupied = 0
            cf_pos = [cf_tf_pos[0], cf_tf_pos[1]] # cf [x y] position from motive
            tb_pos = [tb_tf_pos[0], tb_tf_pos[1]] # tb [x y] position from motive
            cf_grid = occupancy_grid.getGrid(cf_pos) # cf grid number [x y] from occupancy_grid
            tb_grid = occupancy_grid.getGrid(tb_pos) # tb grid number [x y] from occupancy_grid
            shortest_dist = 0
            shortest_idx = 9
            flying_height = 1.5
            landing_height = 1.2
            tb_height = 0.5
            offset = [0.1, -0.15]

            movement = [[-1 1],[0 1], [1 1], [1 0], [1 -1], [0 -1], [-1 -1], [-1 0]]
            next_step = tb_grid + movement #[x y] next position
            next_step_name = ["front_left", "front","front_right","right","back_right","back","back_left","left"]

            if (cf_grid) != (tb_grid):
                for x in range(8):
                    if !(occupied_grid[next_step[x]]): # if not occupied
                        grid_distance[x] = np.linalg.norm([tb_grid - next_step[x]])
                    else:
                        grid_distance[x] = 0 # if occupied, dist = inf 
                for x in range(8):
                    if (grid_distance[x]<shortest_dist && grid_distance[x]!=0):
                        shortest_idx = x
                        shortest_dist = grid_distance
                print ("Going to adjacent " next_step_name[shortest_idx] " grid")
                self._ref_pub.publish([next_step[shortest_idx]+cf_pos flying_height 0 0 0])

            if (cf_grid) == (tb_grid):
                x_dist = cf_pos[0] - tf_pos[0]
                y_dist = cf_pos[1] - tf_pos[1]
                dist = math.sqrt(math.pow(x_dist, 2) + math.pow(y_dist, 2))
                if (dist > 0.01):
                    self._ref_pub.publish([tb_pos+offset flying_height 0 0 0])
                else:
                    # Land
                    self._ref_pub.publish([tb_pos+offset landing_height 0 0 0])
                    rospy.wait_for_service('landing')
                    landing = rospy.ServiceProxy('/land', Empty)
                    landing()
                    track_flag = True

        cf_tf_pos = self._tf_buffer.lookup_transform('cf', 'World', rospy.Time()) #full position
        tb_tf_pos = self._tf_buffer.lookup_transform('tb', 'World', rospy.Time()) #full position

        cf_pos = [cf_tf_pos[0] cf_tf_pos[1]] # cf [x y] position from motive
        tb_pos = [tb_tf_pos[0] tb_tf_pos[1]] # tb [x y] position from motive

        # return (np.linalg.norm([cf_pos-tb_pos])

        return track_flag


    def RegisterCallbacks(self):
    # Subscriber.
    #    self._tf_sub = rospy.Subscriber('/tf', geometry_msgs.msg , queue_size=10)
        # Publisher.
        self._ref_pub = rospy.Publisher('/ref', PositionVelocityStateStamped, queue_size=10)

        return True