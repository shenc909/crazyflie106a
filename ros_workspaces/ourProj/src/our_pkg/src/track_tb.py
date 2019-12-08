#!/usr/bin/env python
import rospy
import tf2_ros
import tf

import numpy as np
from crazyflie_msgs.msg import PositionVelocityStateStamped, PositionVelocityState
import std_msgs.msg
import std_srvs
import subprocess
import time
# import tf_listener

class TrackTurtlebot(object): 
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.track_flag = False
        self.landed = False

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/track_turtlebot"

        # Load parameters.
        #if not self.LoadParameters():
         #   rospy.logerr("%s: Error loading parameters.", self._name)
         #   return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        return True

    def Track(self,occupancy_grid):

        track_flag = False

        try:
            cf_tf_pos = self._tf_buffer.lookup_transform("world", "cf", rospy.Time(0))
            tb_tf_pos = self._tf_buffer.lookup_transform("world", "tb", rospy.Time(0))
            print("HAPPY " + str(rospy.Time.now()))
            occupied_grid = occupancy_grid.getOccupancy() #occupied = 1, not occupied = 0
            cf_pos = [cf_tf_pos.transform.translation.x, cf_tf_pos.transform.translation.y] # cf [x y] position from motive
            tb_pos = [tb_tf_pos.transform.translation.x, tb_tf_pos.transform.translation.y] # tb [x y] position from motive
            cf_grid = occupancy_grid.getGrid(cf_pos) # cf grid number [x y] from occupancy_grid
            tb_grid = occupancy_grid.getGrid(tb_pos) # tb grid number [x y] from occupancy_grid
            shortest_dist = 1000
            shortest_idx = 9
            flying_height = 1.5
            landing_height = 1.1
            tb_height = 0.5
            offset = [0.09, -0.14]

            movement = [(-1, -1),(0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0)]
            
            next_step_name = ["front_left", "front","front_right","right","back_right","back","back_left","left"]

            grid_distance = np.ones(8) * shortest_dist
            if not ((cf_grid == tb_grid).all()):
                rospy.loginfo("cf not tb")
                for x in range(8):
                    next_step = cf_grid + movement[x] #[x y] next position
                    if (occupied_grid[next_step[0]][next_step[1]]) == 0: # if not occupied
                        grid_distance[x] = np.linalg.norm(tb_pos - occupancy_grid.gridToPoint(next_step))
                        # print(grid_distance[x])
                    else:
                        grid_distance[x] = 10000 # if occupied, dist = inf 
                for x in range(8):
                    if grid_distance[x]< shortest_dist:
                        shortest_idx = x
                        shortest_dist = grid_distance[x]
                print ("Going to adjacent " + next_step_name[shortest_idx] + " grid")

                nextpos = PositionVelocityStateStamped()
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                nextpos.header =  h
                # print(tb_grid)
                # print(tb_grid + movement[shortest_idx])
                # nextpos.state = [next_step[shortest_idx]+cf_pos, 0, 0, 0]
                nextpos.state.x = occupancy_grid.gridToPoint((tb_grid + movement[shortest_idx]))[0]
                nextpos.state.y = occupancy_grid.gridToPoint((tb_grid + movement[shortest_idx]))[1]
                nextpos.state.z = flying_height
                nextpos.state.x_dot = 0
                nextpos.state.y_dot = 0
                nextpos.state.z_dot = 0

                # print(nextpos)

                self._ref_pub.publish(nextpos)
                rospy.loginfo("publishing to /ref")

            if (cf_grid == tb_grid).all():
                rospy.loginfo("cf is tb")
                dist = np.linalg.norm([cf_pos[0]-tb_pos[0],cf_pos[1]-tb_pos[1]])
                #math.sqrt(math.pow(x_dist, 2) + math.pow(y_dist, 2))
                if (dist > 0.05):
                    nextpos = PositionVelocityStateStamped()
                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time.now()
                    nextpos.header =  h
                    # nextpos.state = [tb_pos+offset,flying_height, 0, 0, 0]
                    nextpos.state.x = tb_pos[0] + offset[0]
                    nextpos.state.y = tb_pos[1] + offset[1]
                    nextpos.state.z = flying_height
                    nextpos.state.x_dot = 0
                    nextpos.state.y_dot = 0
                    nextpos.state.z_dot = 0
                    self._ref_pub.publish(nextpos)
                    rospy.loginfo("publishing to /ref")

                else:
                    # Land
                    rospy.loginfo("landing")
                    nextpos = PositionVelocityStateStamped()
                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time.now()
                    nextpos.header =  h
                    # nextpos.state = [tb_pos+offset,landing_height, 0, 0, 0]
                    nextpos.state.x = tb_pos[0] + offset[0]
                    nextpos.state.y = tb_pos[1] + offset[1]
                    nextpos.state.z = landing_height
                    nextpos.state.x_dot = 0
                    nextpos.state.y_dot = 0
                    nextpos.state.z_dot = 0
                    self._ref_pub.publish(nextpos)
                    self.track_flag = True

                    if self.track_flag:
                        time.sleep(3)
                    
                        subprocess.call(["rosservice", "call","/land"])
                        self.track_flag = False
                        self.landed = True
                    # rospy.wait_for_service('landing')
                    # landing = rospy.ServiceProxy('/land', Land)
                    # landing()
            # print(tb_pos)

            # cf_tf_pos = self._tf_buffer.lookup_transform('cf', 'world', rospy.Time()) #full position
            # tb_tf_pos = self._tf_buffer.lookup_transform('tb', 'world', rospy.Time()) #full position

            # cf_pos = [cf_tf_pos[0], cf_tf_pos[1]] # cf [x y] position from motive
            # tb_pos = [tb_tf_pos[0], tb_tf_pos[1]] # tb [x y] position from motive

            # return (np.linalg.norm([cf_pos-tb_pos])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf found")
        
        return self.landed

    def RegisterCallbacks(self):
    # Subscriber.
    #    self._tf_sub = rospy.Subscriber('/tf', geometry_msgs.msg , queue_size=10)
        # Publisher.
        self._ref_pub = rospy.Publisher('/ref', PositionVelocityStateStamped, queue_size=10)

        return True