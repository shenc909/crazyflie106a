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
from dijkstra2 import dijkstra
# import tf_listener

class TrackTurtlebot(object): 
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._track_flag = False
        self._landed = False
        self._speedScale = 0.8
        self._landingGain = 0.3
        self._LastGrid = np.array([0,0])
        self._counter = 0
        self._history = list()
        self._history.append(np.array([0,0]))
        self._history.append(np.array([0,0]))
        self._history.append(np.array([0,0]))
        self._history.append(np.array([0,0]))
        self._history.append(np.array([0,0]))
        self._initFlag = False
        self._poseHistory = list()
        self._poseCounter = 0
        self._poseHistoryCount = 3
        self._loopTime = 0.1

        for a in range(self._poseHistoryCount):
            self._poseHistory.append(np.array([0,0]))

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

    def Track(self, occupancy_grid, obstacle_manager, mod_occupied_grid):

        self._track_flag = False

        try:
            cf_tf_pos = self._tf_buffer.lookup_transform("world", "cf", rospy.Time(0))
            tb_tf_pos = self._tf_buffer.lookup_transform("world", "tb1", rospy.Time(0))
            # print("HAPPY " + str(rospy.Time.now()))
            # occupied_grid = occupancy_grid.getOccupancy() #occupied = 1, not occupied = 0
            occupied_grid = mod_occupied_grid
            cf_pos = np.array([cf_tf_pos.transform.translation.x, cf_tf_pos.transform.translation.y]) # cf [x y] position from motive
            tb_pos = np.array([tb_tf_pos.transform.translation.x, tb_tf_pos.transform.translation.y]) # tb [x y] position from motive
            tb_vel = self.findVel(tb_pos)
            occupancy_grid.enterPos(tb_pos, cf_pos)
            cf_grid = occupancy_grid.getGrid(cf_pos) # cf grid number [x y] from occupancy_grid
            tb_grid = occupancy_grid.getGrid(tb_pos) # tb grid number [x y] from occupancy_grid

            if not self._initFlag:
                self._returnGrid = self.ticked(cf_grid)
                self._next_grid = cf_grid
                nextdjgrid = cf_grid
                self._initFlag = True

            #tuning constants
            shortest_dist = 1000
            shortest_idx = 9
            flying_height = 1.3
            landing_height = 1.1
            tb_height = 0.5
            # offset = [0.09, -0.14]
            dist = np.linalg.norm(cf_pos-tb_pos)
            # offset = [-0.20,-0.05]
            offset = [0,0]
            ignoreGridDist = 0.2

            movement = [(1, 1),(1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
            
            next_step_name = ["front_left", "front","front_right","right","back_right","back","back_left","left"]

            grid_distance = np.ones(8) * shortest_dist
            occupied_flag = False

            distanceDiff = np.linalg.norm(tb_pos - cf_pos)
            if distanceDiff > ignoreGridDist:
                    rospy.loginfo("cf not tb")
                    self._returnGrid = self.ticked(cf_grid)
                    # for x in range(8):
                    #     next_step = cf_grid + movement[x] #[x y] next position
                    #     if not ((occupied_grid[next_step[0]][next_step[1]]) == 1): # if not occupied
                    #         grid_distance[x] = np.linalg.norm(tb_grid - next_step)
                    #         # print(grid_distance[x])
                    #     else:
                    #         grid_distance[x] = 10000 # if occupied, dist = inf 
                    #         occupied_flag = True
                    
                    # if occupied_flag:
                    #     grid_distance[0] = 10000
                    #     grid_distance[2] = 10000
                    #     grid_distance[4] = 10000
                    #     grid_distance[6] = 10000
                            

                    # for x in range(8):
                    #     if grid_distance[x]< shortest_dist:
                    #         shortest_idx = x
                    #         shortest_dist = grid_distance[x]
                    # print ("Going to adjacent " + next_step_name[shortest_idx] + " grid")

                    path = dijkstra((mod_occupied_grid).astype(int), cf_grid, tb_grid)
                    # print(path)
                    if not isinstance(path, bool):
                        try:
                            nextdjgrid = path[2]
                        except:
                            nextdjgrid = tb_grid
                        # nextdjgrid = np.array([nextdjgrid[1], nextdjgrid[0]])
                        # print(nextdjgrid)
                        # print(cf_grid)
                    else:
                        nextdjgrid = tb_grid

                    nextpos = PositionVelocityStateStamped()
                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time.now()
                    nextpos.header =  h
                    # print(tb_grid)
                    # print(tb_grid + movement[shortest_idx])
                    # nextpos.state = [next_step[shortest_idx]+cf_pos, 0, 0, 0]
                    # self._next_grid = cf_grid + movement[shortest_idx]
                    # nextpos.state.x = occupancy_grid.gridToPoint((cf_grid + movement[shortest_idx]))[0]
                    # nextpos.state.y = occupancy_grid.gridToPoint((cf_grid + movement[shortest_idx]))[1]
                    nextpos.state.x = occupancy_grid.gridToPoint(nextdjgrid)[0]
                    nextpos.state.y = occupancy_grid.gridToPoint(nextdjgrid)[1]
                    nextpos.state.z = flying_height
                    nextpos.state.x_dot = (occupancy_grid.gridToPoint(nextdjgrid)[0] - cf_pos[0])*self._speedScale#(movement[shortest_idx])[0] * self._speedScale * 1
                    nextpos.state.y_dot = (occupancy_grid.gridToPoint(nextdjgrid)[1] - cf_pos[1])*self._speedScale#(movement[shortest_idx])[1] * self._speedScale * 1
                    nextpos.state.z_dot = 0
                    print(occupancy_grid.gridToPoint(nextdjgrid)[0] - cf_pos[0])
                    print(occupancy_grid.gridToPoint(nextdjgrid)[1] - cf_pos[1])
                    self._ref_pub.publish(nextpos)
                    occupancy_grid.nextGrid(nextdjgrid)
                    
                    
                    # print(nextpos)
                    
                    rospy.loginfo("publishing to /ref")
                    

            # if (cf_grid == tb_grid).all():
            else:
                rospy.loginfo("cf is tb")
                dist = np.linalg.norm(cf_pos-tb_pos)
                #math.sqrt(math.pow(x_dist, 2) + math.pow(y_dist, 2))
                if dist > 0.05:
                    nextpos = PositionVelocityStateStamped()
                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time.now()
                    nextpos.header =  h
                    nextpos.state.x = tb_pos[0] + offset[0] #+ tb_vel[0] * self._loopTime
                    nextpos.state.y = tb_pos[1] + offset[1] #+ tb_vel[1] * self._loopTime
                    nextpos.state.z = landing_height
                    nextpos.state.x_dot = 0#self._landingGain * tb_vel[0]#(tb_pos[0] - cf_pos[0])
                    nextpos.state.y_dot = 0#self._landingGain * tb_vel[1]#(tb_pos[1] - cf_pos[1])
                    nextpos.state.z_dot = 0
                    self._ref_pub.publish(nextpos)
                    rospy.loginfo("publishing to /ref")
                    occupancy_grid.nextGrid(cf_grid)

                else:
                    # Land
                    occupancy_grid.Update(obstacle_manager.getObstacles())
                    occupancy_grid.Visualize()
                    rospy.loginfo("landing")
                    nextpos = PositionVelocityStateStamped()
                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time.now()
                    nextpos.header =  h
                    # nextpos.state = [tb_pos+offset,landing_height, 0, 0, 0]
                    nextpos.state.x = tb_pos[0] + offset[0] #+ tb_vel[0] * self._loopTime
                    nextpos.state.y = tb_pos[1] + offset[1] #+ tb_vel[1] * self._loopTime
                    nextpos.state.z = landing_height
                    nextpos.state.x_dot = 0#tb_vel[0]# + self._landingGain * (tb_pos[0] - cf_pos[0])
                    nextpos.state.y_dot = 0#tb_vel[1]# + self._landingGain * (tb_pos[1] - cf_pos[1])
                    nextpos.state.z_dot = 0
                    self._ref_pub.publish(nextpos)
                    self._track_flag = True

                    if self._track_flag:
                        time.sleep(1.5)
                        rospy.loginfo("landing start")
                        cf_tf_pos = self._tf_buffer.lookup_transform("world", "cf", rospy.Time(0))
                        tb_tf_pos = self._tf_buffer.lookup_transform("world", "tb", rospy.Time(0))
                        dist = np.linalg.norm(cf_pos-tb_pos)
                        if dist > 0.05:
                            rospy.logerr("landing aborted")
                        else:
                            subprocess.call(["rosservice", "call","/land"])
                            self._track_flag = False
                            self._landed = True
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
        
        # except(IndexError):
        #     rospy.logwarn("IndexError")
        
        return (self._landed, self._returnGrid)

    def RegisterCallbacks(self):
    # Subscriber.
    #    self._tf_sub = rospy.Subscriber('/tf', geometry_msgs.msg , queue_size=10)
        # Publisher.
        self._ref_pub = rospy.Publisher('/ref', PositionVelocityStateStamped, queue_size=10)

        return True

    def ticked(self, inputGrid):
        returnItem = self._history[(self._counter+1)%5]
        if not ((returnItem == inputGrid).all()):
            self._counter = self._counter + 1
            self._counter = self._counter%5
            self._history[self._counter] = inputGrid
        return returnItem

    def findVel(self, inputPose):
        self._poseCounter += 1
        self._poseCounter = self._poseCounter % self._poseHistoryCount
        self._poseHistory[self._poseCounter] = inputPose
        self._calcVel = (self._poseHistory[(self._poseCounter - 1) % self._poseHistoryCount] - inputPose)/(self._loopTime * self._poseHistoryCount)
        return self._calcVel