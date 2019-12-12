#!/usr/bin/env python

import rospy
import tf2_ros
import tf

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import numpy as np

class OccupancyGrid(object):
    def __init__(self):
        self._intialized = False
        self._coordcf = [0,0]
        self._coordtb = [0,0]
        self._next_grid = [0,0]
    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up the map.
        self._map = np.zeros((self._x_num, self._y_num))
        self._visualMap = np.copy(self._map)

        self._initialized = True
        return True

    def LoadParameters(self):

        # Dimensions and bounds.
        # TODO! You'll need to set values for class variables called:
        self._x_num = rospy.get_param("~x/num")
        self._x_min = rospy.get_param("~x/min")
        self._x_max = rospy.get_param("~x/max")
        self._x_res = (self._x_max - self._x_min)/self._x_num# The resolution in x. Note: This isn't a ROS parameter. What will you do instead?
        self._y_num = rospy.get_param("~y/num")
        self._y_min = rospy.get_param("~y/min")
        self._y_max = rospy.get_param("~y/max")
        self._y_res = (self._y_max - self._y_min)/self._y_num # The resolution in y. Note: This isn't a ROS parameter. What will you do instead?
        self._fixed_frame = rospy.get_param("~frames/fixed")
        self._vis_topic = rospy.get_param("~topics/vis")

        return True

    def RegisterCallbacks(self):
        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        Marker,
                                        queue_size=10)

        return True
    
    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def PointToVoxel(self, x, y):
        grid_x = int((x - self._x_min) / self._x_res)
        grid_y = int((y - self._y_min) / self._y_res)

        return (grid_x, grid_y)

    # Get the center point (x, y) corresponding to the given voxel.
    def VoxelCenter(self, ii, jj):
        center_x = self._x_min + (0.5 + ii) * self._x_res
        center_y = self._y_min + (0.5 + jj) * self._y_res

        return (center_x, center_y)


    # Colormap to take log odds at a voxel to a RGBA color.
    def Colormap(self, ii, jj):
        p = self._visualMap[ii, jj]
        c = ColorRGBA()
        if p == 2:
            c.r = 0.1
            c.g = 1.0
            c.b = 0.1
            c.a = 0.75
        elif p == 3:
            c.r = 1.0
            c.g = 1.0
            c.b = 0.1
            c.a = 0.75
        elif p == 4:
            c.r = 1.0
            c.g = 1.0
            c.b = 1.0
            c.a = 0.75
        elif p == 5:
            c.r = 1.0
            c.g = 0.1
            c.b = 1.0
            c.a = 0.75
        else:
            c.r = p
            c.g = 0.1
            c.b = 1.0 - p
            c.a = 0.3
        return c

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.
    def Visualize(self):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._fixed_frame
        m.ns = "map"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = self._x_res
        m.scale.y = self._y_res
        m.scale.z = 0.01

        for ii in range(self._x_num):
            for jj in range(self._y_num):
                p = Point(0.0, 0.0, 0.0)
                (p.x, p.y) = self.VoxelCenter(ii, jj)

                m.points.append(p)
                m.colors.append(self.Colormap(ii, jj))

        self._vis_pub.publish(m)
    
    def Update(self, objectmap):
        self._map = np.zeros((self._x_num, self._y_num))
        self._objectmap = objectmap

        for i in range(len(self._objectmap)):
            obj = self._objectmap[i]
            gridXmin = int((obj.getXmin()-self._x_min)/self._x_res)
            gridXmax = int((obj.getXmax()-self._x_min)/self._x_res)
            gridYmin = int((obj.getYmin()-self._y_min)/self._y_res)
            gridYmax = int((obj.getYmax()-self._y_min)/self._y_res)

            for x in range(gridXmin-1, gridXmax+2):
                for y in range(gridYmin-1, gridYmax+2):
                    try:
                        self._map[x][y] = 1
                    except(IndexError):
                        rospy.logwarn("obstacle out of map bounds")
                        continue
        
        self._visualMap = np.copy(self._map)
        self._visualMap[int(self._next_grid[0])][int(self._next_grid[1])] = 5
        self._visualMap[self.getGrid(self._coordtb)[0]][self.getGrid(self._coordtb)[1]] = 2
        self._visualMap[self.getGrid(self._coordcf)[0]][self.getGrid(self._coordcf)[1]] = 3
        if (self.getGrid(self._coordtb) == self.getGrid(self._coordcf)).all():
            self._visualMap[self.getGrid(self._coordcf)[0]][self.getGrid(self._coordcf)[1]] = 4

        
    def getOccupancy(self):
        return self._map

    def getGrid(self,pos):
        xgrid = int((pos[0] - self._x_min)/self._x_res)
        ygrid = int((pos[1] - self._y_min)/self._y_res)
        return np.array([xgrid, ygrid])
    
    def gridToPoint(self, grid):
        xpos = grid[0] * self._x_res + self._x_res/2 + self._x_min
        ypos = grid[1] * self._y_res + self._y_res/2 + self._y_min
        return np.array([xpos, ypos])

    def enterPos(self, coordtb, coordcf):
        self._coordtb = coordtb
        self._coordcf = coordcf
    
    def nextGrid(self, next_grid):
        self._next_grid = next_grid