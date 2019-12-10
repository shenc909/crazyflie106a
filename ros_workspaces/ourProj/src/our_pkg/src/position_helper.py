#!/usr/bin/env python

import rospy
import tf2_ros
import tf
import numpy as np

class PositionHelper:
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def getCfGrid(self, occupancy_grid):
        try:
            cf_tf_pos = self._tf_buffer.lookup_transform("world", "cf", rospy.Time(0))
            # print("HAPPY " + str(rospy.Time.now()))
            cf_pos = np.array([cf_tf_pos.transform.translation.x, cf_tf_pos.transform.translation.y]) # cf [x y] position from motive
            cf_grid = occupancy_grid.getGrid(cf_pos) # cf grid number [x y] from occupancy_grid
            return cf_grid
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf found")