#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from std_msgs.msg import ColorRGBA, Header
class Obstacle:
	def __init__(self, _id, x_pos, y_pos, x_length, y_length, z_height):
		self._id = _id
		self._x_pos = x_pos
		self._y_pos = y_pos

		self._x_left_bound = x_pos - x_length/2
		self._x_right_bound = x_pos + x_length/2

		self._y_left_bound = y_pos - y_length/2
		self._y_right_bound = y_pos + y_length/2

		self._z_lower_bound = 0
		self._z_upper_bound = z_height

	def getXmin(self):
		return self._x_left_bound

	def getXmax(self):
		return self._x_right_bound

	def getYmin(self):
		return self._y_left_bound

	def getYmax(self):
		return self._y_right_bound

class ObstacleManager:
	def __init__(self):
		self._obstacles = {}

	def show_obstacle_in_rviz(self,marker_publisher, params):
		marker = Marker(
					type=Marker.CUBE,
					id=0,
					lifetime=0,
					pose=Pose(Point(params[1], params[2], 0), Quaternion(0, 0, 0, 1)),
					scale=Vector3(params[3], params[4], params[5]),
					color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
		h = Header()
		h.stamp = rospy.Time.now()
		marker.header =  h
		marker_publisher.publish(marker)

	def createObstacle(self, params):
		_id = params[0]
		if _id in self._obstacles:
			print("Obstacle ID already exists!")
			return

		_x_pos = params[1]
		_y_pos = params[2]

		x_length = params[3]
		y_length = params[4]
		z_height = params[5]

		print("Summary of obstacle:\n\tID: %s\n\tPosition(x,y): (%s, %s)\n\tDimensions(x,y,z): (%s, %s, %s)"%(_id,_x_pos,_y_pos,x_length,y_length,z_height))
		
		try: 
			print("Creating obstacle...")
			newObstacle = Obstacle(_id, _x_pos, _y_pos, x_length, y_length, z_height)
			self._obstacles[_id] = newObstacle
			print("Obstacle created")
		except:
			if _id in self._obstacles:
				del self._obstacles[_id]
			print("Failed to create obstacle")

		# Display in RViz
		marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
		rospy.sleep(0.5)                                                             
		self.show_obstacle_in_rviz(marker_publisher, params)


	def deleteObstacle(self, param):
		confirm = input("Confirm delete obstacle %s? (Y/N): " % (param[0]))
		if "y" in confirm.lower():
			try:
				del self._obstacles[param[0]]
				print("Obstacle Deleted")
			except:
				print("Failed to delete obstacle")
		else:
			print("Obstacle not deleted")

	def viewAllObstacles(self):
		for obstacle in self._obstacles.values():
			x_length = obstacle._x_right_bound - obstacle._x_left_bound
			y_length = obstacle._y_right_bound - obstacle._y_left_bound
			z_height = obstacle._z_upper_bound

			print("Obstacle ID: %s" % (obstacle._id))
			print("\tObstacle Position(x,y): (%s, %s)" % (obstacle._x_pos, obstacle._y_pos))
			print("\tX-Axis Length: %s" % (x_length))
			print("\tY-Axis Length: %s" % (y_length))
			print("\tZ-Axis Height: %s" % (z_height))
			print("")

	def getObstacles(self):
		return list(self._obstacles.values())