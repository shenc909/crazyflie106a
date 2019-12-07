#!/usr/bin/env python

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

class ObstacleManager:
	def __init__(self):
		self._obstacles = {}

	def createObstacle(self):
		_id = raw_input("Enter unique Obstacle ID: ")
		while _id in self._obstacles:
			_id = raw_input("Obstacle ID already exists. Enter another ID: ")

		_x_pos = raw_input("Enter x_pos: ")
		_y_pos = raw_input("Enter y_pos: ")

		x_length = raw_input("Enter length of obstacle in X direction: ")
		while x_length < 0:
			x_length = raw_input("Length can only be a positive number. Re-enter: ")
		y_length = raw_input("Enter length of obstacle in Y direction: ")
		while y_length < 0:
			y_length = raw_input("Length can only be a positive number. Re-enter: ")
		z_height = raw_input("Enter height of obstacle in Z direction: ")
		while z_height < 0:
			z_height = raw_input("Height can only be a positive number. Re-enter: ")

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

	def deleteObstacle(self, _id):
		confirm = raw_input("Confirm delete obstacle %s? (Y/N): " % (_id))
		if "y" in confirm.lower():
			try:
				del self._obstacles[_id]
				print("Obstacle Deleted")
			except:
				print("Failed to delete obstacle")
		else:
			print("Obstacle not deleted")

	def viewAllObstacles(self):
		for _id, obstacle in self._obstacles.items():
			x_length = obstacle._x_right_bound - obstacle._x_left_bound
			y_length = obstacle._y_right_bound - obstacle._y_left_bound
			z_height = obstacle._z_upper_bound

			print("Obstacle ID: %s" % (obstacle._id)
			print("\tObstacle Position(x,y): (%s, %s)" % (obstacle._x_pos, obstacle._y_pos))
			print("\tX-Axis Length: %s" % (x_length))
			print("\tY-Axis Length: %s" % (y_length))
			print("\tZ-Axis Height: %s" % (z_height))
			print("")
