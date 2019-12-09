import rospy
from occupancy_grid import OccupancyGrid2d
import std_msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData

def main():
    rospy.init_node('runtime', anonymous=True)
    
    map_pub = rospy.Publisher("/occupancyGrid2d", OccupancyGrid, queue_size=10)
    rospy.Subscriber("/obstacles", obstacles, callback)

    rate = rospy.rate(10)
    
    while not rospy.is_shutdown():
        
        message = OccupancyGrid()
        map_pub.publish(message)
        rate.sleep()
        
    rospy.spin()


def callback(data):



if __name__ == '__main__':
    main()
