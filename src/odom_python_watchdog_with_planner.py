#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64

high_level_cmd_pub = rospy.Publisher('high_level_cmd', String, queue_size=10)
odom_x_pub = rospy.Publisher('odom_x', Float64, queue_size=10)
odom_y_pub = rospy.Publisher('odom_y', Float64, queue_size=10)

def odomCallback(data):
    rospy.loginfo("Odometry X:%f Y:%f", data.pose.pose.position.x, data.pose.pose.position.y)

    odom_x_pub.publish(data.pose.pose.position.x)
    odom_y_pub.publish(data.pose.pose.position.y)

    high_level_cmd_str = "stop"

    if( data.pose.pose.position.x < 2.0 ):
        high_level_cmd_str = "forward"
    else:
        high_level_cmd_str = "stop"

    high_level_cmd_pub.publish(high_level_cmd_str);

    
def odomSubscriber():
    rospy.init_node('odom_python_watchdog_with_planner', anonymous=False)

    rospy.Subscriber("/riss_bot/odom", Odometry, odomCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    odomSubscriber()
