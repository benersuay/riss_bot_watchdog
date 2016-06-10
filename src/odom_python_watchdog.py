#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def odomCallback(data):
    rospy.loginfo("Odometry X:%f Y:%f", data.pose.pose.position.x, data.pose.pose.position.y)
    
def odomSubscriber():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odom_python_watchdog', anonymous=False)

    rospy.Subscriber("/riss_bot/odom", Odometry, odomCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    odomSubscriber()
