#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import TwistStamped

def talker():
    pub = rospy.Publisher("/rmp220/base/vel_cmd", TwistStamped, queue_size=100)
    rospy.init_node("segway_controller", anonymous=True)

    vel = TwistStamped()

    """
    # Linear motion for 2 sec.
    vel.twist.linear.x  = 0.2
    vel.twist.angular.z = 0.0
    pub.publish(vel)
    time.sleep(2)
    """

    """
    vel.twist.linear.x  = 0.0
    vel.twist.angular.z = 0.785
    pub.publish(vel)
    time.sleep(0.333) # Turn 15 degree
    """

    rospy.loginfo("End of publishing.")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
