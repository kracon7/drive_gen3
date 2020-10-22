#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray


def talker():
    pub = rospy.Publisher('mytopic', Int16MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    hello_float = Int16MultiArray()
    hello_float.data = []


    a=0
    while not rospy.is_shutdown():
        if a==0:
                hello_float.data = [380,399,380,380,380,380,380,380]
                rospy.loginfo(hello_float)
                pub.publish(hello_float)
                a=1
        elif a==1:
                hello_float.data = [377,399,380,380,380,380,380,380]
                rospy.loginfo(hello_float)
                pub.publish(hello_float)
                a=0


        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass