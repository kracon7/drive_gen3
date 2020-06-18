#! /usr/bin/env python
import os
import sys
import rospy
import time
import signal
import multiprocessing



def tool_vel():
	os.system('rostopic pub /my_gen3/in/cartesian_velocity kortex_driver/TwistCommand \
		"reference_frame: 0'
	+'\n'+'twist: {linear_x: 0.0, linear_y: 0.0, linear_z: 0.05, angular_x: 0.0, angular_y: 0.0,'
	+'\n'+'angular_z: 0.0}'
	+'\n'+'duration: 0"')

def stop_tool():
	os.system('rostopic pub /my_gen3/in/stop std_msgs/Empty "'+'{'+'}"')


if __name__ == '__main__':
    p = multiprocessing.Process(target=tool_vel, args=())
    s = multiprocessing.Process(target=stop_tool, args=())
    p.start()
    time.sleep(2)
    p.terminate()
    s.start()
    time.sleep(1)
    s.terminate()

