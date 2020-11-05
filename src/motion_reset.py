from python_dxf_parser import dxf_robot_motion
import matplotlib
import rospy
import matplotlib.pyplot as plt
import numpy as np
import csv
import moveit_commander
import moveit_msgs.msg
import general_robotics_toolbox as rox
from geometry_msgs.msg import Pose, Point
import copy
import time
from industrial_msgs.msg import RobotStatus
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState




def main():
    rospy.init_node('dxf_parser')
    dxf_planner=dxf_robot_motion()
    dxf_planner.reset_move()
    

if __name__ == "__main__":
	main()
