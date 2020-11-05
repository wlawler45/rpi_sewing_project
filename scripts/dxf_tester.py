import rospy
from std_msgs.msg import String
import time
#from python_dxf_parser import dxf_robot_motion

file_pub = rospy.Publisher('dxf_file_name', String, queue_size=10)
plan_pub = rospy.Publisher("plan_seam_motion", String, queue_size=10)
motion_pub = rospy.Publisher("move_to_seam_start", String, queue_size=10)
execute_pub = rospy.Publisher("execute_seam_motion", String, queue_size=10)
plan_execute_pub = rospy.Publisher("plan_and_execute_seam", String, queue_size=10)
#'plan_response_string', std_msgs.msg.String, queue_size=10)
#self.execute_seam_response_pub = rospy.Publisher('execute_seam_response_string', std_msgs.msg.String, queue_size=10)
#        self.plan_and_execute_response_pub = rospy.Publisher('plan_and_execute_seam_response_string',
rospy.init_node('node_name')
#r = rospy.Rate(10) # 10hz
time.sleep(1)
def callback(data):
    print(data)

#file_pub.publish("38 BACK DXF.dxf")
file_pub.publish("ARM_PANT_DXF.dxf")
#rospy.Subscriber("", String, callback)
file_response=rospy.wait_for_message("file_response_string",String)
print(file_response)
motion_pub.publish("out_seam")
time.sleep(5)
#move_response=rospy.wait_for_message("move_to_seam_response",String)
#print(move_response)
plan_pub.publish("out_seam")
plan_response=rospy.wait_for_message('plan_response_string',String)
print(plan_response)
execute_pub.publish("out_seam")
execute_response=rospy.wait_for_message('execute_seam_response_string',String)
print(execute_response)
#motion_pub.publish("out_seam")
#time.sleep(5)
#plan_execute_pub.publish("out_seam")
#plan_execute_response=rospy.wait_for_message('plan_and_execute_seam_response_string',String)
#print(plan_execute_response)


#file_response=rospy.wait_for_message("file_response_string",String)
rospy.spin()
r.sleep()
