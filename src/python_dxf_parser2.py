import dxfgrabber
import math
import matplotlib
import rospy
import actionlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import moveit_commander
import moveit_msgs.msg
import general_robotics_toolbox as rox
from geometry_msgs.msg import Pose, Point
import copy
from copy import copy
import time
from industrial_msgs.msg import RobotStatus
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import std_msgs
import control_msgs.msg
import trajectory_msgs.msg
from std_msgs.msg import String
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

class Trajectory(object):
    def __init__(self):
        
        self._client = actionlib.SimpleActionClient(
            'joint_trajectory_action',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        print(time)
        #point.time_from_start.from_sec(time)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        joint_state=rospy.wait_for_message("/joint_states",JointState)
        self._goal.trajectory.joint_names = joint_state.name


class dxf_robot_motion:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.seam_point_dict={}
        self.plan_seam_dict={}
        self.speed=1.0
        self.display=True
        robot_urdf = URDF.from_parameter_server()
        #robot_urdf = URDF.from_xml_string(urdf_str)
        self.kdl_kin = KDLKinematics(robot_urdf, "base_link", "tool0")
        self.move_group = moveit_commander.MoveGroupCommander("GP12_Planner")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        self.dxf_file_response_pub = rospy.Publisher('file_response_string', std_msgs.msg.String, queue_size=10)
        self.plan_response_pub = rospy.Publisher('plan_response_string', std_msgs.msg.String, queue_size=10)
        self.execute_seam_response_pub = rospy.Publisher('execute_seam_response_string', std_msgs.msg.String, queue_size=10)
        self.plan_and_execute_response_pub = rospy.Publisher('plan_and_execute_seam_response_string', std_msgs.msg.String, queue_size=10)
        self.move_to_stream_start_response_pub = rospy.Publisher('move_to_seam_response', std_msgs.msg.String, queue_size=10)
        #Ros message used to pass dxf file name to node and process dxf file
        rospy.Subscriber("dxf_file_name", String, self.dxf_grabber_readfile)
        #Ros message used to plan and view motion before returning
        rospy.Subscriber("plan_seam_motion", String, self.plan_robot_motion_call)
        rospy.Subscriber("execute_seam_motion", String, self.execute_robot_motion_call)
        rospy.Subscriber("plan_and_execute_seam", String, self.plan_and_execute_call)
        rospy.Subscriber("move_to_seam_start", String, self.move_to_seam_start_call)
        self.followjointaction= rospy.Publisher("joint_trajectory_action/goal", control_msgs.msg.FollowJointTrajectoryActionGoal, queue_size=10)
    
    def move_to_seam_start_call(self,data):
        if(self.seam_point_dict.has_key(data.data)):
            motion_point=self.seam_point_dict[data.data][0][0]
            sine_val=self.seam_point_dict[data.data][1][0]
            self.move_to_seam_start(motion_point,sine_val)
            #print("finished_move")
            self.move_to_stream_start_response_pub.publish("Moved to start of seam")
            print("finished_move")
        else:
            self.move_to_stream_start_response_pub.publish("Seam key does not exist in loaded dictionary")
        
    def plan_robot_motion_call(self,data):
        if(self.seam_point_dict.has_key(data.data)):
            motion_points=self.seam_point_dict[data.data][0]
            sine_vals=self.seam_point_dict[data.data][1]
            self.display=True
            plan,fraction,waypoints=self.plan_robot_motion(motion_points,sine_vals)
            if(fraction==0.0):
                self.plan_response_pub.publish("Failed to plan")
            else:
                self.plan_seam_dict[data.data]=plan
                self.plan_response_pub.publish("Planning successful")
        else:
            self.plan_response_pub.publish("Planned seam key does not exist in loaded dictionary")
    
    def execute_robot_motion_call(self,data):
        if(self.plan_seam_dict.has_key(data.data)):
            plan=self.plan_seam_dict[data.data]
            self.execute_planned_trajectory(plan)
            
            self.execute_seam_response_pub.publish("Execution successful")
        else:
            self.execute_seam_response_pub.publish("Could not find plan in dictionary of planned seams, did you publish to plan_seam_motion first?")

    def plan_and_execute_call(self,data):
        if(self.seam_point_dict.has_key(data.data)):
            motion_points=self.seam_point_dict[data.data][0]
            sine_vals=self.seam_point_dict[data.data][1]
            self.display=False
            plan,fraction,waypoints=self.plan_robot_motion(motion_points,sine_vals)
            self.execute_planned_trajectory(plan)
            self.plan_and_execute_response_pub.publish("Planning and execution successful")
        else:
            self.plan_and_execute_response_pub.publish("Planned seam key does not exist in loaded dictionary")

    def distance_calculator(self,start,end):
        return math.sqrt( (start[0]-end[0])**2 + (start[1]-end[1])**2)

    def display_robot_trajectory(self,plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory);
        raw_input("Press Enter to continue...")
    
    def move_to_home(self):
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x=0.0
        wpose.position.y=0.0
        wpose.position.z=0.0
        wpose.orientation.w=0.0
        wpose.orientation.x=0.0
        wpose.orientation.y=0.0
        wpose.orientation.z=0.0
        self.move_group.set_pose_target(wpose)
        result=self.move_group.go(wait=True)

    def move_to_seam_start(self,motion_point,sine):
        
        pose_goal = Pose()
        rpy=[math.pi,0.0,sine-math.pi/2]
        print(rpy)
        R_result=rox.rpy2R(rpy)
        quat=rox.R2q(R_result)
        print(quat)
        pose_goal.orientation.w=0.0
        pose_goal.orientation.x=quat[1]
        pose_goal.orientation.y=quat[2]
        pose_goal.orientation.z=0.0
        #20- sets setpoint in middle of dxf file for robot y axis
        #middle of dxf y axis is 0, so no centering needed here
        x_val=(20-motion_point[0]) *0.0254
        y_val=motion_point[1]*0.0254
        pose_goal.position.x=0.8+y_val

        pose_goal.position.y=x_val
        pose_goal.position.z=0.3
        pose_goal.position.x+=0.1
        #self.robot.set_start_state_to_current_state()
        self.move_group.set_pose_target(pose_goal)
        result=self.move_group.go(wait=True)

        

    def reset_move(self):
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x=0.7
        wpose.position.y=0.56263286
        wpose.position.z=0.3
        wpose.orientation.w=-0.000164824011947
        wpose.orientation.x=1.0
        wpose.orientation.y=0.0
        wpose.orientation.z=0.0
        self.move_group.set_pose_target(wpose)
        result=self.move_group.go(wait=True)

    def execute_consecutive_motions(self,points):
        i=0
        #wpose = self.move_group.get_current_pose().pose
        #self.move_group.set_pose_target(wpose)
        

        #plan=self.move_group.go(wpose,wait=True)
        #self.move_group.execute(plan)
        for i in points:
            self.move_group.set_pose_target(i)
            raw_input("Press Enter to continue...")
            plan=self.move_group.go(wait=True)
            self.move_group.stop()
            #self.display_robot_trajectory(plan)
            #self.move_group.execute(plan)
        
    def execute_planned_trajectory(self,plan):
        #wpose = self.move_group.get_current_pose().pose
        #self.move_group.set_pose_target(wpose)
        #time.sleep(0.5)
        #result=self.move_group.go(wait=True)
        #self.move_group.stop()
        #print("plan:")
        #print(plan)
        
        message=rospy.wait_for_message("/robot_status",RobotStatus)
        joint_state=rospy.wait_for_message("/joint_states",JointState)
        while(message.in_motion.val==1):
            print(message)
            message=rospy.wait_for_message("/robot_status",RobotStatus)
        #self.move_group.clear_pose_targets()
        #print(joint_state)
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.move_group.set_start_state(moveit_robot_state)
        #add check to see if plan start state agrees with start state or consider adding back in getting current pose and appending it to waypoints potentially
        result=self.move_group.execute(plan, wait=True)
        #print(result)
        if(not result):
            print("not executed")
            self.execute_seam_response_pub.publish("Execution Failed initially but retrying")
            wpose = self.move_group.get_current_pose().pose
            #print(wpose)
            wpose.position.z+=0.1
            self.move_group.set_pose_target(wpose)
            result=self.move_group.go(wait=True)
            plan,fraction,waypoints=self.plan_robot_motion(self.dxf_points,self.sines)
            
            self.execute_planned_trajectory(plan)
            
        else:
        
            self.execute_seam_response_pub.publish("Executed Successfully")

        #time.sleep(1)

    def plan_robot_motion(self,dxf_points,sines):
        self.dxf_points=dxf_points
        self.sines=sines
        waypoints=[]
        message=rospy.wait_for_message("/robot_status",RobotStatus)
        joint_state=rospy.wait_for_message("/joint_states",JointState)
        
        while(message.in_motion.val==1):
            message=rospy.wait_for_message("/robot_status",RobotStatus)
        #self.move_group.clear_pose_targets()
        print("hello")
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.move_group.set_start_state(moveit_robot_state)
        #waypoints.append(self.move_group.get_current_pose().pose)
        #wpose = self.move_group.get_current_pose().pose
        #waypoints.append(wpose)
        #state = self.robot.get_current_state()
        #self.move_group.set_start_state(state)
        
        
        for i in range(len(dxf_points)):
            pose_goal = Pose()
            rpy=[math.pi,0.0,sines[i]-math.pi/2]
            print(rpy)
            R_result=rox.rpy2R(rpy)
            quat=rox.R2q(R_result)
            print(quat)
            pose_goal.orientation.w=0.0
            pose_goal.orientation.x=quat[1]
            pose_goal.orientation.y=quat[2]
            pose_goal.orientation.z=0.0
            #20- sets setpoint in middle of dxf file for robot y axis
            #middle of dxf y axis is 0, so no centering needed here
            x_val=(20-dxf_points[i][0]) *0.0254
            y_val=dxf_points[i][1]*0.0254
            pose_goal.position.x=0.8+y_val
            pose_goal.position.y=x_val
            pose_goal.position.z=0.3
            print(pose_goal)
            
            waypoints.append(pose_goal)

        """
        euclidean_distances=[]
        for i in range(1,len(waypoints)):
            distance=pow(pow(waypoints[i].position.x-waypoints[i-1].position.x,2)+pow(waypoints[i].position.y-waypoints[i-1].position.y,2)+pow(waypoints[i].position.z-waypoints[i-1].position.z,2),0.5)
            print(distance)
            euclidean_distances.append(distance)
        
        error_code=None
        """
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
        
        print(type(plan))
        #self.scene.motion_plan_request
        replan_value=0
        while(replan_value<3 and fraction<1.0):
            print(fraction)
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
            replan_value+=1
            
            print("WARNING Portion of plan failed, replanning")
        if(replan_value>2):
            return 0,0,0
        #print(len(euclidean_distances))
        #print(len(plan.joint_trajectory.points))
        #print(waypoints[0])
        
        #print(self.kdl_kin.forward(plan.joint_trajectory.points[0].positions))
        #print(self.kdl_kin.forward(plan.joint_trajectory.points[0].positions).item(3))
        #print(self.kdl_kin.forward(plan.joint_trajectory.points[0].positions).item(7))
        #print(self.kdl_kin.forward(plan.joint_trajectory.points[0].positions).item(11))
        #print(plan.joint_trajectory.points)
        total_distance=0
        distances=[]
        for i in range(1,len(plan.joint_trajectory.points)):
            old_cartesian=self.kdl_kin.forward(plan.joint_trajectory.points[i-1].positions)
            new_cartesian=self.kdl_kin.forward(plan.joint_trajectory.points[i].positions)
            distance=pow(pow(new_cartesian.item(3)-old_cartesian.item(3),2)+pow(new_cartesian.item(7)-old_cartesian.item(7),2)+pow(new_cartesian.item(11)-old_cartesian.item(11),2),0.5)
            distances.append(distance)
            total_distance+=distance
            #new_time=plan.joint_trajectory.points[i].time_from_start.to_sec()+(distance/self.speed)
            #old_time=plan.joint_trajectory.points[i].time_from_start.to_sec()
            #print("new time")
            #print(new_time)
            #print(distance/self.speed)
            #print(old_time)
            #if(new_time > old_time):
            #    plan.joint_trajectory.points[i].time_from_start.from_sec(new_time)
            #else:
                
            #    print("Error, speed faster than possible execution time")
                
        print(total_distance)
        print(distances)
        total_time=plan.joint_trajectory.points[-1].time_from_start.to_sec()
        print(total_time)
        times=[]
        for i in distances:
            new_time=(i/total_distance) *total_time
            times.append(new_time)
        
        

        """
        if(new_timestamp > old_timestamp)
        next_waypoint->time_from_start.fromSec(new_timestamp);
        else
        {
            //ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow.");
        }"""
        #print point.time_from_start.secs
        #for i in range(len(plan.joint_trajectory.points)-1):
         #   print("time between points:\n")
        #    print(plan.joint_trajectory.points[i+1].time_from_start.nsecs-plan.joint_trajectory.points[i].time_from_start.nsecs)
        #for i in range(1,len(plan.joint_trajectory.points)-1):
        #    plan.joint_trajectory.points[i].velocities=[-0.1,0.05,0.11,0.00000001,-0.05,0.2]


        #plan=self.move_group.retime_trajectory(moveit_robot_state,plan,velocity_scaling_factor=1.0, algorithm="iterative_spline_parameterization")
        if(self.display):
            self.display_robot_trajectory(plan) 
        #self.actiongoal= control_msgs.msg.FollowJointTrajectoryActionGoal()
        self.goal = control_msgs.msg.FollowJointTrajectoryGoal()
        #self.actiongoal.header=std_msgs.msg.Header()
        self.goal.trajectory.joint_names=joint_state.name
        #self.goal.trajectory.points.resize(len(plan.joint_trajectory.points))
        traj=Trajectory()
        time=0.01
        traj.add_point(plan.joint_trajectory.points[0].positions,time)
        #time=0.0
        for i in range(1,len(plan.joint_trajectory.points)):
            time+=times[i-1]
            traj.add_point(plan.joint_trajectory.points[i].positions,time)
        
        #traj.start()
        #traj.wait(15.0)


        """
            trajectory_point=trajectory_msgs.msg.JointTrajectoryPoint()
            
            trajectory_point.positions=plan.joint_trajectory.points[i].positions
            
            trajectory_point.velocities=plan.joint_trajectory.points[i].velocities
            #trajectory_point.accelerations=plan.joint_trajectory.points[i].accelerations
            trajectory_point.time_from_start=plan.joint_trajectory.points[i].time_from_start
            
            self.goal.trajectory.points.append(trajectory_point)
        
        print(len(plan.joint_trajectory.points))
        
        
        self.actiongoal.goal=self.goal
        self.followjointaction.publish(self.actiongoal)
        print("published goal")
        """
        return plan,fraction, waypoints


    def dxf_grabber_readfile(self,data):
        filename=data.data
        dxf=dxfgrabber.readfile(filename)
        layer_count = len(dxf.layers)
        self.seam_point_dict={}
        #print(layer_count)
        #print(len(dxf.blocks))
        for entity in dxf.entities:
            print(entity.dxftype)
        all_blocks= [entity for entity in dxf.entities if entity.dxftype == 'INSERT']
        all_splines=[entity for entity in dxf.entities if entity.dxftype == 'SPLINE']
        #print(all_splines)
        if(len(all_blocks)!=0):
            test_block=dxf.blocks[all_blocks[0].name]
            all_polylines= [entity for entity in test_block if entity.dxftype == 'POLYLINE']
        else:
            all_polylines=[entity for entity in dxf.entities if entity.dxftype == 'POLYLINE']
        
        #print(len(all_polylines))
        x=0
        last_in=False
        removed=[]
        flag=False
        vertices=[]
        sine_vals=[]
        #with open('eggs.csv', 'w') as csvfile:
        #    spamwriter = csv.writer(csvfile, delimiter=' ',
        #                        quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for i in range(len(all_polylines)):
            start=all_polylines[i].points[0]
            end=all_polylines[i].points[-1]
            
            
            for e in range(len(all_polylines)):
                if(i==e):
                    continue
                else:
                
                    if(all_polylines[e].points[0]==start and all_polylines[e].points[-1]==end):
                        #print("identical start found")
                        #print(start)
                        #print(i)
                        #print(e)
                    #if(all_polylines[e].points[-1]==end):
                        #print("identical end found")
                        #print(end)
                        #print(i)
                        #print(e)
                        if(len(all_polylines[i].points)==len(all_polylines[e].points) and not flag):
                            flag=True
                            removed.append(all_polylines[e])
                        if(len(all_polylines[i].points)>len(all_polylines[e].points)):
                            
                            #print(len(all_polylines[i].points))
                            #print(len(all_polylines[e].points))
                            removed.append(all_polylines[e])
                            
        
        #print(vertices)
        outside=0
        for entry in removed:
            
            all_polylines.remove(entry)
        #print(len(all_polylines))    
        for i in range(len(all_polylines)):
            for point in all_polylines[i].points:
                vertices.append(point)
        try:
            shape=matplotlib.path.Path(vertices,closed=True)       
        except:
            self.dxf_file_response_pub.publish(String("dxf file chosen does not contain a closed path, are you sure the file is valid?"))         
        colors=['b','g','r','c','m','y','k']
        seam_counter=0
        for entity in all_polylines:
            color=colors[x]
            sine_vals=[]
            motion_points=[]
            for i in range(len(entity.points)-1):
                
                plt.plot([entity.points[i][0],entity.points[i+1][0]],[entity.points[i][1],entity.points[i+1][1]],color)
                #spamwriter.writerow("%f %f"%(entity.points[i][0],entity.points[i][1]))
                if(i==0):
                    magnitude=self.distance_calculator(entity.points[i],entity.points[i+1])
                    x_vals=[entity.points[i][0],entity.points[i+1][0]]
                    y_vals=[entity.points[i][1],entity.points[i+1][1]]
                else:
                    
                    magnitude=self.distance_calculator(entity.points[i-1],entity.points[i+1])
                    x_vals=[entity.points[i-1][0],entity.points[i][0],entity.points[i+1][0]]
                    y_vals=[entity.points[i-1][1],entity.points[i][1],entity.points[i+1][1]]
                
                #bestfitlineslope, offset=np.polyfit(x_vals,y_vals,1)
                
                #m=(entity.points[i+1][1]-entity.points[i][1])/(entity.points[i+1][0]-entity.points[i][0])
                #print(entity.points[i+1][0]-entity.points[i][0])
                
                #print(magnitude/bestfitlineslope)
                
                bestfitlineslope, offset=np.polyfit(x_vals,y_vals,1)
                #print("best fit line:")
                #print(bestfitlineslope)
                #print(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))]))
                #plt.arrow(entity.points[i][0],entity.points[i][1],bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),1/(math.sqrt(bestfitlineslope**2+1)))
                #plt.plot(entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1)),'bo')                
                if(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))])):
                    outside+=1
                    value=bestfitlineslope/(math.sqrt(bestfitlineslope**2+1))/(-1/(math.sqrt(bestfitlineslope**2+1)))
                    plt.arrow(entity.points[i][0],entity.points[i][1],bestfitlineslope/(math.sqrt(bestfitlineslope**2+1)),-1/(math.sqrt(bestfitlineslope**2+1)))
                    sine_vals.append(math.atan2(-1/(math.sqrt(bestfitlineslope**2+1)),bestfitlineslope/(math.sqrt(bestfitlineslope**2+1))))
                else:
                    value=bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1))/(1/(math.sqrt(bestfitlineslope**2+1)))
                    #print([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))])
                    plt.arrow(entity.points[i][0],entity.points[i][1],bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),1/(math.sqrt(bestfitlineslope**2+1)))
                    sine_vals.append(math.atan2(1/(math.sqrt(bestfitlineslope**2+1)),bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1))))
                    #plt.plot(entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1)),'bo')
                #plt.arrow(entity.points[i][0],entity.points[i][1],-(entity.points[i+1][1]-entity.points[i][1])/magnitude,(entity.points[i+1][0]-entity.points[i][0])/magnitude)
                
                #print(distance_calculator(entity.points[i],[entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))]))
                #print(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))]))
                #print(end)
                #plt.plot([entity.points[i][0],entity.points[i][1]],[entity.points[i][0]-end,entity.points[i][1]+end])

                
                motion_points.append(entity.points[i])
                

            x_vals=[entity.points[-2][0],entity.points[-1][0]]
            y_vals=[entity.points[-2][1],entity.points[-1][1]]
            bestfitlineslope, offset=np.polyfit(x_vals,y_vals,1)
            if(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))])):
                outside+=1
                
                value=bestfitlineslope/(math.sqrt(bestfitlineslope**2+1))/(-1/(math.sqrt(bestfitlineslope**2+1)))  
                plt.arrow(entity.points[-1][0],entity.points[-1][1],bestfitlineslope/(math.sqrt(bestfitlineslope**2+1)),-1/(math.sqrt(bestfitlineslope**2+1)))
                sine_vals.append(math.atan2(-1/(math.sqrt(bestfitlineslope**2+1)),bestfitlineslope/(math.sqrt(bestfitlineslope**2+1))))
            else:
                plt.arrow(entity.points[-1][0],entity.points[-1][1],bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),1/(math.sqrt(bestfitlineslope**2+1)))
                sine_vals.append(math.atan2(1/(math.sqrt(bestfitlineslope**2+1)),bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1))))
            x+=1
            #sine_vals.append(math.asin(value))
            motion_points.append(entity.points[-1])
            #print(sine_vals)
            if(seam_counter==0):                
                self.seam_point_dict["bottom_seam"]=[motion_points,sine_vals]
            elif(seam_counter==1):
                self.seam_point_dict["in_seam"]=[motion_points,sine_vals]
            elif(seam_counter==2):
                self.seam_point_dict["j_seam"]=[motion_points,sine_vals]
            elif(seam_counter==3):
                self.seam_point_dict["top_seam"]=[motion_points,sine_vals]
            elif(seam_counter==4):
                self.seam_point_dict["out_seam"]=[motion_points,sine_vals]
            else:
                self.dxf_file_response_pub.publish(String("dxf file contained too many seams, check to see if dxf file is valid"))

            #plan, fraction, points=self.plan_robot_motion(motion_points,sine_vals)
            #if(fraction<1.0):
            #    self.execute_consecutive_motions(points)
            #else:
            
            #self.display_robot_trajectory(plan)
            #self.execute_planned_trajectory(plan)
            if(x+1>len(colors)):
                x=0
            seam_counter+=1
            #spamwriter.writerow("")
            #print("end")
        #print(self.seam_point_dict)
        self.dxf_file_response_pub.publish(String("Successfully parsed dxf file"))
        
        #print(outside)
        #plt.show()
        """if(all_lines[0].start[0]>all_lines[1].start[0]):
            bottom_hem=all_lines[0]
            waist_hem=all_lines[1]
        else:
            bottom_hem=all_lines[1]
            waist_hem=all_lines[0]
        print(bottom_hem.start)
        print(bottom_hem.end)
        print(distance_calculator(all_lines[0].start,all_lines[0].end))
        print(waist_hem.start)
        print(waist_hem.end)
        print(distance_calculator(all_lines[1].start,all_lines[1].end))
        for control_points in all_splines[0].fit_points:
            print(control_points)
            
        """
    
    


def main():
    rospy.init_node('dxf_parser')
    dxf_planner=dxf_robot_motion()
    filename="ARM_PANT_DXF.dxf"
    #dxf_planner.dxf_grabber_readfile(filename)
    rospy.spin()
    

if __name__ == "__main__":
	main()
