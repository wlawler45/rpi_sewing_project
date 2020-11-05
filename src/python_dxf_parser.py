import dxfgrabber
import math
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

class dxf_robot_motion:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.move_group = moveit_commander.MoveGroupCommander("GP12_Planner")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        

    def distance_calculator(self,start,end):
        return math.sqrt( (start[0]-end[0])**2 + (start[1]-end[1])**2)

    def display_robot_trajectory(self,plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory);
        raw_input("Press Enter to continue...")
    
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
            wpose = self.move_group.get_current_pose().pose
            #print(wpose)
            wpose.position.z+=0.1
            self.move_group.set_pose_target(wpose)
            result=self.move_group.go(wait=True)
            plan,fraction,waypoints=self.plan_robot_motion(self.dxf_points,self.sines)
            self.execute_planned_trajectory(plan)
        
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
        
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.move_group.set_start_state(moveit_robot_state)
        #wpose = self.move_group.get_current_pose().pose
        #waypoints.append(wpose)
        #state = self.robot.get_current_state()
        #self.move_group.set_start_state(state)
        for i in range(len(dxf_points)):
            pose_goal = Pose()
            rpy=[math.pi,0.0,sines[i]]
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
        
        replan_value=0
        error_code=None
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
        
        
        while(replan_value<3 and fraction<1.0):
            print(fraction)
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
            replan_value+=1
            print("WARNING Portion of plan failed, replanning")
        
        
        
        
        return plan,fraction, waypoints


    def dxf_grabber_readfile(self,filename):
        dxf=dxfgrabber.readfile(filename)
        layer_count = len(dxf.layers)
        #print(layer_count)
        #print(len(dxf.blocks))
        all_blocks= [entity for entity in dxf.entities if entity.dxftype == 'INSERT']
        all_splines=[entity for entity in dxf.entities if entity.dxftype == 'SPLINE']
        #print(all_blocks[0].name)
        test_block=dxf.blocks[all_blocks[0].name]
        all_polylines= [entity for entity in test_block if entity.dxftype == 'POLYLINE']
        #for entity in test_block:
        #    print(entity.dxftype)
        #print(len(all_polylines))
        x=0
        last_in=False
        removed=[]
        flag=False
        vertices=[]
        sine_vals=[]
        with open('eggs.csv', 'w') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=' ',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
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
            shape=matplotlib.path.Path(vertices,closed=True)       
            colors=['b','g','r','c','m','y','k']
            for entity in all_polylines:
                color=colors[x]
                sine_vals=[]
                motion_points=[]
                for i in range(len(entity.points)-1):
                    
                    plt.plot([entity.points[i][0],entity.points[i+1][0]],[entity.points[i][1],entity.points[i+1][1]],color)
                    spamwriter.writerow("%f %f"%(entity.points[i][0],entity.points[i][1]))
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
                print(sine_vals)
                plan, fraction, points=self.plan_robot_motion(motion_points,sine_vals)
                #if(fraction<1.0):
                #    self.execute_consecutive_motions(points)
                #else:                
                self.display_robot_trajectory(plan)
                self.execute_planned_trajectory(plan)
                if(x+1>len(colors)):
                    x=0
                spamwriter.writerow("")
                print("end")
            
        #print(outside)
        plt.show()
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
    file=dxf_planner.dxf_grabber_readfile(filename)
    

if __name__ == "__main__":
	main()
