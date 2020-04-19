#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import RobotState
import numpy as np
import numpy.matlib
from geometry_msgs.msg import Pose
import math


class sys_class():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "sim_sys"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    
    def compute_inverse_kinematics(self,end_effector_pose):
        '''
        compute the inverse kinematics of the given end effector pose,return joint values,end_effector_pose should be a pose
        '''
        request=GetPositionIKRequest()
        request.ik_request.group_name=self.group_name
        request.ik_request.ik_link_name = "pan_link"
        request.ik_request.attempts = 100
        request.ik_request.pose_stamped.header.frame_id = "wx"
        request.ik_request.pose_stamped.pose.position.x = end_effector_pose.position.x
        request.ik_request.pose_stamped.pose.position.y = end_effector_pose.position.y
        request.ik_request.pose_stamped.pose.position.z = end_effector_pose.position.z
        request.ik_request.pose_stamped.pose.orientation.x = end_effector_pose.orientation.x
        request.ik_request.pose_stamped.pose.orientation.y = end_effector_pose.orientation.y
        request.ik_request.pose_stamped.pose.orientation.z = end_effector_pose.orientation.z
        request.ik_request.pose_stamped.pose.orientation.w = end_effector_pose.orientation.w
        ik_response=self.compute_ik(request)
        #print(ik_response)
        joint_value=ik_response.solution.joint_state.position
        joint_values=[]
        if len(joint_value) < 10:
            rospy.logerr('the given end_effector_pose has no results')
            return joint_values
        else:
            for i in range(len(joint_value)):
                joint_values.append(joint_value[i])
            #print(joint_value)
            return joint_values
    
    def compute_forward_kinematics(self,joint_values,goal_link):
        '''
        compute the forward kinematics of the given joint values with reference to the reference link, return a posestamped
        '''
        fk_request=GetPositionFKRequest()
        links=self.robot.get_link_names()
        fk_request.fk_link_names=links
        state=RobotState()
        joint_names=['wx_agv2_1','agv2_virx','agv2_viry','agv2_virz','joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6','joint_a7']
        state.joint_state.name=joint_names
        state.joint_state.position=joint_values
        fk_request.robot_state=state
        fk_response=self.compute_fk(fk_request)
        index=fk_response.fk_link_names.index(goal_link)
        end_effector_pose=fk_response.pose_stamped[index]
        return end_effector_pose

    
    def compute_jacobian_matrix(self,joint_value):
        '''
        compute the jacobian matrix of the given joint value,the given joint value should be (1,6) array
        '''
        if len(joint_value) < 6:
            r=[]
            return []
        else:
            jacobian_matrix_m=numpy.matlib.zeros((6,6))
            jacobian_matrix_m=self.group.get_jacobian_matrix(joint_value)
            jacobian_matrix=np.asarray(jacobian_matrix_m)
            return jacobian_matrix

def quaternion_to_euler(given_oriantation):
    '''
    transfor the given orientation into the ruler angle,return R,P,Y
    '''
    R=math.atan2(2*(given_oriantation.w*given_oriantation.x+given_oriantation.y*given_oriantation.z),1-2*(math.pow(given_oriantation.x,2)+math.pow(given_oriantation.y,2)))
    P=math.asin(2*(given_oriantation.w*given_oriantation.y-given_oriantation.x*given_oriantation.z))
    Y=math.atan2(2*(given_oriantation.w*given_oriantation.z+given_oriantation.x*given_oriantation.y),1-2*(math.pow(given_oriantation.y,2)+math.pow(given_oriantation.z,2)))
    return R,P,Y

def euler_to_quaternion(R,P,Y):
    '''
    transfor the given ruler angle to orientation ,return orientation
    '''
    p=Pose()
    p.orientation.w=math.cos(R/2)*math.cos(P/2)*math.cos(Y/2)+math.sin(R/2)*math.sin(P/2)*math.sin(Y/2)
    p.orientation.x=math.sin(R/2)*math.cos(P/2)*math.cos(Y/2)-math.cos(R/2)*math.sin(P/2)*math.sin(Y/2)
    p.orientation.y=math.cos(R/2)*math.sin(P/2)*math.cos(Y/2)+math.sin(R/2)*math.cos(P/2)*math.sin(Y/2)
    p.orientation.z=math.cos(R/2)*math.cos(P/2)*math.sin(Y/2)-math.sin(R/2)*math.sin(P/2)*math.cos(Y/2)
    return p.orientation




class trajectory_class():
    def __init__(self):
        self.omega=5/math.pi
        self.period=30
    
    def desired_trajectory(self,time):
        '''
        generate the desired trajectory, return a pose. the function can be replaced by a trajectory from the simulator 
        '''
        desired_pose=Pose()
        desired_pose.position.x=3
        desired_pose.position.y=time*0.1+0.15
        desired_pose.position.z=1
        desired_pose.orientation.x=0.5
        desired_pose.orientation.y=0.5
        desired_pose.orientation.z=0.5
        desired_pose.orientation.w=0.5
        return desired_pose

    def nutation(self,time):
        '''
        according omega,period and time , this function generate the nutation of the wx.retrun a Pose
        '''
        R=self.omega*math.sin(2*math.pi/self.period*time)
        P=self.omega*math.cos(2*math.pi/self.period*time)
        Y=0
        quaternion=euler_to_quaternion(R,P,Y)
        nutation_pose=Pose()
        nutation_pose.orientation=quaternion

        return nutation_pose



class planner_class():
    def __init__(self):
        self.trajectory=trajectory_class()
        self.sys=sys_class()
        self.debug=False
        self.time_duration=0.5
        self.total_time=10
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory.txt','w') as f:
            f.write('this is the MBX pose,the info is time,positionx,positiony,positionz,orientationw,orientationx,orientationy,orientationz'+'\r\n')
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory.txt','w') as f:
            f.write('this is the FWX joint values,the following info is time,x,y,theta,joint1,joint2,joint3,joint4,joint5,joint6,joint7'+'\r\n')
    
    def write_mbx_info(self,time,pose):
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory.txt','a') as f:
            f.write(str(time)+' '+str(pose.position.x)+' '+str(pose.position.y)+' '+str(pose.position.z)+' '+str(pose.orientation.w)+\
            ' '+str(pose.orientation.x)+' '+str(pose.orientation.y)+' '+str(pose.orientation.z)+'\r\n')
    
    def write_fwx_info(self,time,joint_value):
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory.txt','a') as f:
            f.write(str(time)+' '+str(joint_value[1]/2)+' '+str(joint_value[2]/2)+' '+str(joint_value[3]/2)+' '+str(joint_value[4])+\
            ' '+str(joint_value[5])+' '+str(joint_value[6])+' '+str(joint_value[7])+' '+str(joint_value[8])+' '+str(joint_value[9])+\
            ' '+str(joint_value[10])+'\r\n')

    def add_nutation(self,joint_values,time):
        '''
        this function generate the movement of wx with nutation according to the joint values and nutation function. return time and pose
        '''
        nutation_pose=self.trajectory.nutation(time)
        wx_pose=Pose()
        wx_pose.position.x=joint_values[1]/2
        wx_pose.position.y=joint_values[2]/2
        wx_pose.position.z=1 #this z value is NOT acurate. need to be confermed
        R,P,Y=quaternion_to_euler(nutation_pose.orientation)
        R+=joint_values[0]
        Y+=joint_values[3]/2
        wx_pose.orientation=euler_to_quaternion(R,P,Y)
        return time,wx_pose
    
    def plan(self):
        count=0
        while count*self.time_duration < self.total_time:
            desired_pose=self.trajectory.desired_trajectory(count*self.time_duration)
            joint_values=self.sys.compute_inverse_kinematics(desired_pose)
            if self.debug:
                print(joint_values)
            if len(joint_values)>0:
                time,wx_pose=self.add_nutation(joint_values,count*self.time_duration)
                rospy.loginfo('writing trajectory')
                self.write_mbx_info(time,wx_pose)
                self.write_fwx_info(time,joint_values)
                count+=1
            else:
                rospy.logerr('no IKsolution found. exit the program')
                exit()



def main():

    DEBUG=False

    rospy.init_node('sys_planner')
    while rospy.get_time()==0:
        continue
    planner=planner_class()
    if DEBUG:
        end_pose=Pose()
        end_pose.position.x=3
        end_pose.position.y=0
        end_pose.position.z=0.5
        end_pose.orientation.x=0.5
        end_pose.orientation.y=0.5
        end_pose.orientation.z=0.5
        end_pose.orientation.w=0.5
        joint_values=planner.sys.compute_inverse_kinematics(end_pose)
        print(joint_values)
        goal_link='agv_link'
        e_pose=planner.sys.compute_forward_kinematics(joint_values,goal_link)
        print(e_pose)

    planner.plan()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():


        rate.sleep()


if __name__ == '__main__':
    main()