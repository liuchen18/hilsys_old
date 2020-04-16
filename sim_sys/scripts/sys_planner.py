#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPossitionIK, GetPositionIKRequest, GetPositionIKResponse
import numpy as np
import numpy.matlib
from geometry_msgs.msg import Pose


class sys_class():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "sim_sys"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
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


class trajectory_class():
    def __init__(self,flag=True):
        self.got_tra=flag
    
    def desired_trajectory(self,time):
        '''
        generate the desired trajectory, return a pose. the function can be replaced by a trajectory from the simulator 
        '''
        desired_pose=Pose()
        desired_pose.position.x=3
        desired_pose.position.y=time*0.1+0.15
        desired_pose.position.z=1.0-0.1*math.sin(time/2)
        desired_pose.orientation.x=0.5
        desired_pose.orientation.y=0.5
        desired_pose.orientation.z=0.5
        desired_pose.orientation.w=0.5
        return desired_pose
    def nutation(self,omega,period,time):
        '''
        according omega,period and time , this function generate the nutation of the wx
        '''
        nutation_pose=Pose()

        return nutation_pose



class planner_class():
    def __init__(self):
        self.trajectory=trajectory_class(False)
        self.sys=sys_class()
        self.debug=True
        self.time_duration=0.5
        self.total_time=10
    
    def write_info(self,info):
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_trajectory.txt','a') as f:
            f.write(str(info[0])+' '+str(info[1])+' '+str(info[2])+'\r\n')
    
    def plan(self):
        count=0
        while count*self.time_duration < self.total_time:
            desired_pose=self.trajectory.desired_trajectory(count*self.time_duration)
            joint_values=self.sys.compute_inverse_kinematics(desired_pose)
            if self.debug:
                print(joint_values)
            if len(joint_values)>0:
                self.write_info(joint_values)
                count+=1
            else:
                rospy.logerr('no IKsolution found. exit the program')
                exit()



def main():
    DEBUG=True
    rospy.init_node('sys_planner')
    while rospy.get_time()==0:
        continue
    planner=planner_class()
    if DEBUG:
        end_pose=Pose()
        end_pose.position.x=3
        end_pose.position.y=0
        end_pose.position.z=0.3
        end_pose.orientation.x=0.5
        end_pose.orientation.y=0.5
        end_pose.orientation.z=0.5
        end_pose.orientation.w=0.5
        joint_values=planner.sys.compute_inverse_kinematics(end_pose)
        print(joint_values)
    #planner.plan()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():


        rate.sleep()


if __name__ == '__main__':
    main()