#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import copy
from geometry_msgs.msg import Twist


class fwx_control_class():
    def __init__(self):
        self.joint1_pub=rospy.Publisher('joint1_command',Float64,queue_size=10)
        self.joint2_pub=rospy.Publisher('joint2_command',Float64,queue_size=10)
        self.joint3_pub=rospy.Publisher('joint3_command',Float64,queue_size=10)
        self.joint4_pub=rospy.Publisher('joint4_command',Float64,queue_size=10)
        self.joint5_pub=rospy.Publisher('joint5_command',Float64,queue_size=10)
        self.joint6_pub=rospy.Publisher('joint6_command',Float64,queue_size=10)
        self.joint7_pub=rospy.Publisher('joint7_command',Float64,queue_size=10)
        self.base_vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.time,self.joint1,self.joint2,self.joint3,self.joint4,self.joint5,self.joint6,self.joint7=[],[],[],[],[],[],[],[]
        self.x,self.y,self.theta=[],[],[]
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory.txt','r') as f:
            for line in f.readlines()[1:]:
                data=list(map(float,line.split()))
                self.time.append(copy.deepcopy(data[0]))
                self.x.append(copy.deepcopy(data[1]))
                self.y.append(copy.deepcopy(data[2]))
                self.theta.append(copy.deepcopy(data[3]))
                self.joint1.append(copy.deepcopy(data[4]))
                self.joint2.append(copy.deepcopy(data[5]))
                self.joint3.append(copy.deepcopy(data[6]))
                self.joint4.append(copy.deepcopy(data[7]))
                self.joint5.append(copy.deepcopy(data[8]))
                self.joint6.append(copy.deepcopy(data[9]))
                self.joint7.append(copy.deepcopy(data[10]))
        rospy.loginfo('got the planned data')
        self.time_duration=self.time[2]-self.time[1]
        self.rate=rospy.Rate(1/self.time_duration)
        
    def iiwa_msg_pub(self,joint1_v,joint2_v,joint3_v,joint4_v,joint5_v,joint6_v,joint7_v):
        '''
        publish the joint velocity
        '''
        j1_vel,j2_vel,j3_vel,j4_vel,j5_vel,j6_vel,j7_vel=Float64(),Float64(),Float64(),Float64(),Float64(),Float64(),Float64()
        j1_vel.data=joint1_v
        j2_vel.data=joint2_v
        j3_vel.data=joint3_v
        j4_vel.data=joint4_v
        j5_vel.data=joint5_v
        j6_vel.data=joint6_v
        j7_vel.data=joint7_v
        #print(str(joint1_v)+'  '+str(joint2_v)+'  '+str(joint3_v)+'  '+str(joint4_v)+'  '+str(joint5_v)+'  '+str(joint6_v)+'  '+str(joint7_v))
        self.joint1_pub.publish(j1_vel)
        self.joint2_pub.publish(j2_vel)
        self.joint3_pub.publish(j3_vel)
        self.joint4_pub.publish(j4_vel)
        self.joint5_pub.publish(j5_vel)
        self.joint6_pub.publish(j6_vel)
        self.joint7_pub.publish(j7_vel)
    def base_msg_pub(self,x,y,theta):
        '''
        publish the base twist
        '''
        msg=Twist()
        msg.linear.x=x
        msg.linear.y=y
        msg.angular.z=theta
        self.base_vel_pub.publish(msg)

    def iiwa_init(self):
        temp_count=0
        while temp_count < 8:
            self.iiwa_msg_pub(self.joint1[0]/8/self.time_duration,self.joint2[0]/8/self.time_duration,self.joint3[0]/8/self.time_duration,self.joint4[0]/8/self.time_duration,self.joint5[0]/8/self.time_duration,self.joint6[0]/8/self.time_duration,self.joint7[0]/8/self.time_duration,)
            temp_count+=1
            self.rate.sleep()
    
    def compute_velocity(self,time_count):
        joint1_vel=(self.joint1[time_count+1]-self.joint1[time_count])/self.time_duration
        joint2_vel=(self.joint2[time_count+1]-self.joint2[time_count])/self.time_duration
        joint3_vel=(self.joint3[time_count+1]-self.joint3[time_count])/self.time_duration
        joint4_vel=(self.joint4[time_count+1]-self.joint4[time_count])/self.time_duration
        joint5_vel=(self.joint5[time_count+1]-self.joint5[time_count])/self.time_duration
        joint6_vel=(self.joint6[time_count+1]-self.joint6[time_count])/self.time_duration
        joint7_vel=(self.joint7[time_count+1]-self.joint7[time_count])/self.time_duration
        x_v=(self.x[time_count+1]-self.x[time_count])/self.time_duration
        y_v=(self.y[time_count+1]-self.y[time_count])/self.time_duration
        print(y_v)
        theta_v=(self.theta[time_count+1]-self.theta[time_count])/self.time_duration
        return joint1_vel,joint2_vel,joint3_vel,joint4_vel,joint5_vel,joint6_vel,joint7_vel,x_v,y_v,theta_v


        

def main():
    rospy.init_node('fwx_move')
    fwx_control=fwx_control_class()
    while rospy.get_time() == 0:
        continue
    fwx_control.iiwa_init()
    start_time=rospy.get_time()
    msg_length=len(fwx_control.time)
    time_count=0
    while not rospy.is_shutdown():
        if time_count<msg_length-1:
            joint1_vel,joint2_vel,joint3_vel,joint4_vel,joint5_vel,joint6_vel,joint7_vel,x_v,y_v,theta_v=fwx_control.compute_velocity(time_count)
            fwx_control.iiwa_msg_pub(joint1_vel,joint2_vel,joint3_vel,joint4_vel,joint5_vel,joint6_vel,joint7_vel)
            fwx_control.base_msg_pub(x_v,y_v,theta_v)
        else:
            fwx_control.iiwa_msg_pub(0,0,0,0,0,0,0)
            fwx_control.base_msg_pub(0,0,0)
            rospy.loginfo('the planned trajectory is finished')
            exit()
        time_count+=1
        fwx_control.rate.sleep()


    


if __name__ == '__main__':
    main()

