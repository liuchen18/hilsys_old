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


        

def main():
    rospy.init_node('fwx_move')
    fwx_control=fwx_control_class()
    time,joint1,joint2,joint3,joint4,joint5,joint6,joint7=[],[],[],[],[],[],[],[]
    x,y,theta=[],[],[]
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split()))
            time.append(copy.deepcopy(data[0]))
            x.append(copy.deepcopy(data[1]))
            y.append(copy.deepcopy(data[2]))
            theta.append(copy.deepcopy(data[3]))
            joint1.append(copy.deepcopy(data[4]))
            joint2.append(copy.deepcopy(data[5]))
            joint3.append(copy.deepcopy(data[6]))
            joint4.append(copy.deepcopy(data[7]))
            joint5.append(copy.deepcopy(data[8]))
            joint6.append(copy.deepcopy(data[9]))
            joint7.append(copy.deepcopy(data[10]))
    rospy.loginfo('got the planned data')
    time_duration=time[2]-time[1]
    while rospy.get_time() == 0:
        continue
    rate=rospy.Rate(1/time_duration)
    temp_count=0
    while temp_count < 8:
        fwx_control.iiwa_msg_pub(joint1[0]/8/time_duration,joint2[0]/8/time_duration,joint3[0]/8/time_duration,joint4[0]/8/time_duration,joint5[0]/8/time_duration,joint6[0]/8/time_duration,joint7[0]/8/time_duration,)
        temp_count+=1
        rate.sleep()
    rospy.sleep(1)
    start_time=rospy.get_time()
    msg_length=len(time)
    time_count=0
    while not rospy.is_shutdown():
        if time_count<msg_length-1:
            joint1_vel=(joint1[time_count+1]-joint1[time_count])/time_duration
            joint2_vel=(joint2[time_count+1]-joint2[time_count])/time_duration
            joint3_vel=(joint3[time_count+1]-joint3[time_count])/time_duration
            joint4_vel=(joint4[time_count+1]-joint4[time_count])/time_duration
            joint5_vel=(joint5[time_count+1]-joint5[time_count])/time_duration
            joint6_vel=(joint6[time_count+1]-joint6[time_count])/time_duration
            joint7_vel=(joint7[time_count+1]-joint7[time_count])/time_duration
            x_v=(x[time_count+1]-x[time_count])/time_duration
            y_v=(y[time_count+1]-y[time_count])/time_duration
            print(y_v)
            theta_v=(theta[time_count+1]-theta[time_count])/time_duration
            fwx_control.iiwa_msg_pub(joint1_vel,joint2_vel,joint3_vel,joint4_vel,joint5_vel,joint6_vel,joint7_vel)
            fwx_control.base_msg_pub(x_v,y_v,theta_v)
        else:
            fwx_control.iiwa_msg_pub(0,0,0,0,0,0,0)
            fwx_control.base_msg_pub(0,0,0)
            rospy.loginfo('the planned trajectory is finished')
            exit()
        time_count+=1
        rate.sleep()


    


if __name__ == '__main__':
    main()

