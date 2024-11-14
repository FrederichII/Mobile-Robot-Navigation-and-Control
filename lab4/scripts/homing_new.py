#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import numpy as np
from sys import exit
from time import sleep

flag = 0
state = 0
init_distance = 0
curr_dist = 0
curr_angle = 0
init_angle = 0
last_time = None
velocity = 0

def find_target(arr):
        init_distance = 1000
        # print(arr)
        arr_1=[]
        for e in arr:
            if e != 0:
                arr_1.append(e)
        # print(arr_1)
        #计算数组相邻元素差值的绝对值
        diff = np.abs(np.diff(arr_1))
        # 设置阈值，用来识别突变
        threshold = np.percentile(diff, 98)
        # print(diff)
        #print(threshold)
        mutation_indices = np.where(diff > threshold)[0]
        #print(mutation_indices)
        for i in range(len(mutation_indices)-1):
            if mutation_indices[i+1]-mutation_indices[i] == 3:#找到连续的三个突变点
                target_index=int((mutation_indices[i]+mutation_indices[i+1])/2)
                print(target_index)
                init_distance=arr_1[target_index]
                target_index=arr.index(init_distance)
                print(target_index)
                if init_distance<1.5:
                    break
        #print(f"target_index:{target_index}")
        return target_index,init_distance

def find_target2(arr):
        init_distance = 1.5
        arr = arr[-20:]+arr[:20]
        # print(arr)
        arr_1=[]
        for e in arr:
            if e != 0:
                arr_1.append(e)
        # print(arr_1)
        #计算数组相邻元素差值的绝对值
        diff = np.abs(np.diff(arr_1))
        # 设置阈值，用来识别突变
        threshold = np.percentile(diff, 95)
        # print(diff)
        #print(threshold)
        mutation_indices = np.where(diff > threshold)[0]
        print(len(mutation_indices))
        #print(mutation_indices)
        # target_index=int(np.mean(mutation_indices))
        x = int(3 / init_distance) + 3
        print(x)
        for i in range(len(mutation_indices)-1):
            print(mutation_indices[i+1]-mutation_indices[i])
            if mutation_indices[i+1]-mutation_indices[i] > x and  mutation_indices[i+1]-mutation_indices[i] < 90:#找到连续的三个突变点
                target_index=int((mutation_indices[i]+mutation_indices[i+1])/2)
                print(target_index)
                init_distance=arr_1[target_index]
                target_index=arr.index(init_distance)
                print(target_index)
                if init_distance<1.5:
                    break
        
        #print(f"target_index:{target_index}")
        return target_index,init_distance

class HomingController:
    def __init__(self,name):
        rospy.init_node(name, anonymous=True)

        self.p = rospy.get_param("~linear_gain", 0.8)  # 线速度增益
        self.k = rospy.get_param("~angular_gain", 2)  # 角速度增益
        #self.init_distance = rospy.get_param("~init_distance", 1.2)  # 初始距离（预设50cm)
        self.target_distance = rospy.get_param("~target_distance", 0.15)  # 目标距离 (15 cm)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        sleep(1)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.cmd = Twist()
        self.stop = 0

    def laser_callback(self, msg):
        global init_distance,init_angle,curr_angle,curr_dist,flag
        # print(type(msg.ranges))
        target_index,init_distance = find_target(list(msg.ranges)) #find index of target by diff
        curr_dist = init_distance
        init_angle = msg.angle_min + target_index * msg.angle_increment # index to angle
        print(f"init_angle:{init_angle}")
        if init_angle>np.pi :
            init_angle = init_angle - 2*np.pi
        curr_angle = init_angle
        if (curr_dist > self.target_distance):
            flag = 0
        self.laser_sub.unregister() # not subscribe any more

    def laser_callback2(self, msg):
        if self.stop:
            return 0
        global init_distance,init_angle,curr_angle,curr_dist,flag
        # print(type(msg.ranges))
        target_index,init_distance = find_target2(list(msg.ranges)) #find index of target by diff
        curr_dist = init_distance
        print(curr_dist)
        init_angle = msg.angle_min + target_index * msg.angle_increment # index to angle
        
        if curr_dist > self.target_distance :
                # print(self.p,"this is p")
                self.cmd.linear.x = self.p*abs(curr_dist-self.target_distance)
        else:
            print("stop")
            self.cmd.linear.x = 0
            self.stop = 1
            
        self.cmd_pub.publish(self.cmd)
        # self.laser_sub.unregister() # not subscribe any more

    def imu_callback(self, msg):
        global flag,init_distance,init_angle,last_time,curr_angle,curr_dist,velocity
        current_time = rospy.Time.now()
        # print(f"curr_time,last_time{current_time},{last_time}")
        if (flag==0): # target-finding state
            print(f"flag={flag}")
            
            if last_time is not None:
                time_diff = (current_time - last_time).to_sec()
                # print(msg.angular_velocity)
                curr_angle = curr_angle - msg.angular_velocity.z * time_diff
            print(f"curr_ang:{curr_angle}")
            if abs(curr_angle) <= 0.02: # rotated to the correct direction
                flag = 1
                self.cmd.angular.z = 0
            else :
                print("rotating")
                self.cmd.angular.z = self.k*(curr_angle)
            
                        
        elif flag == 1 : # forwarding state
            print(f"flag:{flag}")
            print(f"init_dist{init_distance}")
            print(f"curr_dist:{curr_dist}")
            self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback2)
            # if last_time is not None:
            #     time_diff = (current_time - last_time).to_sec()
            #     print(f"acc{msg.linear_acceleration.x}")
            #     velocity += (msg.linear_acceleration.x-0.05)* time_diff
            #     print(f"vel:{velocity}")
            #     curr_dist = curr_dist - velocity*time_diff 
            
                
        #     else :
        #         self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback2)
                
        #         # print(f"curr_dist_stop{curr_dist}")
               
                
        #         flag = 2
        # else:
        #     pass

        #print(self.cmd)
        self.cmd_pub.publish(self.cmd)
        last_time = current_time
    

    def run(self):
        rospy.spin()
        

if __name__ == '__main__':
    controller = HomingController('homing_controller')
    controller.run()
