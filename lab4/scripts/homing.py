#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

flag = 0
distance=[]
target_index=0

class HomingController:
    

    def __init__(self,name):
        rospy.init_node(name, anonymous=True)

        self.p = rospy.get_param("~linear_gain", 0.5)  # 线速度增益
        self.k = rospy.get_param("~angular_gain", 0.5)  # 角速度增益
        self.init_distance = rospy.get_param("~init_distance", 1.2)  # 初始距离（预设50cm)
        self.target_distance = rospy.get_param("~target_distance", 0.15)  # 目标距离 (15 cm)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd = Twist()

    def laser_callback(self,msg):
        global flag,distance,target_index
        distance=[]
        if (flag==0):
            #找到与初始距离相近的方位
            # print(msg.ranges)
            interested = list(msg.ranges[0:45])
            print(interested)
            for num in interested :
                if self.init_distance-0.1 < num < self.init_distance+0.1 :
                    distance.append(num)
                    print(msg.ranges.index(num))
                    
            print(distance)
            target_index = int((msg.ranges.index(distance[0])+msg.ranges.index(distance[len(distance)-1]))/2)
            print(target_index)
            angle_to_pillar = msg.angle_min + target_index * msg.angle_increment
            
            if abs(angle_to_pillar) <= 0.05:
                flag = 1
                self.cmd.angular.z = 0
            else :
                self.cmd.angular.z = self.k*(angle_to_pillar)
        else :
            dist=msg.ranges[target_index] 
            print(f"dist:{dist}")
            if dist > self.target_distance :
                self.cmd.linear.x = self.p*abs(dist-self.target_distance)
            else :
                self.cmd.linear.x = 0
        self.cmd_pub.publish(self.cmd)
        print(f"flag:{flag}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = HomingController('homing_controller')
    controller.run()
