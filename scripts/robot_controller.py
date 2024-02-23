#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class PIDController:
    def __init__(self):
        # self.kp = 4
        self.kp = 2
        self.ki = 0.0
        # self.kd = 1550
        self.kd = 450
        self.prev_error = 0
        self.integral = 0
        self.desired_track_error = 0.05

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output


class Robot_controller:
    def __init__(self):
        rospy.init_node('Lidar_controller', anonymous=True)
        self.vel_pub_l_f = rospy.Publisher('/285610_robot/joint_l_f_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub_l_b = rospy.Publisher('/285610_robot/joint_l_b_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub_r_f = rospy.Publisher('/285610_robot/joint_r_f_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub_r_b = rospy.Publisher('/285610_robot/joint_r_b_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub_turret = rospy.Publisher('/285610_robot/joint_turret_position_controller/command', Float64, queue_size=10)
        self.vel_pub_forearm = rospy.Publisher('/285610_robot/joint_forearm_position_controller/command', Float64, queue_size=10)
        self.vel_pub_prism = rospy.Publisher('/285610_robot/joint_prism_position_controller/command', Float64, queue_size=10)
        self.lidar_l_sub = rospy.Subscriber('/285610_robot/laser_l/scan', LaserScan, self.lidar_l_callback)
        self.lidar_r_sub = rospy.Subscriber('/285610_robot/laser_r/scan', LaserScan, self.lidar_r_callback)
        self.rate = rospy.Rate(40)     
        self.left_distance = 0.0 
        self.right_distance = 0.0 
        self.proportional = 2
        self.velocity = 47
        self.controller = PIDController()
        
    def control(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)

            pos_msg = Float64()
            pos_msg.data = -10
            robot_controller.vel_pub_prism.publish(pos_msg)
            pos_msg.data = -10
            robot_controller.vel_pub_forearm.publish(pos_msg)
            pos_msg.data = 0
            robot_controller.vel_pub_turret.publish(pos_msg)
            rospy.spin()
            self.rate.sleep()
    
    def lidar_l_callback(self, data):
        #rospy.loginfo('L: '+ str(min(data.ranges)))

        if isinstance(min(data.ranges), float):
            self.update_control(True, min(data.ranges))

    def lidar_r_callback(self, data):
        #rospy.loginfo('R: '+ str(min(data.ranges)))

        if isinstance(min(data.ranges), float):
            self.update_control(False, min(data.ranges))

   
    def update_control(self, isLeft, distance):
        if isLeft:
            self.left_distance = distance
        else:
            self.right_distance = distance
        self.move(self.left_distance-self.right_distance)

    def stop(self):
        stop_msg = Float64()
        stop_msg.data = 0
        self.vel_pub_l_f.publish(stop_msg)
        self.vel_pub_r_f.publish(stop_msg)     
        self.vel_pub_l_b.publish(stop_msg)
        self.vel_pub_r_b.publish(stop_msg) 
        # self.push()


    # def push(self):
    #     pos_msg = Float64()
    #     pos_msg.data = 0
    #     robot_controller.vel_pub_forearm.publish(pos_msg)

    #     rospy.sleep(1)
    #     pos_msg.data = 100
    #     robot_controller.vel_pub_prism.publish(pos_msg)
   
    def move(self, diff):
        move_l_msg = Float64()
        move_r_msg = Float64()

        move_l_msg.data = self.velocity + self.controller.compute(diff)
        move_r_msg.data = self.velocity - self.controller.compute(diff)
    
        boost = max(move_r_msg.data,move_l_msg.data)
        move_l_msg.data += 50-boost
        move_r_msg.data += 50-boost

        self.vel_pub_l_f.publish(move_l_msg)
        self.vel_pub_r_f.publish(move_r_msg)  
        self.vel_pub_l_b.publish(move_l_msg)
        self.vel_pub_r_b.publish(move_r_msg) 
        if (move_l_msg.data>50.0)or(move_r_msg.data>50.0):
            rospy.logerr("!!!ERROR!!!")
        rospy.loginfo('R: '+ str(move_r_msg.data) + ' | L: '+ str(move_l_msg.data))  
    

    
if __name__ == '__main__':
    robot_controller = Robot_controller()

    robot_controller.control()



        
