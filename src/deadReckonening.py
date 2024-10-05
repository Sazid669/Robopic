#!/usr/bin/python
 
import numpy as np
import math 
import rospy  
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix 
from tf.broadcaster import TransformBroadcaster 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymap3d import geodetic2ned
from visualization_msgs.msg import Marker, MarkerArray 
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray 

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped   
    
class DeadReckonening:
    def __init__(self) -> None: 

        # robot constants 
        self.wheel_radius = 0.035 # meters      
        self.wheel_base_distance = 0.23 # meters    
 
        # initial pose of the robot  
        self.Xk = np.zeros([3,1])          
        
        # velocity and angular velocity of the robot
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.dt = 0
        self.goal = 0

        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_wheel_received = False

        # self.last_time = rospy.Time.now()
        self.gb = 0

        # covariance details
        self.Pk = np.array([[0.04, 0, 0],    # 2,2 matrix  
                            [0, 0.04, 0], 
                            [0, 0, 0.04]])       # 3,3 matrix with diagonal 0.04            
        
        self.Qk = np.array([[0.2**2, 0],    # 2,2 matrix   
                             [0, 0.2**2]])     

        self.Rk = np.array([[2.0, 0],    # 2,2 matrix   
                             [0, 2.0]])    

  
        # joint state subscriber  
        self.js_sub = rospy.Subscriber("/velocities", JointState, self.joint_state_callback_new)   
        self.goal_sub = rospy.Subscriber('/goal_set', Float64MultiArray,  self.goals_set)   
        
        # odom publisher 
        self.odom_pub = rospy.Publisher("kobuki/odom", Odometry, queue_size=10)
        self.tf_br = TransformBroadcaster()
 
    def goals_set(self, new):
        # print('goal received')
        self.goal = 1

    def joint_state_callback_new(self, msg): 
        # if self.goal == 1:
            if self.gb == 0:
                self.last_time = rospy.Time.now() 
                self.gb = 1  
             
            self.right_wheel_velocity = msg.velocity[0]        
            self.left_wheel_velocity = msg.velocity[1]   


            # print('entered')  
            left_lin_vel = self.left_wheel_velocity * self.wheel_radius 
            right_lin_vel = self.right_wheel_velocity * self.wheel_radius


            self.v =  (left_lin_vel + right_lin_vel) / 2.0   
            self.w = (right_lin_vel - left_lin_vel) / self.wheel_base_distance   
            
            #calculate dt 
            self.current_time = rospy.Time.from_sec(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9) 
            self.dt = (self.current_time - self.last_time).to_sec()
            # print(self.dt)  
            self.last_time = self.current_time  
            # self.dt = 0.005   

            self.Xk[0,0] = self.Xk[0,0] + np.cos(self.Xk[2,0]) * self.v * self.dt 
            self.Xk[1,0] = self.Xk[1,0] + np.sin(self.Xk[2,0]) * self.v * self.dt   
            self.Xk[2,0] = self.Xk[2,0] + self.w * self.dt          
            # print('position', self.Xk)    
                        
            Ak = np.array([[1, 0, -np.sin(self.Xk[2,0])*self.v*self.dt], 
                            [0, 1, np.cos(self.Xk[2,0])*self.v*self.dt],
                            [0, 0, 1]])    

            Bk = np.array([[np.cos(self.Xk[2,0])*self.dt*0.5, np.cos(self.Xk[2,0])*self.dt*0.5],    
                                [np.sin(self.Xk[2,0])*self.dt*0.5, np.sin(self.Xk[2,0])*self.dt*0.5],  
                                [(self.dt*self.wheel_radius)/self.wheel_base_distance, -(self.dt*self.wheel_radius)/self.wheel_base_distance]])        
                    
            self.Pk = np.dot(np.dot(Ak, self.Pk),Ak.T) + np.dot(np.dot(Bk, self.Qk),Bk.T)
            self.publish_odom(self.Xk, self.Pk, self.current_time, self.v, self.w)  

    
    def publish_odom(self, Xk, Pk, current_time, v, w): 
                self.Xk = Xk
                self.Pk = Pk  
                self.v = v
                self.w = w
                self.current_time = current_time 
                # print('yahoo')   
                q = quaternion_from_euler(0, 0, self.Xk[2,0]) 

                odom = Odometry()
                odom.header.stamp = self.current_time   
                odom.header.frame_id = "world_ned"    
                odom.child_frame_id = "turtlebot/kobuki/base_footprint"    
                odom.twist.twist.linear.x = self.dt 

                odom.pose.pose.position.x =  self.Xk[0,0] 
                odom.pose.pose.position.y =  self.Xk[1,0]
                odom.pose.covariance = [self.Pk[0,0], self.Pk[0,1], 0.0, 0.0, 0.0, self.Pk[0,2],    
                                        self.Pk[1,0], self.Pk[1,1], 0.0, 0.0, 0.0, self.Pk[1,2],   
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        self.Pk[2,0], self.Pk[2,1], 0.0, 0.0, 0.0, self.Pk[2,2]]             

                odom.pose.pose.orientation.x = q[0] 
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2] 
                odom.pose.pose.orientation.w = q[3]    

                odom.twist.twist.linear.x = self.v 
                odom.twist.twist.angular.z = self.w
                
                self.odom_pub.publish(odom)    

                self.tf_br.sendTransform((self.Xk[0,0], self.Xk[1,0], 0.0), q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)    
 
 
if __name__ == '__main__':  

    rospy.init_node("differential_drive") 
    robot = DeadReckonening() 
    rospy.spin() 




    





