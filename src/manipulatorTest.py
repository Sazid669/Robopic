#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

class Manipulator():
    
    def __init__(self, theta, theta2, theta3, theta4):
        # Initialize the class with joint angles
        self.theta = theta
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.q = [[theta], [theta2], [theta3], [theta4]]
        # Subscriber to joint states
        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.JointState_callback)
        # Publisher for joint velocities
        self.joint_velocity_pub = rospy.Publisher("/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)
        self.goal = rospy.Publisher('goal', PoseStamped, queue_size=10)
        # rospy.Subscriber("desired_pose", PoseStamped, self.get_pose)
    def JointState_callback(self, data):
        # Callback function for joint states
        if data.name == ['swiftpro/joint1', 'swiftpro/joint2', 'swiftpro/joint3', 'swiftpro/joint4']:
            # Update joint angles
            self.theta, self.theta2, self.theta3, self.theta4 = data.position 
            # Construct joint positions
            self.q = [[self.theta], [self.theta2], [self.theta3], [self.theta4]]
            # Update kinematics and run control
            # print(self.q)
            self.pose_EE()
            self.run()

    def pose_EE(self):
        r = R.from_matrix(self.Kinematics()[:3,:3])
        q = R.as_quat(r)

        p = PoseStamped()
        p.header.frame_id = "swiftpro/manipulator_base_link"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x=self.Kinematics()[0,3]
        p.pose.position.y=self.Kinematics()[1,3]
        p.pose.position.z=self.Kinematics()[2,3]

        p.pose.orientation.x=q[0]
        p.pose.orientation.y=q[1]
        p.pose.orientation.z=q[2]
        p.pose.orientation.w=q[3]

        self.pose_EE_pub.publish(p)

    def Kinematics(self):
        a1 =  0.0132             
        a2 = 0.1588             
        a3 = 0.056                           
        d2 = 0.142             
        a13 = a1 +a3
        self.Hm_ee = np.array([ 
                            [np.cos(self.theta + self.theta4),  -np.sin(self.theta + self.theta4), 0, (a13 - d2 * np.sin(self.theta2)  + a2 * np.cos(self.theta3)) * np.cos(self.theta)],
                            [np.sin(self.theta + self.theta4), np.cos(self.theta + self.theta4), 0, (a13 - d2 * np.sin(self.theta2)  + a2 * np.cos(self.theta3)) * np.sin(self.theta)],
                            [0 , 0, 1, -0.0358 -  d2 * np.cos(self.theta2) - a2 * np.sin(self.theta3)],
                            [ 0    , 0     ,   0   , 1 ]])
        return self.Hm_ee
        # return self.Hm_ee
        
    def Jacobian(self):
        a1 =  0.0132             
        a2 = 0.1588             
        a3 = 0.056              
        d1 = 0.108              
        d2 = 0.142             
        d3 = 0.0722    
        a13 = a1 +a3
        self.J = np.array([ 
                            [-np.sin(self.theta) * (a13 - d2 * np.sin(self.theta2) + a2 * np.cos(self.theta3)) , -d2 * np.cos(self.theta) * np.cos(self.theta2)  ,a2 * np.sin(self.theta3) * np.cos(self.theta)  ,0],
                            [np.cos(self.theta) * (a13 - d2 * np.sin(self.theta2) + a2 * np.cos(self.theta3)) , -d2 * np.sin(self.theta) * np.cos(self.theta2)  ,-a2 * np.sin(self.theta3) * np.sin(self.theta)  ,0],
                            [0 , d2 * np.sin(self.theta2)  ,-a2 * np.cos(self.theta3)  ,0],
                            [ 0    , 0     ,   0   , 0 ],
                            [ 0    , 0     ,   0   , 0 ],
                            [1    , 0     ,   0   , 1 ]])
        return self.J          

        
    
    def DLS(self, A, damping):
        # Damped Least Squares method for computing inverse of a matrix
        x = A @ np.transpose(A)
        DLS = np.transpose(A) @ np.linalg.inv(x + damping**2 * np.identity(x.shape[0]))
        return  DLS
    
    # def update(self, q):
    #     # Update joint positions
    #     self.q = q
    #     # Update kinematics
    #     self.T = self.Kinematics()
    #     self.J = self.Jacobian()
    # def get_pose(self,position):
    #     # print(position.pose.position.x, position.pose.position.y, position.pose.position.z)
    #     self.goal= np.array([position.pose.position.x, position.pose.position.y, position.pose.position.z]).reshape(3,1)
    #     self.run(self.goal) 
    
    def send_velocity(self, velocities):
        # Publish joint velocities
        p = Float64MultiArray()
        p.data = [float(velocities[0]), float(velocities[1]), float(velocities[2]), float(velocities[3])]
        self.joint_velocity_pub.publish(p)
    
    def run(self):
        # Run control loop
        T = self.Kinematics() 
        J = self.Jacobian() 
        j= J[:3,:] # Extract position Jacobian
        sigma = T[:3,3].reshape(3,1) # Extract position
        # goal = [0.15, 0.15, -0.15]
        goal = [0.11, 0.11, -0.03]
        goals = np.array(goal).reshape(3,1)
        err = goals - sigma # Compute error
        # print("Error", err)
        dq = self.DLS(j, 0.1) @ err # Compute joint velocity
        max_vel = 0.1
        s = np.max(dq/max_vel)
        if s > 1:
            dq = dq/s # Scale joint velocities if required
        dt = 1.0/60.0
        self.q += dt * dq
        # print("after running", self.q)
        self.send_velocity( dq) # Publish joint velocities

        pose_stamped = PoseStamped()
        # Set the reference frame for the marker
        pose_stamped.header.frame_id = "swiftpro/manipulator_base_link"
        pose_stamped.pose.position.x = goal[0]
        pose_stamped.pose.position.y = goal[1]
        pose_stamped.pose.position.z = goal[2]
        self.goal.publish(pose_stamped)
        


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('kinematics_node', anonymous=True)
        intervention = Manipulator(0, 0, 0, 0)   
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
