#!/usr/bin/env python3
import numpy as np 
import math    
import rospy 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray,Int32
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import Odometry
import tf.transformations as tf 
from tf.transformations import*
from define import *
 
 
class MainIntervention:    
    def __init__(self): 
  
        self.wheel_base_distance = 0.23 
        self.wheel_radius =  0.035  
        self.dt=0.0
        
        ######  Initialize the robot state #####
        self.state = [0.0, 0.0, 0.0]  
       
        ######## Joint Limits ########
        self.max = 0.5
        self.max_a = self.max + 0.01 
        self.min = -0.5  
        self.min_a = self.min - 0.01 
        self.goal = None 
        self.tasks=[]
        
        ####### Publishers ########
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)   
        self.goal_check = rospy.Publisher('/goal_check', PoseStamped, queue_size=10)  
        self.joint_velocity= rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10) 
        self.wheel_velocity= rospy.Publisher("/turtlebot/kobuki/commands/wheel_velocities", Float64MultiArray, queue_size=10)     
        self.J_wheel_velocity= rospy.Publisher("/velocities", JointState, queue_size=10)      

        ####### Subscribers #######
        self.weight_sub = rospy.Subscriber('/weight_set', Float64MultiArray,  self.weight_service)  
        self.goal_sub = rospy.Subscriber('/goal_set', Float64MultiArray,  self.goal_service)  
        self.task_sub = rospy.Subscriber('/task_set', Int32,  self.task_service)   
        self.joints_sub = rospy.Subscriber('/turtlebot/joint_states', JointState, self.JointState_callback)   
        self.odom_sub = rospy.Subscriber("kobuki/odom", Odometry, self.odom_callback)   
        
        
        
    def task_service(self, task_index):
        """
        Callback function for setting the active tasks based on the provided index.

        Parameters:
        - task_index (Int32): A message containing an integer value representing the indices of tasks to activate.

        Functionality:
        - Updates the active task index.
        - Selects the tasks from the tasks list based on the provided indices.
        """
        # Update the active task index
        self.selected_task= task_index.data
        
     
        
        # Select the tasks based on the provided indices
        tasks = [self.tasks[i] for i in self.selected_task]
        
        # Update the tasks list with the selected tasks
        self.tasks = tasks

                       
                             
    def goal_service(self, goal_msg):
        """
        Callback function for setting the desired goal positions.

        Parameters:
        - goal_msg (Float64MultiArray): A message containing the goal positions.

        Functionality:
        - Sets the desired goal positions.
        - Publishes the goal pose.
        - Defines the task hierarchy.
        """
        # Set the desired goal positions
        self.goal = [[goal_msg.data[0], goal_msg.data[1], goal_msg.data[2]], [goal_msg.data[3]]]
        # Define the goal pose
        self.goal_pose = goal_pose(np.array(self.goal[0]), np.array(self.goal[1]))
        
        # Publish the goal pose for verification
        self.goal_check.publish(self.goal_pose)

        # Define the task hierarchy
        
        self.tasks = [   
            
                    Jointlimits3D("First Joint", np.array([0.0]), np.array([self.max, self.max_a, self.min, self.min_a]),1),  
                    Jointlimits3D("Second Joint", np.array([0.0]), np.array([self.max, self.max_a, self.min, self.min_a]),2),
                    # Jointlimits3D("Third Joint", np.array([0.01]), np.array([self.max, self.max_a, self.min, self.min_a]),3),  
                    # Jointlimits3D("Fourth Joint", np.array([0.01]), np.array([self.max, self.max_a, self.min, self.min_a]),4),                              
                    Position3D("End-Effector Position", np.array(self.goal[0]).reshape(3,1), 6),  
                    # Orientation3D("end effector orientation", np.array(self.goal[1]), 6),
                    # Configuration3D("Configuration", np.array([(self.goal[0],self.goal[1])]).reshape(4, 1), 6) 
                    # BaseOrientation3D("Base orientation", np.array(self.goal[1]), 1)
                    
                                        
                    ]   

 
    def weight_service(self, weight_msg):
        """
        Callback function for setting the weights for the DLS solution.

        Parameters:
        - weight_msg (Float64MultiArray): A message containing the weights.

        Functionality:
        - Updates the weights used in the DLS solution.
        """
        # Update the weights used in the DLS solution
        self.weight = weight_msg.data

    def odom_callback(self, odom_msg): 
        
        """"
        Callback function for updating the robot's state based on odometry data.

        Parameters:
        - odom_msg (Odometry): A message containing the odometry data, including position and orientation.

        Functionality:
        - Converts quaternion orientation to Euler angles.
        - Updates the robot's state with the current position and orientation.
        """

        # Extract the quaternion from the odometry message
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
        
        # Convert the quaternion to Euler angles
        _,_,yaw= euler_from_quaternion(quaternion)
        
        # Update the robot's state with the current position and orientation (yaw angle)
        self.state = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            yaw
        ])     

  
    def send_velocity(self, q):
        """
        Publishes the computed joint and wheel velocities.

        Parameters:
        - q (numpy array): A 6x1 column vector containing:
                        - Angular velocity (index 0)
                        - Linear velocity (index 1)
                        - Joint velocities for four joints (indices 2, 3, 4, 5)
        
        Functionality:
        - Extracts and publishes the joint velocities.
        - Converts linear and angular velocities to left and right wheel velocities.
        - Publishes wheel velocities.
        """
        
        
        """" Publishing Joint Velocity """
        joint_vel_mesg = Float64MultiArray()
        # Extract the joint velocities  and convert them to float
        joint_vel_mesg.data = [float(q[2, 0]), float(q[3, 0]), float(q[4, 0]), float(q[5, 0])]
        
        # Publish the joint velocities to the 'joint_velocity' topic
        self.joint_velocity.publish(joint_vel_mesg)

        # Converting linear and angular velocities to left and right wheel velocities
        w = q[0, 0]  # Angular velocity
        v = q[1, 0]  # Linear velocity
        
        # Calculate right wheel velocity
        v_r = (2 * v + w * self.wheel_base_distance) / (2 * self.wheel_radius)
        
        # Calculate left wheel velocity
        v_l = (2 * v - w * self.wheel_base_distance) / (2 * self.wheel_radius)

        """" Publishing Wheel Velocity """
        wheel_vel_msg= Float64MultiArray()
        # Convert wheel velocities to float and store in the message
        wheel_vel_msg.data = [float(v_r), float(v_l)]
        
        # Publish the wheel velocities to the 'wheel_velocity' topic
        self.wheel_velocity.publish(wheel_vel_msg)

        """ Publishing for odometry """
        joint_state_msg = JointState()
        # Set the timestamp of the message to the current time
        joint_state_msg.header.stamp = rospy.Time.now()
        # Set the wheel velocities in the 'velocity' field of the JointState message
        joint_state_msg.velocity = [float(v_r), float(v_l)]
        # Publish the wheel velocities to the 'J_wheel_velocity' topic
        self.J_wheel_velocity.publish(joint_state_msg)

    def JointState_callback(self,msg):
        """
        Callback function for processing incoming joint state data.
        
        Parameters:
        - data (JointState): The current state of the robot's joints, including their names, positions, velocities, and efforts.
        
        Functionality:
        - Checks if a goal is set.
        - Verifies that the received joint names match the expected joint names.
        - Updates the manipulator model with the current joint positions.
        - Calls the method to update the robot state and handle task execution.
        """
        
        # Check if a goal is set
        if self.goal is not None:
            # List of joint names
            names = ['turtlebot/swiftpro/joint1', 'turtlebot/swiftpro/joint2', 'turtlebot/swiftpro/joint3', 'turtlebot/swiftpro/joint4']
            
            # Check if the received data corresponds to the joint names of the manipulator
            if msg.name == names:
                # Store the joint positions as a column vector
                self.theta = np.array(msg.position, ndmin=2).T
                # Initialize the manipulator model with the joint positions
                self.robot = Manipulator(self.theta)
                # Call the method to update the robot state and tasks
                self.update_robot_and_tasks()
                
                

    def update_robot_and_tasks(self):
        
        """
        Updates the robot model and executes tasks based on the current state and goals.
        
        Functionality:
        - Updates the robot model with the current joint velocities and state.
        - Iterates through each task, updating and checking if the task is active.
        - Computes task errors and the Jacobian matrix for each active task.
        - Uses weighted damped least-squares to solve for joint velocities that minimize task errors.
        - Applies velocity limits to ensure joint velocities are within safe bounds.
        - Publishes the computed joint velocities and end-effector pose.
        """
        
        # Time step for integration
        dt = self.dt
        # Identity matrix for the number of degrees of freedom of the robot
        P = np.eye(self.robot.getDOF())
        # Initialize the joint velocity vector
        dq = np.zeros((self.robot.getDOF(), 1))
        # Update the robot model with the current joint velocities and state
        self.robot.update(dq, dt, self.state)
        
        
        """ Recursive TP Formulation Algorithm  """
        # Iterate over the tasks
        for i in range(len(self.tasks)):
            # Update the task with the current robot state
            self.tasks[i].update(self.robot)
            
            # Check if the task is active
            if self.tasks[i].bool_is_Active():
                # Get the task error
                err = self.tasks[i].getError()
                # Get the task Jacobian
                J = self.tasks[i].getJacobian()
                # Compute the augmented Jacobian
                J_bar = J @ P
                # Compute the weighted damped least-squares solution for the Jacobian
                J_DLS = W_DLS(J_bar, 0.1, self.weight)
                # Compute the pseudo-inverse of the augmented Jacobian
                J_pinv = np.linalg.pinv(J_bar)
                
                # Compute the task velocity
                dq = dq + J_DLS @ (err - J @ dq)
                
                # Update the null-space projector
                P = P - (J_pinv @ J_bar)
                
                # Joint velocity limits
                vmax= 0.3
                vmin= -0.3
                # Enforce the velocity limits on the computed joint velocities
                for q in range(len(dq)):
                    if dq[q] < vmin:
                        dq = scale(dq, vmin, q)
                    if dq[q] > vmax:
                        dq = scale(dq, vmax, q)
                dq = dq.reshape(6, 1)
        
        # Send the computed joint velocities
        self.send_velocity(dq)
        # Publish the end-effector pose
        pose_ee = pose_EE(self.robot.getEETransform())
        self.pose_EE_pub.publish(pose_ee)
        
        # Update the robot model with the new joint velocities and state
        self.robot.update(dq, dt, self.state)
        
 
if __name__ == '__main__': 
    try:
        rospy.init_node('main_node', anonymous=True)  
        MainIntervention()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass