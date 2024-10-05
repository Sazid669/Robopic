#!/usr/bin/python3

import py_trees  # Import the py_trees library for behavior trees
import rospy  # Import the rospy library for ROS (Robot Operating System) interactions
from geometry_msgs.msg import PoseStamped  # Import the PoseStamped message type from geometry_msgs
import py_trees.decorators  # Import decorators from py_trees
import py_trees.display  # Import display functions from py_trees
import time  # Import the time library for time-related functions
from nav_msgs.msg import Odometry  # Import the Odometry message type from nav_msgs
from std_msgs.msg import Float64MultiArray  # Import the Float64MultiArray message type from std_msgs
import math  # Import the math library for mathematical functions
from hands_on_intervention.srv import intervention  # Import the intervention service from hands_on_intervention

from tf.transformations import euler_from_quaternion  # Import the euler_from_quaternion function from tf.transformations
import numpy as np  # Import the numpy library for numerical operations
from std_srvs.srv import SetBool  # Import the SetBool service from std_srvs
import signal  # Import the signal library for signal handling

# Class to move the robot to a pick location
class MoveRobotToPick(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(MoveRobotToPick, self).__init__(name)
        # Subscribe to the pose of the end-effector
        self.sub_pose_ee = rospy.Subscriber('pose_EE', PoseStamped, self.ee_pose_callback)
        # Subscribe to the position of Aruco markers
        self.image_sub = rospy.Subscriber("/aruco_position", Float64MultiArray, self.aruco_position_callback)
        # Subscribe to the odometry information
        self.sub_odom = rospy.Subscriber("kobuki/odom", Odometry, self.odom_callback)
        time.sleep(1)  # Sleep to ensure the subscriptions are set up

    def setup(self):
      
        self.logger.debug("  %s [MoveRobotToPick::setup()]" % self.name)
        # Wait for the services to be available
        rospy.wait_for_service('goal_server')
        rospy.wait_for_service('weight_server')
        rospy.wait_for_service('aruco_server')
        rospy.wait_for_service('task_server')
        try:
            # Create service proxies
            self.set_aruco = rospy.ServiceProxy('aruco_server', intervention)
            self.set_goal = rospy.ServiceProxy('goal_server', intervention)
            self.set_weight = rospy.ServiceProxy('weight_server', intervention)
            self.set_task = rospy.ServiceProxy('task_server', intervention)
            self.logger.debug("  %s [MoveRobotToPick::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [MoveRobotToPick::setup()]" % self.name)

    def initialise(self):
       
        self.goal_xyz = 0
        self.distance = 0
        self.aruco_pose = 0
        # Define weights for the robot's movements
        self.weight = [1.0, 1.0, 1000.0, 1000.0, 1000.0, 1000.0]
        self.weight_arm_pose = [1000.0, 1000.0, 1.0, 1.0, 1.0, 1.0]
        self.logger.debug("  %s [MoveRobotToPick::initialise()]" % self.name)

    def update(self):
        try:
            # Set up signal handler for interrupting the process
            signal.signal(signal.SIGINT, self.signal_handler)
            for i in range(2):
                self.logger.debug("  {}: Publishing goal position".format(self.name))
                if i == 0:
                    rospy.logerr('1st iteration')
                    # Call the Aruco service to get the position
                    response = self.set_aruco()
                    time.sleep(0.5)
                    # Set the initial goal position
                    goal_position = [0.3, 0.01, -0.25, 0.0]
                    # Set the weight for the arm
                    response = self.set_weight(self.weight_arm_pose)
                    # Send the goal position to the server
                    response = self.set_goal(goal_position)
                    time.sleep(0.2)
                    threshold = 0.08
                    print('Next Goal Point')
                    # Update the goal position from the blackboard
                    goal_position = blackboard.front_point
                    response = self.set_goal(goal_position)
                    self.goal_xyz = goal_position[0:3]
                    print('Goal_position',self.goal_xyz)
                    # Calculate the distance between the goal and the current end-effector position
                    distance = math.dist(self.goal_xyz, self.ee_pose)
                    while distance > threshold:
                        try:
                            distance = math.dist(self.goal_xyz, self.ee_pose)
                            print('Approaching towards the goal')
                            time.sleep(0.5)
                        except KeyboardInterrupt:
                            print('Loop stopped by user')
                            break
                else:
                    rospy.logerr('2nd iteration')
                    # Set the weight for the robot
                    response = self.set_weight(self.weight)
                    time.sleep(0.2)
                    
                    rospy.loginfo("Weight set successfully")
                    self.logger.debug("  {}: Publishing goal position".format(self.name))
                    rospy.loginfo("  {}: Publishing goal position".format(self.aruco_pose))
                    # Update the goal position from the Aruco marker
                    goal_position = blackboard.aruco
                    print('Goal position', goal_position)
                    response = self.set_goal(goal_position)
                    time.sleep(1)
                    rospy.loginfo("Goal set successfully")
                    self.goal_xyz = goal_position[0:3]
                    threshold = 0.065
                    distance = math.dist(self.goal_xyz, self.ee_pose)
                    while distance > threshold:
                        try:
                            distance = math.dist(self.goal_xyz, self.ee_pose)
                          
                            print('Approaching towards the goal')
                            time.sleep(1)
                        except KeyboardInterrupt:
                            print('Loop stopped by user')
                            break
                if self.distance < threshold:
                    rospy.logwarn('Goal reached')
            return py_trees.common.Status.SUCCESS
        except:
            self.logger.debug(
                "  {}: (Error  in MoveRobotToPick block) calling service".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [MoveRobotToPick::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

    def ee_pose_callback(self, data):
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    def aruco_position_callback(self, aruco_msg):
        self.aruco_pose = aruco_msg.data
        blackboard.aruco = self.aruco_pose
        print('Aruco_position', self.aruco_pose)

    def odom_callback(self, odom_data):
        self.dt = odom_data.twist.twist.linear.x
        quaternion = (odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.robot_state = np.array([odom_data.pose.pose.position.x, odom_data.pose.pose.position.y])
        self.front_point = [self.robot_state[0] + 0.3, self.robot_state[1] - 0.01, -0.3, 0.0]
        blackboard.front_point = self.front_point

    def signal_handler(self, signal, frame):
        print("Loop stopped by user")
        exit(0)

# Class to move the robot to a place location
class MoveRobotToPlace(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(MoveRobotToPlace, self).__init__(name)
        # Subscribe to the pose of the end-effector
        self.sub_pose_ee = rospy.Subscriber('pose_EE', PoseStamped, self.ee_pose_callback)
        # Subscribe to the position of Aruco markers
        self.image_sub = rospy.Subscriber("/aruco_position", Float64MultiArray, self.aruco_position_callback)
        # Subscribe to the odometry information
        self.sub_odom = rospy.Subscriber("kobuki/odom", Odometry, self.odom_callback)
        time.sleep(1)

    def setup(self):
        self.logger.debug("  %s [MoveRobotToPlace::setup()]" % self.name)
        # Wait for the services to be available
        rospy.wait_for_service('goal_server')
        rospy.wait_for_service('weight_server')
        rospy.wait_for_service('aruco_server')
        rospy.wait_for_service('task_server')
        try:
            # Create service proxies
            self.set_aruco = rospy.ServiceProxy('aruco_server', intervention)
            self.set_goal = rospy.ServiceProxy('goal_server', intervention)
            self.set_weight = rospy.ServiceProxy('weight_server', intervention)
            self.set_task = rospy.ServiceProxy('task_server', intervention)
            self.logger.debug("  %s [MoveRobotToPlace::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [MoveRobotToPlace::setup()]" % self.name)

    def initialise(self):
    
        self.goal_xyz = 0
        self.distance = 0
        # Define weights for the robot's movements
        self.weight = [1.0, 1.0, 1000.0, 1000.0, 1000.0, 1000.0]
        self.logger.debug("  %s [MoveRobotToPlace::initialise()]" % self.name)

    def update(self):
        try:
            # Set up signal handler for interrupting the process
            signal.signal(signal.SIGINT, self.signal_handler)
            self.logger.debug("  %s [MoveRobotToPlace::update()]" % self.name)
            # Set the weight for the robot
            response = self.set_weight(self.weight)
            time.sleep(0.2)
        
            rospy.loginfo("Weight set successfully")
            # Define the goal position
            goal_position = [self.robot_state[0] + 1.5, -0.01, -0.36, 0.0]
            print('Goal_position', goal_position)
            # Send the goal position to the server
            response = self.set_goal(goal_position)
            time.sleep(1)
            rospy.loginfo("Goal set successfully")
            self.goal_xy = goal_position[0:2]
            print('Goal_position', self.goal_xy)
            threshold = 0.2
            # Calculate the distance between the goal and the current robot state
            distance = math.dist(self.goal_xy, self.robot_state)
            while distance > threshold:
                try:
                    distance = math.dist(self.goal_xy, self.robot_state)
                   
                    print('Approaching towards the goal')
                    time.sleep(0.5)
                except KeyboardInterrupt:
                    print('Loop stopped by user')
                    break
            return py_trees.common.Status.SUCCESS
        except:
            self.logger.debug(
                "  {}: (Error  in MoveRobotToPlace block) calling service".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [MoveRobotToPlace::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

    def ee_pose_callback(self, data):
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    def aruco_position_callback(self, aruco_msg):
        self.aruco_pose = aruco_msg.data
        blackboard.aruco = self.aruco_pose
        print('Aruco_position', self.aruco_pose)

    def odom_callback(self, odom_data):
        quaternion = (odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.robot_state = np.array([odom_data.pose.pose.position.x, odom_data.pose.pose.position.y])

    def signal_handler(self, signal, frame):
        print("Loop stopped by user")
        exit(0)

# Class to handle picking points
class PickPoints(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(PickPoints, self).__init__(name)
        # Subscribe to the pose of the end-effector
        self.sub_pose_ee = rospy.Subscriber('pose_EE', PoseStamped, self.ee_pose_callback)
        # Subscribe to the odometry information
        self.sub_odom = rospy.Subscriber("kobuki/odom", Odometry, self.odom_callback)
        time.sleep(2)

    def setup(self):
        self.logger.debug("  %s [PickPoints::setup()]" % self.name)
        # Wait for the services to be available
        rospy.wait_for_service('goal_server')
        rospy.wait_for_service('weight_server')
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        try:
            # Create service proxies
            self.set_pump_proxy = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            self.set_goal = rospy.ServiceProxy('goal_server', intervention)
            self.set_weight = rospy.ServiceProxy('weight_server', intervention)
            self.logger.debug("  %s [PickPoints::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [PickPoints::setup()]" % self.name)

    def initialise(self):
        self.goal_xyz = 0
        self.distance = 0
        # Define weights for the robot's movements
        self.weight = [1000.0, 1000.0, 1.0, 1.0, 1.0, 1.0]
        self.angle = float(math.radians(0))
        self.goal_position = blackboard.aruco
        # Define pick points
        pick_1 = [self.goal_position[0], self.goal_position[1], -0.144, self.angle]
        pick_2 = [self.goal_position[0] + 0.05, self.goal_position[1], -0.3, self.angle]
        pick_3 = [self.robot_state[0], self.robot_state[1] - 0.27, -0.365, self.angle]
        self.point_locations = [pick_1, pick_2, pick_3]
        self.gripper_on = False
        self.logger.debug("  %s [PickPoints::initialise()]" % self.name)

    def update(self):
        try:
            # Set up signal handler for interrupting the process
            signal.signal(signal.SIGINT, self.signal_handler)
            # Set the weight for the robot
            response = self.set_weight(self.weight)
            rospy.loginfo("Weight set successfully")
            if self.point_locations:
                for i in range(len(self.point_locations)):
                    self.logger.debug("  {}: Publishing goal position".format(self.name))
                    point = self.point_locations.pop(0)
                    goal_point = point
                    # Send the goal position to the server
                    response = self.set_goal(goal_point)
                    time.sleep(0.5)
                    if i == 0:
                        response = self.set_pump_proxy(True)
                        time.sleep(0.5)
                    time.sleep(2)
                    rospy.loginfo("Goal set successfully")
                    self.goal_xyz = goal_point[0:3]
                    threshold = 0.02
                    # Calculate the distance between the goal and the current end-effector position
                    distance = math.dist(self.goal_xyz, self.ee_pose)
                    while distance > threshold:
                        try:
                            distance = math.dist(self.goal_xyz, self.ee_pose)
                          
                            print('Approaching towards the goal')
                            time.sleep(0.5)
                        except KeyboardInterrupt:
                            print('Loop stopped by user')
                            break
                    rospy.logwarn('Goal reached')
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        except:
            self.logger.debug(
                "  {}: (Error  in PickPoints block) calling service ".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [PickPoints::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

    def ee_pose_callback(self, data):
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    def odom_callback(self, odom_data):
        quaternion = (odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.robot_state = np.array([odom_data.pose.pose.position.x, odom_data.pose.pose.position.y])

    def signal_handler(self, signal, frame):
        print("Loop stopped by user")
        exit(0)

# Class to handle placing points
class PlacePoints(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(PlacePoints, self).__init__(name)
        # Subscribe to the pose of the end-effector
        self.sub_pose_ee = rospy.Subscriber('pose_EE', PoseStamped, self.ee_pose_callback)
        # Subscribe to the odometry information
        self.sub_odom = rospy.Subscriber("kobuki/odom", Odometry, self.odom_callback)
        time.sleep(2)

    def setup(self):
        self.logger.debug("  %s [PlacePoints::setup()]" % self.name)
        # Wait for the services to be available
        rospy.wait_for_service('goal_server')
        rospy.wait_for_service('weight_server')
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        try:
            # Create service proxies
            self.set_pump_proxy = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            self.set_goal = rospy.ServiceProxy('goal_server', intervention)
            self.set_weight = rospy.ServiceProxy('weight_server', intervention)
            self.logger.debug("  %s [PlacePoints::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [PlacePoints::setup()]" % self.name)

    def initialise(self):
        self.goal_xyz = 0
        self.distance = 0
        # Define weights for the robot's movements
        self.weight = [1000.0, 1000.0, 1.0, 1.0, 1.0, 1.0]
        self.angle = float(math.radians(0))
        # Define place points
        place_1 = [self.robot_state[0] + 0.25, self.robot_state[1] - 0.175, -0.3, self.angle]
        place_2 = [self.robot_state[0] + 0.24, self.robot_state[1] - 0.175, -0.134, self.angle]
        self.point_locations = [place_1, place_2]
        self.gripper_on = True
        self.logger.debug("  %s [PlacePoints::initialise()]" % self.name)

    def update(self):
        try:
            # Set up signal handler for interrupting the process
            signal.signal(signal.SIGINT, self.signal_handler)
            # Set the weight for the robot
            response = self.set_weight(self.weight)
            rospy.loginfo("Weight set successfully")
            if self.point_locations:
                for i in range(len(self.point_locations)):
                    self.logger.debug("  {}: Publishing goal position".format(self.name))
                    point = self.point_locations.pop(0)
                    goal_point = point
                    # Send the goal position to the server
                    response = self.set_goal(goal_point)
                    time.sleep(0.5)
                    time.sleep(2)
                    rospy.loginfo("Goal set successfully")
                    self.goal_xyz = goal_point[0:3]
                    threshold = 0.05
                    # Calculate the distance between the goal and the current end-effector position
                    distance = math.dist(self.goal_xyz, self.ee_pose)
                    while distance > threshold:
                        try:
                            distance = math.dist(self.goal_xyz, self.ee_pose)
                           
                            print('Approaching towards the goal')
                            time.sleep(0.5)
                        except KeyboardInterrupt:
                            print('Loop stopped by user')
                            break
                    rospy.logwarn('Goal reached')
                    if i == 1:
                        response = self.set_pump_proxy(False)
                        time.sleep(0.5)
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        except:
            self.logger.debug(
                "  {}: (Error  in PlacePoints block) calling service ".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [PlacePoints::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

    def ee_pose_callback(self, data):
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    def odom_callback(self, odom_data):
        quaternion = (odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.robot_state = np.array([odom_data.pose.pose.position.x, odom_data.pose.pose.position.y])

    def signal_handler(self, signal, frame):
        print("Loop stopped by user")
        exit(0)

# Class to move the robot to the home position
class MoveToHome(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(MoveToHome, self).__init__(name)
        # Subscribe to the pose of the end-effector
        self.sub_pose_ee = rospy.Subscriber('pose_EE', PoseStamped, self.ee_pose_callback)
        # Subscribe to the odometry information
        self.sub_odom = rospy.Subscriber("kobuki/odom", Odometry, self.odom_callback)
       
        time.sleep(2)

    def setup(self):
        self.logger.debug("  %s [MoveToHome::setup()]" % self.name)
        # Wait for the goal server to be available
        rospy.wait_for_service('goal_server')
        rospy.wait_for_service('goal_server')
        try:
            # Create service proxy
            self.set_goal = rospy.ServiceProxy('goal_server', intervention)
            self.set_weight = rospy.ServiceProxy('weight_server', intervention)
            self.logger.debug("  %s [MoveToHome::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [MoveToHome::setup()]" % self.name)

    def initialise(self):
        self.goal_xyz = 0
        self.distance = 0
        # Define the home position for the robot base 
        self.weight = [1.0, 1.0, 1000.0, 1000.0, 1000.0, 1000.0]
        self.home_position = [0.0, 0.0, 0.0, 0.0]
        self.source_frame = 'world_ned'
        self.logger.debug("  %s [MoveToHome::initialise()]" % self.name)

    def update(self):
        try:
            # Set up signal handler for interrupting the process
            signal.signal(signal.SIGINT, self.signal_handler)
            # Move the robot base to the home position
            response=self.set_weight(self.weight)
            response = self.set_goal(self.home_position)
            time.sleep(0.5)
            rospy.loginfo("Home position goal set successfully")
            self.goal_xy = self.home_position[0:2]
            print('Goal_position',self.goal_xy)
            threshold = 0.4
            # Calculate the distance between the goal and the current end-effector position
            distance = math.dist(self.goal_xy, self.robot_state)
            while distance > threshold:
                try:
                    distance = math.dist(self.goal_xy, self.robot_state)
                   
                    print('Approaching towards home position')
                    time.sleep(0.5)
                    if distance<threshold:
                        rospy.logwarn('Home position reached')
                        break
                except KeyboardInterrupt:
                    print('Loop stopped by user')
                    break
            
            return py_trees.common.Status.SUCCESS
       
        except rospy.ServiceException as e:
            self.logger.debug("  %s [MoveToHome::update() Service call failed: %s]" % (self.name, str(e)))
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        self.logger.debug("  %s [MoveToHome::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

    def ee_pose_callback(self, data):
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    def odom_callback(self, odom_data):
        quaternion = (odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.robot_state = np.array([odom_data.pose.pose.position.x, odom_data.pose.pose.position.y])

    def signal_handler(self, signal, frame):
        print("Loop stopped by user")
        exit(0)

# Function to create the behavior tree
def create_tree():
    # Create instances of each behavior
    move_robot_to_pick = MoveRobotToPick("Move_Robot_To_Pick_Object")
    pick_points = PickPoints("Pick_Object")
    move_robot_to_place = MoveRobotToPlace("Move_Robot_To_Place_Object")
    place_points = PlacePoints("Place_Object")
    move_to_home = MoveToHome("Move_To_Home_Position")

    # Create the root of the behavior tree
    root = py_trees.composites.Sequence(name="Aruco_Pick_and_Place", memory=True)
    # Add the behaviors as children of the root
    root.add_children([move_robot_to_pick, pick_points, move_robot_to_place, place_points, move_to_home])

    return root

# Function to execute the behavior tree
def execute_behavior_tree(tick_count=1):
    """
    Sets up and executes the behavior tree for a specified number of ticks.
    
    :param tick_count: Number of times the tree should be ticked.
    """
    root = create_tree()
    behavior_tree = py_trees.trees.BehaviourTree(root)
    py_trees.display.render_dot_tree(root, name="behavior_tree")  # Save the tree structure to a dot file

    try:
        print("Setting up for all tree children...")
        behavior_tree.setup(timeout=15)  # Setup the tree with a timeout

        # Tick the tree for the specified number of ticks or until interrupted
        for _ in range(tick_count):
            try:
                behavior_tree.tick()
                rospy.sleep(1)  # Simulate time passing (e.g., waiting for 1 second before the next tick)
            except KeyboardInterrupt:
                print("Interrupted by the user.")
                break
    except KeyboardInterrupt:
        print("Execution interrupted.")

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    blackboard = py_trees.blackboard.Blackboard()
    rospy.init_node("behavior_tree")
    execute_behavior_tree(tick_count=1)
    rospy.spin()
