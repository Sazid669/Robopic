import rosbag
import matplotlib.pyplot as plt

# Function to extract X and Y data from a specific topic in a bag file
def extract_xy_data(bag_file, topic_name, x_attr, y_attr):
    x_data = []
    y_data = []
    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        x_data.append(eval(f"msg.{x_attr}"))  # Access the x attribute dynamically
        y_data.append(eval(f"msg.{y_attr}"))  # Access the y attribute dynamically
    bag.close()
    return x_data, y_data

# File path for the ROS bag file
bag_file = '/home/syma/catkin_ws/src/hands_on_intervention/xy.bag'

# Topic names and attributes for x and y data
topic_info = [
    ('/kobuki/odom', 'pose.position.x', 'pose.position.y'),  # Adjust this to the actual fields for /kobuki/odom
    ('/pose_EE', 'pose.position.x', 'pose.position.y')  # Adjust this to the actual fields for /pose_EE
]

# Extract data from both topics
x_data_1, y_data_1 = extract_xy_data(bag_file, *topic_info[0])
x_data_2, y_data_2 = extract_xy_data(bag_file, *topic_info[1])

# Check if data is extracted properly


# Plot the X-Y data for both topics
plt.figure()
# plt.plot(x_data_1, y_data_1, 'o', color='blue', label='Topic 1 (/kobuki/odom)')  # Blue for /kobuki/odom
plt.plot(x_data_2, y_data_2, 'x', color='green', label='End-Effector Position')  # Green for /pose_EE
plt.xlabel('X Plane')
plt.ylabel('Y Plane')
plt.title('X-Y Plane Trajectory')
plt.legend()
plt.grid(True)
plt.show()
