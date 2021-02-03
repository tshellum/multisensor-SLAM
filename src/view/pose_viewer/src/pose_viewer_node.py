import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation

from geometry_msgs.msg import PoseStamped




class Viewer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 10000)
        self.ax.set_ylim(-7, 7)
        return self.ln

    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return yaw   

    def odom_callback(self, msg):
        yaw_angle = self.getYaw(msg.pose.pose)
        self.y_data.append(yaw_angle)
        x_index = len(self.x_data)
        self.x_data.append(x_index+1)

    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln



def main():
    rospy.init_node('pose_visual_node')
    viz = Viewer()
    sub = rospy.Subscriber('pose_relative_topic', Odometry, viz.odom_callback)

    ani = FuncAnimation(viz.fig, viz.update_plot, init_func=viz.plot_init)
    plt.show(block=True)


if __name__ == '__main__':
    main()