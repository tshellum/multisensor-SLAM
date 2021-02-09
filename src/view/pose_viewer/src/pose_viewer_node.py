#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
# import tf
from tf2_msgs.msg import TFMessage

import os
import numpy as np
from matplotlib import pyplot as plt


# Alt: https://answers.ros.org/question/264767/plotting-real-time-data/


class Viewer:
	def __init__(self):
		# self.fig, self.ax = plt.subplots()
		# # self.ln, = plt.plot([], [], 'ro')
		self.xdata_vo, self.ydata_vo = [], []
		self.xdata_gt, self.ydata_gt = [], []
		self.stamp = []

		# im = plt.imread(os.path.abspath(__file__+"../../../../../../resources/kitti00.png"))
		# plt.imshow(im)


	def vo_callback(self, msg):
		# rospy.loginfo("x:%f y:%f z:%f w:%f x:%f y:%f z:%f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)

		stamp = msg.header.stamp
		self.stamp.append(stamp.secs + stamp.nsecs * 1e-9)		
		self.xdata_vo.append(msg.pose.position.x)
		self.ydata_vo.append(msg.pose.position.y)

		plt.title('Visual odometry pose estimate')

		plt.xlabel('Longitude [m]')
		plt.ylabel('Latitude [m]')

		plt.plot(msg.pose.position.y, msg.pose.position.x, '*', color='darkblue')
		plt.axis("equal")
		plt.draw()
		plt.pause(0.00000000001)

	def gt_callback(self, msg):
		# rospy.loginfo("x:%f y:%f z:%f w:%f x:%f y:%f z:%f", msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z, msg.transforms[0].transform.rotation.w, msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z)
		
		self.xdata_gt.append(msg.transforms[0].transform.translation.z)
		self.ydata_gt.append(msg.transforms[0].transform.translation.x)

		plt.title('Visual odometry pose estimate')

		plt.xlabel('Longitude [m]')
		plt.ylabel('Latitude [m]')

		plt.plot(msg.transforms[0].transform.translation.z, -msg.transforms[0].transform.translation.x, '*', color='red')
		plt.axis("equal")
		plt.draw()
		plt.pause(0.00000000001)


if __name__ == '__main__':
	rospy.init_node('Pose viewer')
	rospy.loginfo("\n\n\n ----- Starting pose viewer ----- \n")

	viz = Viewer()
	rospy.Subscriber('pose_world_topic', PoseStamped, viz.vo_callback)
	# rospy.Subscriber('pose_ground_truth_topic', TFMessage, viz.gt_callback)

	plt.ion()
	plt.show()

	rospy.spin()


