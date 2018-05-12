# imports may be necessary
from geometry_msgs.msg import Twist
import numpy as np
from math import *

class Controller():
	def __init__(self, path, target_speed, obstacle, obstacle_center, obstacle_radius):
		self.path = path
		self.target_speed = target_speed
		self.obstacle = obstacle
		self.obstacle_center = obstacle_center
		self.obstacle_radius = obstacle_radius
		self.error_x = 0
		self.error_theta = 0.

	def force_controller(self, current_state):
		dest_force = 10.
		k_obs = 10.
		k_p = 3.
		x, y, theta = current_state
		x_obs, y_obs = self.obstacle_center

		x_des = x_obs * 2.
		y_des = 0

		if theta > pi: theta = 2. * pi - theta
		if theta < -pi: theta = 2. * pi + theta

		dist = sqrt((x_obs - x)**2 + (y_obs - y)**2)
		f_obs_ = k_obs / dist
		if (dist > self.obstacle_radius):
			f_obs_ = 0.

		f_obs = f_obs_ * self.compute_force([x, y], [x_obs, y_obs])
		f_des = dest_force * self.compute_force([x_des, y_des], [x, y])
		print "------------------------------------------------------"

		print f_obs, f_des

		f = f_obs + f_des
		f_x, f_y = f
		theta_ = atan(f_y / f_x)
		print "theta_", theta_, "theta", theta

		v_ = self.target_speed
		w_ = (theta_ - theta) * k_p

		print v_, w_

		if w_ > 0.45: w_ = 0.45
		if w_ < -0.45: w_ = -0.45

		return (v_, w_)

	def compute_force(self, from_, to_):
		x, y = to_
		x_, y_ = from_
		theta = atan((y - y_) / (x - x_))
		return np.asarray([cos(theta), sin(theta)])

	def theta_diff(self, t_, t):
		#print '-------------------'
		#print t_, t
		#print t_ - t
		diff = t_ - t
		'''
		while t_ < 0: t_ += 2 * pi
		while t < 0: t += 2 * pi
		t_ = t_ %  (2 * pi)
		t = t %  (2 * pi)
		'''
		#print t_, t
		diff2 = t_ - t
		if diff2 > pi:
			diff2 = diff2 - 2 * pi
		elif diff2 < -pi:
			diff2 = diff2 + 2 * pi
		#print diff2
		return diff2

	def pid_controller(self, state_, state):
		p = 1.0
		i = 0.3

		x, y, theta = state
		x_, y_, theta_ = state_
		cmd = Twist()
		e_x = x_ - x
		self.error_x += e_x
		e_theta = self.theta_diff(theta_, theta)
		self.error_theta += e_theta
		cmd.linear.x = p * e_x + i * self.error_x
		cmd.linear.y = 0
		cmd.linear.z = 0
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = p* e_theta + i * self.error_theta

		return cmd

	def step_path(self, current_state, s, use_force_controller = False, use_pid_controller = False):
		"""
		Takes the current state and final state and returns the twist command to reach the target state
		according to the path variable

		Parameters
		----------
		current_state: :obj:`numpy.ndarray`
			twist representing the current state of the turtlebot.  see utils.py
		s: float
			the path length the turtlebot should have travelled so far

		Returns
		-------
		:obj:`geometry_msgs.msg.Twist`
			Twist message to be sent to the turtlebot

		"""
		# YOUR CODE HERE
		target_vel = self.path.target_velocity(s)
		cmd = Twist()

		k1 = 2.0
		k2 = 1.5
		k3 = 1.5

		if not use_force_controller:
			x, y, theta = self.path.transform_dest(s, current_state)
			x_, y_, theta_ = self.path.transform_dest(s, self.path.target_state(s))
			'''
			if theta < 0:
				theta = 2 * pi + theta
			if theta_ < 0:
				theta_ = 2 * pi + theta_
			'''
			vx_, vy_, w_ = self.path.target_velocity(s)

			C = np.diag([cos(self.theta_diff(theta_, theta)), 1.])
			u1 = -k1 * (x_ - x)
			u2 = k2 * self.target_speed * np.sinc(self.theta_diff(theta_, theta)) * (y_ - y) - k3 * self.theta_diff(theta_, theta)

			# print cos(theta_ - theta) * self.target_speed, x_ - x

			res = C.dot(np.asarray([[self.target_speed], [w_]])) - np.asarray([[u1], [u2]])

			vd = res[0][0]
			wd = res[1][0]
		else:
			vd, wd = self.force_controller(current_state)

		# print vd, wd


		cmd.linear.x = vd
		cmd.linear.y = 0
		cmd.linear.z = 0
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = wd
		if use_pid_controller:
			state = self.path.transform_dest(s, current_state)
			state_ = self.path.transform_dest(s, self.path.target_state(s))
			cmd = self.pid_controller(state_, state)

		# print cmd
		"""

		cmd.linear.x = vx_
		cmd.linear.y = vy_
		cmd.linear.z = 0
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = w_
		"""
		return cmd


