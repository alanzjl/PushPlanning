import numpy as np
import math
from math import pi, sin, cos, asin, acos, atan, atan2, sqrt
from utils import *
from matplotlib import pyplot as plt

target_speed = 0.2
target_angular = 0.1

class MotionPath:
    def target_state(self, s):
        """
        Target position of turtlebot given the path length s

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        raise NotImplementedError()

    def transform_dest(self, s, state_):
        return state_

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the path length s

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        raise NotImplementedError()

    @property
    def total_length(self):
        """ total path length
        Returns
        -------
        float
            total path length
        """
        raise NotImplementedError()

    @property
    def end_state(self):
        """ Final state after completing the path
        Returns
        -------
        :obj:`numpy.ndarray`
            Final state after completing the path
        """
        return self.target_state(self.total_length)

class AnglePath(MotionPath):
    def __init__(self, angle, left_turn):
        self.angle = abs(angle)
        self.left_turn = left_turn if angle >= 0 else not left_turn
        # speed of turning: turning 2pi equals going forward 2m
        self.length = 2. * self.angle / (2 * pi)

    def target_state(self, s):
        # for pure turning, assume distance is the same with going forward
        theta = self.angle * s / self.length
        if self.left_turn:
            return np.asarray([0, 0, theta])
        else:
            return np.asarray([0, 0, -theta])
    
    def target_velocity(self, s):
        t = self.length / target_speed
        w = self.angle / t
        if self.left_turn:
            return np.asarray([0, 0, w])
        else:
            return np.asarray([0, 0, -w])

    @property
    def total_length(self):
        return self.length

class ArcPath(MotionPath):
    def __init__(self, radius, angle, left_turn):
        """
        Parameters
        ----------
        radius: float
            how big of a circle in meters
        angle: float
            how much of the circle do you want to complete (in radians).  
            Can be positive or negative
        left_turn: bool
            whether the turtlebot should turn left or right
        """
        self.radius = radius
        self.angle = angle
        self.left_turn = left_turn
        self.length = abs((angle / (2*pi)) * (2. * pi * radius))

    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Circular Arc Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        # YOUR CODE HERE
        arcLength = abs(2. * pi * self.radius * (self.angle / (2. * pi)))
        # traveled angle
        # TODO: whatif s is negative
        angle = self.angle * (s / arcLength)
        x = self.radius * sin(angle)
        y = self.radius * (1. - cos(angle))
        # when angle < 0, going backwards instead of turning around
        theta = angle
        if self.left_turn:
            return np.asarray([x, y, theta])
        else:
            return np.asarray([x, -y, -theta])


    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Circular Arc Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        # YOUR CODE HERE
        if s > self.length:
            return np.asarray([0., 0., 0.])

        arcLength = abs(2. * pi * self.radius * (self.angle / (2. * pi)))
        angle = self.angle * (s / arcLength)

        # angular velocity
        if (self.left_turn and self.angle < 0) or (not self.left_turn and self.angle > 0):
            w = -1 * target_angular
        else:
            w = target_angular
        vx = target_speed * cos(angle)
        vy = target_speed * sin(angle)
        if self.left_turn:
            return np.asarray([vx, vy, w])
        else:
            return np.asarray([vx, -vy, -w])


    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        return self.length

class LinearPath(MotionPath):
    def __init__(self, length):
        """
        Parameters
        ----------
        length: float
            length of the path
        """
        self.length = length

    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Linear Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        # YOUR CODE HERE
        return np.asarray([s, 0., 0.])

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Linear Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        # YOUR CODE HERE
        if s >= self.length:
            return np.asarray([0., 0., 0.])
        return np.asarray([target_speed, 0., 0.])


    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        return self.length

class ChainPath(MotionPath):
    def __init__(self, subpaths):
        """
        Parameters
        ----------
        subpaths: :obj:`list` of :obj:`MotionPath`
            list of paths which should be chained together
        """
        self.subpaths = subpaths

    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Chained Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        # YOUR CODE HERE
        tmp = s
        state = np.asarray([0., 0., 0.])
        trans = np.eye(3)
        for i in range(len(self.subpaths)):
            if tmp > self.subpaths[i].total_length:
                if i == len(self.subpaths) - 1:
                    return compute_twist(trans, state, self.subpaths[i].end_state)
                tmp -= self.subpaths[i].total_length
                state = compute_twist(trans, state, self.subpaths[i].end_state)
                trans = rigid(state)
            else:
                return compute_twist(trans, state, self.subpaths[i].target_state(tmp))
        print("path error")
        exit(1)

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Chained Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        tmp = s
        for i in range(len(self.subpaths)):
            if i == len(self.subpaths) - 1:
                return self.subpaths[i].target_velocity(tmp)
            if tmp > self.subpaths[i].total_length:
                tmp -= self.subpaths[i].total_length
            else:
                return self.subpaths[i].target_velocity(tmp)
        print("finding target_vel error")
        exit(1)
    
    def transform_dest(self, s, state_):
        """
        Transform from start point frame to last destination frame
        """
        tmp = s
        state, theta = get_state(state_)
        for  i in range(len(self.subpaths)):
            if tmp > self.subpaths[i].total_length:
                s, t = get_state(self.subpaths[i].end_state)
                trans = np.linalg.pinv(rigid(self.subpaths[i].end_state))
                state = trans.dot(state)
                theta -= t
                tmp -= self.subpaths[i].total_length
            else:
                return np.asarray([state[0], state[1], theta])
        return np.asarray([state[0], state[1], theta])


    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        s = 0.
        for p in self.subpaths:
            s += p.total_length
        return s

def get_state(state):
    return np.asarray([state[0], state[1], 1.]), state[2]

def compute_twist(trans, state_ori, state_new):
    tmp = np.asarray([state_new[0], state_new[1], 1.])
    tmp = trans.dot(tmp)
    theta = state_new[2] + state_ori[2]
    if theta > -pi and theta < -(pi - 1e-3):
        theta = - (pi - 1e-3)
    elif theta < pi and theta > pi - 1e-3:
        theta = pi - 1e-3
    elif theta > pi and theta < 2.0 * pi:
        theta = theta - 2.0 * pi
    elif theta < -pi and theta > -2.0 * pi:
        theta = theta + 2.0 * pi
    elif theta > 2.0 * pi:
        theta -= 2.0 * pi
    elif theta < -2.0 * pi:
        theta += 2.0 * pi

    return np.asarray([tmp[0], tmp[1], theta])

def compute_obstacle_avoid_path(dist, obs_center, obs_radius):
    # YOUR CODE HERE
    r = 1.2 * obs_radius
    p = obs_center[0]
    R = (p**2 - r**2) / (2.0 * r)
    theta1 = atan(p / R)
    theta2 = 2. * theta1
    path = ChainPath([
        ArcPath(R, theta1, False),
        ArcPath(r, theta2, True),
        ArcPath(R, theta1, False),
    ])
    return path

def pathGenerate(points):
    # current heading direction
    theta = 0.
    # starting point is (0., 0.)
    start = (0., 0.)
    paths = []
    for end in points:
        turn_abs = atan2((end[1] - start[1]), (end[0] - start[0]))
        turn = turn_abs - theta
        # print '----------'
        if turn > pi:
            turn -= 2 * pi
        if turn < -pi:
            turn += 2 * pi
        turnpath = AnglePath(abs(turn), turn > 0) 
        # print "turn: ", abs(turn), turn > 0
        linearpath = LinearPath(sqrt((end[1] - start[1]) ** 2 + (end[0] - start[0]) ** 2))
        theta = turn_abs
        start = end
        if abs(turn) > 1e-2: 
            paths.append(turnpath)
        paths.append(linearpath)
    return ChainPath(paths)


def plot_path(path):
    """
    Plots on a 2D plane, the top down view of the path passed in

    Parameters
    ----------
    path: :obj:`MotionPath`
        Path to plot
    """
    s = np.linspace(0, path.total_length, 1000, endpoint=False)
    twists = np.array(list(path.transform_dest(si, path.target_state(si)) for si in s))
    twists2 = np.array(list(path.target_state(si) for si in s))

    # plt.plot(twists[:,0], twists[:,1], "ro", linewidth=2)
    plt.plot(twists2[:,0], twists2[:,1], "ro", linewidth=2)
    # plt.plot(s, twists[:, 2], "bo")
    plt.show()

# YOUR CODE HERE
parallel_parking_path = ChainPath([
    LinearPath(1.0),
    ArcPath(.5, -pi/2, False),
    ArcPath(.5, -pi/2, True)
    ])

# YOUR CODE HERE
three_point_turn_path = ChainPath([
    ArcPath(.5, pi/2, False),
    ArcPath(.5, -pi/2, True),
    LinearPath(1.0)
    ])

if __name__ == '__main__':
    # path = three_point_turn_path
    # path = ArcPath(5., pi/2, True)
    # path = compute_obstacle_avoid_path(0., [1.0, 0.0], 0.5)
    # path = parallel_parking_path
    path = pathGenerate([(1,0), (2, 1), (3, 0)])
    print(path.total_length)
    print(path.end_state)
    plot_path(path)
