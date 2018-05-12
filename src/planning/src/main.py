#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from paths import *
from utils import *
from math import *
from controllers import *
import tf
import tf.transformations as tfs
from baseTracker import BaseTrack
import pathGen
from objectTracker import ObstacleTrack

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

# path = parallel_parking_path
# path = three_point_turn_path
# path = pathGenerate([
#     (2 - l/2 - d/2,0),
#     (1.7 - l/2 - d/2,0),
#     (1.7 - l/2 - d/2, -1.0),
#     (2, -1.0),
#     (2, 2 - l/2 - d/2)
#     ])
# path = compute_obstacle_avoid_path(0.0, vec(1.0, -0.0), 0.5)
# path = LinearPath(2.5)
# path = AnglePath(pi, True)
# path = ArcPath(1, -pi/2, True)
pathGenerator = pathGen.PathGenerator()
object_target_loc, path = pathGenerator.path()

k = []
# what is k?

# import target speed from paths
# target_speed = 0.

obstacle = False
obstacle_center = vec(1.0, -0.2)
obstacle_radius = 0.5

# controller = Controller(path, k, target_speed, obstacle, obstacle_center, obstacle_radius)
controller = Controller(path, target_speed, obstacle, obstacle_center, obstacle_radius)

base_track = BaseTrack()
object_track = ObstacleTrack()

GAZEBO = False

def main():
    rospy.init_node('Lab3', anonymous=False)

    rospy.loginfo("To stop TurtleBot CTRL + C")
    rospy.on_shutdown(shutdown)
    
    # setting up the transform listener to find turtlebot position
    listener = tf.TransformListener()
    from_frame = 'odom'
    to_frame = 'base_link'
    # listener.waitForTransform(from_frame, to_frame, rospy.Time(0))
    broadcaster = tf.TransformBroadcaster()

    # this is so that each loop of the while loop takes the same amount of time.  The controller works better 
    # if you have this here
    rate = rospy.Rate(10)

    # getting the position of the 
    listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(5.0))
    if GAZEBO:
        start_pose = base_track.pose
        start_pos = [start_pose.position.x, start_pose.position.y]
        start_rot = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]
    else:
        start_pos, start_rot = listener.lookupTransform(from_frame, to_frame, listener.getLatestCommonTime(from_frame, to_frame))
    # 3x1 array, representing (x,y,theta) of robot starting state
    start_state = np.asarray([start_pos[0], start_pos[1], getYawFromQuat(start_rot)])

    times = []
    actual_states = []
    target_states = []
    actual_object_states = []
    s = 0
    while not rospy.is_shutdown() and s <= path.total_length:
        if GAZEBO:
            current_pose = base_track.pose
            current_pos = [current_pose.position.x, current_pose.position.y]
            current_rot = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        else:
            current_pos, current_rot = listener.lookupTransform(from_frame, to_frame, listener.getLatestCommonTime(from_frame, to_frame))

        # 3x1 array, representing (x,y,theta) of current robot state
        current_state = np.asarray([current_pos[0], current_pos[1], getYawFromQuat(current_rot)])

        # 3x1 array representing (x,y,theta) of current robot state, relative to starting state.  look at rigid method in utils.py
        current_xy = np.asarray([current_pos[0], current_pos[1], 1.0])
        current_xy = np.linalg.pinv(rigid(start_state)).dot(current_xy)
        current_state = np.asarray([current_xy[0], current_xy[1], current_state[2] - start_state[2]])
        target_state = path.target_state(s)
        # print current_state, target_state

        # for the plot at the end
        times.append(s * 10)
        actual_states.append(current_state)
        target_states.append(target_state)

        current_object_pose = object_track.pose
        actual_object_states.append(np.asarray([current_object_pose.position.x, current_object_pose.position.y]))

        # I may have forgotten some parameters here
        move_cmd = controller.step_path(current_state, s)
        cmd_vel.publish(move_cmd)

        # I believe this should be the same as the ros rate time, so if you change that, change it here too
        s += target_speed / 10
        # this is governing how much each loop should run for.  look up ROS rates if you're interested
        rate.sleep()

    times = np.array(times)
    actual_states = np.array(actual_states)
    target_states = np.array(target_states)
    actual_object_states = np.asarray(actual_object_states)

    plt.figure()
    colors = ['blue', 'green', 'red']
    labels = ['x', 'y', 'theta']
    for i in range(3):
        plt.plot(times, actual_states[:,i], color=colors[i], ls='solid', label=labels[i])
        plt.plot(times, target_states[:,i], color=colors[i], ls='dotted')
    plt.legend()
    plt.figure()
    plt.plot(object_target_loc[:, 0], object_target_loc[:, 1], 'r-')
    plt.plot(actual_object_states[:, 0], actual_object_states[:, 1], 'b-')
    plt.show()
    
def shutdown():
    rospy.loginfo("Stopping TurtleBot")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def getYawFromQuat(rot):
    q1, q2, q3, q0 = rot
    yaw = atan2(2 * (q0 * q3 + q1 * q2), 1. - 2.*(q2 ** 2 + q3 ** 2))
    return yaw
 
if __name__ == '__main__':
    try:
        main()
    except e:
        print e
        rospy.loginfo("Lab3 node terminated.")
