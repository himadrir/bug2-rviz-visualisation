import rospy
import random
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2


regions = [10, 10, 10, 10, 10]
st = ['GOAL_SEEK', 'GOAL_SEEK', 'WALL_FOLLOW', 'TERMINAL']
state = 0  #0-orient 1-goal seek 2-wall follow  3-terminal
bot_pose = Point()
bot_pose.x = -8
bot_pose.y = -2

hit_pose = None
hit_count = 0
wall_flag = False
angle = 2
bot_vel = 0

twist = Twist()

goal = Point()
init_pose = Point()

goal.x = rospy.get_param('goal_pos_x')
goal.y = rospy.get_param('goal_pos_y')

init_pose.x = -8
init_pose.y = -2

top_left = None
top_right = None
front = None
left = None
right = None

distance_moved = None

threshold1 = 1
threshold_left = 0.5








def distance_to_line():
    # p0 is the current position
    # p1 and p2 points define the line
    global bot_pose, goal, init_pose
    p0 = bot_pose
    p1 = Point()
    p2 = Point()
    p1.x = init_pose.x
    p1.y = init_pose.y

    p2.x = goal.x
    p2.y = goal.y

    # here goes the equation
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance

def wall_follow():
    global regions, wall_flag, twist, left, top_left, front, top_right, right, bot_vel, threshold1, threshold_left, bot_pose, hit_pose, hit_count, distance_moved, state
    distance_moved = math.sqrt(pow(bot_pose.y - hit_pose.y, 2) + pow(bot_pose.x - hit_pose.x, 2))

    if distance_to_line() < 0.2 and distance_moved > 0.5 and wall_flag == True:
        hit_count = hit_count + 1
        state = 0
        twist.linear.x = 0
        twist.angular.z = 0
        wall_flag = False
        bot_vel.publish(twist)

    elif front < threshold1 or top_left < threshold1 or top_right < threshold1:      #wall infront turn right
        twist.linear.x = 0
        twist.angular.z = -0.5

    elif left >= threshold_left:  #move bot towards wall if drifting
        twist.linear.x = 0.5
        twist.angular.z  = 0.6
    else:                        #move along the wall
        twist.linear.x = 1
        twist.angular.z = 0

    bot_vel.publish(twist)
    return

def rotate_bot_towards_goal():
    global angle, state
    thres = 0.1

    if abs(angle) > thres:
        twist.linear.x = 0
        twist.angular.z = -0.3



    else:
        twist.linear.x = 0
        twist.angular.z = 0
        state = 1

    bot_vel.publish(twist)
    return


def bot_orientation_yaw(msg):
    global bot_pose, angle, goal

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    bot_pose = msg.pose.pose.position

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    curr_x = goal.x - x
    curr_y = goal.y - y
    ang = atan2(curr_y, curr_x)
    angle = ang - theta
    # print(angle)

def goal_seek():
    global twist, bot_vel, angle, front, threshold1, wall_flag, hit_pose, bot_vel, state, hit_count, bot_pose, goal

    x =  abs(bot_pose.x - goal.x)
    y = abs(bot_pose.y - goal.y)


    if x < 0.5 and y < 0.5:  #check if bot is at terminal
        twist.linear.x = 0
        twist.angular.z = 0
        state = 3

    elif front < threshold1:   #obstacle detected switch to wall follow
        hit_count += 1
        twist.linear.x = 0
        twist.angular.z = 0
        hit_pose = bot_pose

        wall_flag = True
        state = 2

    else:
        twist.linear.x = 1
        twist.angular.z = 0

    bot_vel.publish(twist)
    return



def Laser_callback(data):
    #dividing sensor scan into zones
    global left, top_left, front, top_right, right
    left_b = 71
    fl_b = 173
    frnt_b = 186
    fr_b = 288
    right_b = 359

    global regions


    regions = [
        min( min(data.ranges[0:left_b]) , 10),
        min(min(data.ranges[left_b+1:fl_b]), 10),
        min(min(data.ranges[fl_b+1:frnt_b]), 10),
        min(min(data.ranges[frnt_b+1:fr_b]), 10),
        min(min(data.ranges[fr_b+1:right_b]), 10)
    ]
    left = regions[4]
    top_left = regions[3]
    front = regions[2]
    top_right = regions[1]
    right = regions[0]



def bugg2():
    rospy.Subscriber('/base_scan', LaserScan, Laser_callback)
    rospy.Subscriber('/odom', Odometry, bot_orientation_yaw)
    global st, state, bot_vel, bot_pose, goal, hit_count, angle, distance_moved
    bot_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    old_state = 1
    # print('0')if st[state] != 'TERMINAL':
    while state != 3 and not rospy.is_shutdown() :
        # print("State: " + str(state) + " " + "angle: " + str(angle) + " " + "Hit Count: " + str(hit_count) + " " + "distance_moved: " + str(distance_moved) + " ")
        if st[old_state] != st[state]:
            print(st[state])
        # print(state, '   ', hit_count)
        old_state = state
        if state == 0:
            # print('GOAL_SEEK')
            rotate_bot_towards_goal()

        elif state == 2:
            # print('WALL_FOLLOW')
            wall_follow()

        elif state == 1:
            # print('GOAL_SEEK')
            goal_seek()







    if state == 3:
        print(st[state])



if __name__ == '__main__':

    rospy.init_node('bug2', anonymous=True)
    bugg2()
    rospy.spin()
