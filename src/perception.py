import rospy
import random
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
from visualization_msgs.msg import Marker
import numpy as np


def callback(data):
    pub_rviz = rospy.Publisher("/points",Marker,queue_size=10)
    sensor_data = data.ranges
    counter = 0
    angle_factor = data.angle_increment
    ang_min = data.angle_min
    coordinates = []
    for r in sensor_data:
        counter += 1
        if r < 3:
            x_coordinate = r * np.cos(ang_min + angle_factor * counter)  #convert coordinates from polar to cartesian
            y_coordinate = r * np.sin(ang_min + angle_factor * counter)
            coordinates.append((x_coordinate,y_coordinate))


    best_random_point = []
    line_coeff = []
    points = coordinates
    best_inlier_count = 0
    best_random_point = []
    for k in range(0,30):
        # points = coordinates
        inliers = []
        outliers = []
        best_inlier = []
        best_line_points = []

        random_points = random.sample(points, 2)
        line = construct_line(random_points)
        line_coeff = line
        for c in coordinates:
            choice = random.choice(points)
            distance = calc_distance(choice, line_coeff)
            if distance < 0.7:
                inliers.append(choice)
            else:
                outliers.append(choice)

        current_inlier_count = len(inliers)
            # print(len(inliers)," ",len(outliers))
        if current_inlier_count > best_inlier_count:
            best_inlier = inliers
            best_inlier_count = current_inlier_count
            best_random_point = random_points

            # for i in best_inlier:
            #     for p in  points:
            #         if i == p:
            #             points.remove(i)

        # if len(points) < 3:
        #     break





    line_marker=Marker()
    line_marker.header.frame_id = "base_laser_link"
    line_marker.type = Marker.LINE_STRIP
    line_marker.color.g = 1
    line_marker.color.a = 1
    line_marker.type = Marker.ADD
    line_marker.lifetime = rospy.Duration(0)
    line_marker.scale.x = 0.05
    best_random_point1 = best_random_point[0]
    best_random_point2 = best_random_point[1]
    p1 = Point()
    p1.x = best_random_point1[0]
    p1.y = best_random_point1[1]
    p1.z = 0
    p2 = Point()
    p2.x = best_random_point2[0]
    p2.y = best_random_point2[1]
    p2.z = 0
    line_marker.points.append(p1)
    line_marker.points.append(p2)
    pub_rviz.publish(line_marker)

    return




def construct_line(ran_points):
	d = (ran_points[1][1] - ran_points[0][1])/(ran_points[1][0] - ran_points[0][0])
	a = d
	b = -1
	c = ran_points[0][1] - (ran_points[0][0] * d)
	line = [a,b,c]
	#print ("line -->",line)
	return line

def calc_distance(point1 , line_coeff):
	nr = (line_coeff[0] * point1[0]) + (line_coeff[1]*point1[1]) + line_coeff[2]
	nr = math.fabs(nr)
	dr = math.sqrt(line_coeff[0]*line_coeff[0] + line_coeff[1]*line_coeff[1])
	return (nr/dr)



    # print(coordinates,'\n')



if __name__=='__main__':

    try:
        rospy.init_node('perception',anonymous=True)
        rospy.Subscriber('base_scan',LaserScan,callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
