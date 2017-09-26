#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import copy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	self.lane = None
	self.car_pose = None

        rospy.spin()

    def pose_cb(self, msg):
	self.car_pose = msg
	x = self.car_pose.pose.position.x
	y = self.car_pose.pose.position.y
	rospy.logwarn("vehicle position (x, y): (%s, %s)", x, y)
	rospy.logwarn("vehicle orientation (x, y, z, w): %s", self.car_pose.pose.orientation)

	if self.lane == None:
		return

        self.publish_next_waypoints()


    def waypoints_cb(self, waypoints):
	self.lane = waypoints
      
	x = self.lane.waypoints[0].pose.pose.position.x
	#rospy.logwarn("x-coord of first waypoint position: %s", x)
	waypoints_count = len(self.lane.waypoints)
	#rospy.logwarn("wapoints_count: %s", waypoints_count)

    def publish_next_waypoints(self):
	index = self.find_next_closest_point_index()
	#rospy.logwarn("next closes point index: %s", index)

	lane = Lane()
	lane.header = self.lane.header
	lane.waypoints = self.lane.waypoints[index:index+LOOKAHEAD_WPS]
	#rospy.logwarn("next 200 way points count: %s", len(lane.waypoints))

	x = lane.waypoints[0].pose.pose.position.x
	y = lane.waypoints[0].pose.pose.position.y
	rospy.logwarn("next 200 waypoints' first point (x, y): (%s, %s)", x, y)
	rospy.logwarn("next 200 waypoints' first point orientation (x, y, z, w): %s", lane.waypoints[0].pose.pose.orientation)

	#publish data
	self.final_waypoints_pub.publish(lane)
	
    def get_waypoint_position(self, waypoint):
	return waypoint.pose.pose.position

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def euclidean_distance(self, a, b):
	return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z - b.z)**2)

    def orientation_distance(self, a, b):
	yaw_a = math.atan2(2.0*(a.y*a.z + a.w*a.x), a.w*a.w - a.x*a.x - a.y*a.y + a.z*a.z)
	yaw_b = math.atan2(2.0*(b.y*b.z + b.w*b.x), b.w*b.w - b.x*b.x - b.y*b.y + b.z*b.z)
	return math.sqrt((yaw_a-yaw_b)**2)
	#return math.sqrt((a.z - b.z)**2 + (a.w - b.w)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist



    def find_next_closest_point_index(self):
	index = -1
	min_distance = 999999
	min_orientation_distance = 999999
	car_position = self.car_pose.pose.position
	car_orientation = self.car_pose.pose.orientation

	for i in range(len(self.lane.waypoints)):
		wp_position = self.lane.waypoints[i].pose.pose.position
		wp_orientation = self.lane.waypoints[i].pose.pose.orientation
		
		#check if point is behind current vehicle position
		#if wp_position.y < car_position.y:
		#	continue;

		distance = self.euclidean_distance(wp_position, car_position)
		orientation_distance = self.orientation_distance(car_orientation, wp_orientation)
		if (distance < min_distance):
			min_distance = distance
			min_orientation_distance = orientation_distance
			index = i

	return index


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
