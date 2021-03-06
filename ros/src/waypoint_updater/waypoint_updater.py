#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math
import numpy as np

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
RATE = 30  # Rate of updating waypoints (Waypoint follower frequency)
BUFFER_TO_STOPLINE = 2
MAX_DECEL = .5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # equation for hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        # if closest is behind vehicle
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_final_waypoints()
        self.final_waypoints_pub.publish(final_lane)

    def generate_final_waypoints(self):
        lane = Lane()
        lane.header = self.base_waypoints.header

        closest_idx = self.get_closest_waypoint_idx()
        furthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:furthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= furthest_idx):
            lane.waypoints = base_waypoints
        else:
            # If need to stop
            waypoints_need_update = self.get_waypoints_need_update(base_waypoints, closest_idx)
            lane.waypoints = self.decelerate_waypoints(waypoints_need_update, closest_idx)

        return lane

    def get_waypoints_need_update(self, waypoints, closest_idx):
        # assume traffic is updated quite frequent, so stopline_wp_idx > closest_idx
        num_waypoints_need_update = self.stopline_wp_idx - closest_idx + BUFFER_TO_STOPLINE
        waypoints_need_update = waypoints[:num_waypoints_need_update]
        return waypoints_need_update

    def decelerate_waypoints(self, waypoints, closest_idx):
        # Make a new list to store final waypoints (since base waypoints are published only once)
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            # make stop action before the stopline
            stop_idx = max(self.stopline_wp_idx - closest_idx - BUFFER_TO_STOPLINE, 0)
            dist = self.distance(waypoints, i, stop_idx)
            # OPTION: smooth deceleration
            vel = math.sqrt(2 * MAX_DECEL * dist)

            if vel < 1.:
                vel = 0.
            # do not go over vel limit
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # store the base waypoints
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [
                [waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                for waypoint in waypoints.waypoints
            ]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        """probably do not want to overwrite the base waypoints"""
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
