Final waypoints is generated using base_waypoints, traffic_waypoint, obstacle_waypoint and current_pose.

Among which, obstacle_waypoint is generated using base_waypoints, current_pose and image_color and traffic_waypoint is generated using base_waypoints, image_color, current_pose.

The final waypoints are sent to DBW(drive by wire: meaning the throttle, brake and steering have electronic control) node which sends control messages to car/simulator. The DBW node also gets updates about car/vehicle status.
