Final waypoints is generated using base_waypoints, traffic_waypoint, obstacle_waypoint and current_pose.

Among which, obstacle_waypoint is generated using base_waypoints, current_pose and image_color and traffic_waypoint is generated using base_waypoints, image_color, current_pose.

Be mindful about the abrupt signal/data change (light state, car velocity etc.), consistency check should always be performed before taking any action.

The final waypoints are sent to DBW(drive by wire: meaning the throttle, brake and steering have electronic control) node which sends control messages to car/simulator. The DBW node also gets updates about car/vehicle status.

## Useful command to run while running the program

rosparam get /waypoint_loader/velocity
rosparam get /traffic_light_config
Rosparam get /waypoint_loader/path

rostopic echo -c /vehicle/traffic_lights
Rostopic echo -c /traffic_waypoint
rostopic echo -c /base_waypoints
