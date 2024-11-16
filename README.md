### Aruco Marker based Docking Differential Robot

## Nodes:

# aruco_detect
Reads data from camera, identifies aruco markers, receives marker id and selects the appropriate marker to calculate relative location to the bot and sends the location data in Pose2D format. This is done to reduce traffic of multiple poses of all aruco markers being detected.
    
    * Subscribes to "/desired_marker_id" to get what marker id 
    * Subscribes to "/camera/image_raw" to get camera feed
    * Subscribes to "/camera/camera_info" to get camera info

# diff_drive
Contains the controller for the bot. Goal manager and accordingly allots next goal for the bot to go to and accordingly assigns marker id and publishes to get dock station location. 

Transforms all coordinates to global coordinates to keep things simple. Converts marker location to global, also gets current bot location from "/odom". 

Control system deals with a polar coordinate frame so that its easier with a differential drive. The angular velocity, omega, of the bot is controlled according to what direction the bot needs to head to, and the velocity is controlled simply on how far the goal is. Once near the goal, omega control switches to match the required orientation near each dockstation i.e. perpendicular to the aruco marker. 

Omega limiting is used where we reduce omega in high speeds by limiting the radius of curvature


## Build and Run:

The package is developed on ros-humble running on docker. They need to be downloaded and setup according to their respective documentations.

*Copy the model files from* `/rs_diff/src/models` *and paste it into* `/install/rs_diff/share/rs_diff/models` *for the aruco markers to launch after colcon building the package*