# aruco_broadcaster

Repository of a project that using a config file publish the tf of the arucos.

In order to use the aruco_broadcaster package, the following steps must be followed:

First of all, the aruco parameters must be set in yaml file. Some parameters are necessary:
the camera reference frame w.r.t. are referred the arucos and aruco_frame that is how will be named. A markerList parameters is optional with the #id to use.

Example of Parameters:

- markersFixList : [0, 100, 101, 102, 103, 104, 105,...] #Id's number of all markers that are wanted to display and starts with a 0
- camera_frame: kinect2_rgb_optical_frame
- aruco_frame: aruco_frame

Once all parameters have been set up, the aruco_broadcaster node is ready to be launched.

In order to launch the aruco_broadcaster node, the file prova_aruco must be launched:
```
$ ros2 launch aruco_broadcaster aruco_broadcaster.launch
```


