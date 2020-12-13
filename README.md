# Gazebo-Human-and-obstacle-tracker
This package is modified for tracking humans and small moving obstacles for tracking velocity, radius, and position

For Obstacle detection package we have used following set of parameters for robust human detection and our environment obstacles detection:

![params_final](https://user-images.githubusercontent.com/49041896/102007696-3fcf3700-3cf9-11eb-98cd-343db53bdcca.png)


The following is the result:

![ezgif com-gif-maker (5)](https://user-images.githubusercontent.com/49041896/102007586-75bfeb80-3cf8-11eb-8c87-80c70906ea4a.gif)


# Lauching files:

$ Launch Gazebo file
```
roslaunch turtlebot3_gazebo turtlebot3_env.launch 
```

$ Launch Laser scan spliter to split the Laser scan into front and back scan
```
roslaunch split_scan split_scan.launch 
```

$ Launch obstcale detector package
```
roslaunch obstacle_detector detector.launch
```

$ Launch turtlebot teteoperation
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
```

![obstacle_detect](https://user-images.githubusercontent.com/49041896/101849358-89672880-3b25-11eb-8dc6-33262c6d647f.gif)

# References & Useful resources
* M.Przybyła, “Detection and tracking of 2d geometric obstacles from lrfdata,” in2017 11th International Workshop on Robot Motion and Control(RoMoCo). IEEE, 2017, pp. 135–141.
* https://github.com/tysik/obstacle_detector


