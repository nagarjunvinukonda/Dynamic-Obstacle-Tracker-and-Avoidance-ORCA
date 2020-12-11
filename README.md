# Gazebo-Human-and-obstacle-tracker
This package is modified for tracking humans and small moving obstacles for tracking velocity, radius, and position

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
roslaunch obstacle_detector demo.launch
```

$ Launch turtlebot teteoperation
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
```

![obstacle_detect](https://user-images.githubusercontent.com/49041896/101849358-89672880-3b25-11eb-8dc6-33262c6d647f.gif)


