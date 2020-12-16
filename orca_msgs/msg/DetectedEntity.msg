#This is to store the detected entity data namely the position, velocity and radius data.
# Since this would be later by agent.msg & obstacle.msg in the odom_frame. Transform it from base_link to _odom before publishing the data

Vector2 pos
Vector2 vel
float64 radius
