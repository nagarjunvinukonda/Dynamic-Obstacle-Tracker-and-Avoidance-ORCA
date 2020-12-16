#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

tf::Transform T_map_odom;
double t_future = 0;

void sendTransform(const ros::TimerEvent& e)
{
  // Broadcast a tf for base_link to the global frame
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(T_map_odom, ros::Time::now()+ros::Duration(t_future), "map", "odom"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_map_odom");
  ros::NodeHandle handle;

  T_map_odom.setOrigin( tf::Vector3(0,0, 0)); // coinciding {map} frame with the {odom} 
  T_map_odom.setRotation(tf::createQuaternionFromYaw(0));
  
  ros::Duration pubMapOdom_(0.05);
  ros::Timer pubMapOdomTimer_ = handle.createTimer(pubMapOdom_, &sendTransform);

  
  ros::spin();
  printf("\nExiting normally\n");
  return 0;
}
