//om


// THis file works and has been tested
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

std::string robot = "turtlebot3_waffle_pi";
std::string obs = "unit_cylinder";
void model_function (const gazebo_msgs::ModelStates::ConstPtr & ptr){
	
	int obs_index{0}, robot_index{0};
	for (int i=0; i<(ptr->name).size(); i++){
	std::cout << (ptr->name)[i] << "\t";
	
	if((ptr->name)[i]==robot)
		robot_index = i;
	else if((ptr->name)[i]==obs)
		obs_index = i;
	}

	
	
	auto robot_data{(ptr->pose)[robot_index]};
	auto obs_data {(ptr->pose)[obs_index]};

	std::cout << "Robot current location : \n";
	std::cout << robot_data.position.x << " " << robot_data.position.y << " " << robot_data.position.z << "\n";

	std::cout << "\n";
	ros::Duration(5).sleep();
}

int main(int argc, char** argv){

	ros::init(argc, argv, "understanding_model_states");

	ros::NodeHandle nh;

	ros::Subscriber sh = nh.subscribe("/gazebo/model_states", 10, model_function);

	ros::spin();

	return 0;

}
