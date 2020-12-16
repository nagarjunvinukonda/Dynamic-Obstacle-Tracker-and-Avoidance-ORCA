
#ifndef TEST_SIMULATOR_1
#define TEST_SIMULATOR_1

#pragma once

#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#ifndef RVO_SEED_RANDOM_NUMBER_GENERATOR
#define RVO_SEED_RANDOM_NUMBER_GENERATOR 1
#endif


#include <cmath>
#include <cstdlib>
#include <vector>
#include <fstream>
// #include <opencv2/opencv.hpp>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
#include <ctime>
#endif

#if _OPENMP
#include <omp.h>
#endif



/* Including the ros headers*/
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "orca_msgs/AgentState.h"
#include "orca_msgs/DetectedEntity.h"


/* Including the RVO headers*/
#include "orca/Definitions.h"
#include "orca/RVO.h"


/*Set CV FLAG*/
#define CV_FLAG true




class Test_Sim{
	private:
	/* Saving into LOG Files*/
	std::ofstream log;

	
	// -------------- ROS VARIABLES --------------------
	ros::NodeHandle nh_;	 				    /*Class Node handle*/
	
	ros::Publisher velocity_pb_;		 		/*Class Member publisher for Velocity*/
	
	ros::Subscriber model_state_sh_;		    /*Class Member subscriber for getting Gazebo Model States*/

    ros::Publisher my_robot_state_pb_;		 	/*Class Member publisher for publishing robot position*/

	ros::Subscriber static_obstacle_sh_;     	/* Class Member subscriber for obtaining static obstacle data*/

	void modelStatesCallbackFunction_(const gazebo_msgs::ModelStates::ConstPtr& );

	void staticObstaclesCallBackFunction_(const sensor_msgs::LaserScanConstPtr& );

    double T_robot_world[3][3];

    tf::TransformListener listener1_; 		// for base_footprint->map
    
	tf::StampedTransform transform1_; 		// for base_footprint->map

	tf::TransformListener listener2_; 		// for base_scan->map
    
	tf::StampedTransform transform2_; 		// for base_scan->map
	
	// ------------------------------------

	//* Defining the variables for ORCA 
	double robotRadius_{22.0f};
	
	double safetyBuffer_{188.0f};
	
	double netRobotRadius_{robotRadius_ + safetyBuffer_};
	
	float maxSpeed_{};
	
	RVO::Vector2 velocity;

	
	double timeHorizonAgent_{5.0f};
	
	double timeHorizonObstacle_{1200.00f};

	
	std::string robotName_ {"turtlebot3_waffle"};
	
	RVO::Vector2 robotStart_{RVO::Vector2(-500.00f, 0.00f)};
	
	RVO::Vector2 robotGoal_{RVO::Vector2(600.00f, 0.00f)};

	// Obstacle Data
	
	std::string obsName_ {"unit_cylinder"};
	
	double centerX_{0.00f}; 
	
	double centerY_{0.00f};
	
	int angularDivisions_{360};
	
	double radiusObstacle_{120.0f}; // in cm

	float neighborDist_{};
	
	size_t maxNeighbors_{};



	// ------------------------------------
	//* ORCA Env Setup  

	
	RVO::RVOSimulator *sim;				// Creating a simulator instance 

	std::vector<RVO::Vector2> goals_;	// Vector to keep track of goals of each agent
	
	void setupScenario_();				// Setting up the scenario

	void setupAgent_();					// Setting up agents 
	
	void setupObstacle_();			 	// Setting up obstacle points

	void updateVisualization_();		// Method Update visualization (Increment Time Step dT)
	
	void setPreferredVelocities_();		// Setting up v_pref

	bool runORCA_(); 					// Execute the ORCA Loop
	
	bool reachedGoal_();				// Check if agent reached goal
	
	
	// ------------------------------------

	//* Helper member functions and variables
	bool b_obstacleInitialized_{false};

	ros::Time startTimeSimulation_;
	
	ros::Time endTimeSimulation_;

	RVO::Vector2 robotCurrentPosition_{robotStart_};		
	
	std::vector<RVO::Vector2> obstData_;
	
	std::vector<RVO::Vector2> obstDataFinal_;
	
	void velocityPublisher_(const geometry_msgs::Twist& ); 							// Alters velocity to ensure drift proof motion & finally publishes it

    tf::Vector3 transformVelocity_(tf::Matrix3x3& mat, tf::Vector3& vel);			// Tranforms vel wrt the transformation mat

    geometry_msgs::Twist transformVelToRobotFrame_(geometry_msgs::Twist&);  		// Vel transformation [map -> base_footprint]

	void dispTransformationMat_(tf::Matrix3x3& mat);								// Prints transfomation matrix

	void printObstacleVector_ (std::vector<RVO::Vector2>& );						// Prints obstacle vector

	double norm2_(RVO::Vector2& currPoint, RVO::Vector2& prevPoint);     			// Computes distance between two scan points squared

		
	public:


	Test_Sim();

	Test_Sim(ros::NodeHandle & nh);

	~Test_Sim();

	RVO::Vector2 getRobotCurrentPosition();					// returns current robot position as RVO::Vector2
	
	RVO::Vector2 transformPointToWorldFrame(tf::Matrix3x3& , const tf::Vector3& , const tf::Vector3& );
	
};



#endif
