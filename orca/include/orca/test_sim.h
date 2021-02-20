
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

#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

	ros::Subscriber agent_state_sh_;			/*Class Member subscriber for obtaining agent data*/
	
	
	ros::Subscriber _occupancyGridSubscriber;   /* Nagarjun's code*/

	
	
	
	void modelStatesCallbackFunction_(const gazebo_msgs::ModelStates::ConstPtr& );

	void staticObstaclesCallBackFunction_(const sensor_msgs::LaserScanConstPtr& );

	void agentStateCallbackFunction_(const orca_msgs::AgentState::ConstPtr& );

	// /****************************/

	// // Nagarjun code:

	void _occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid);

	Eigen::Vector3d convertVec2ToVec3(const RVO::Vector2& position);

	cv::Mat _occupancyGridImage;

	bool _b_isOccupancyGridImageInitialized{false};

	Eigen::Vector3d OccupancyGridPosition;

	

	// /*******************************/


    double T_robot_world[3][3];

    tf::TransformListener listener1_; 		// for base_footprint->map
    
	tf::StampedTransform transform1_; 		// for base_footprint->map

	tf::TransformListener listener2_; 		// for base_scan->map
    
	tf::StampedTransform transform2_; 		// for base_scan->map
	
	// ------------------------------------

	//* Defining the variables for ORCA 
	double robotRadius_{22.0f};
	
	// double safetyBuffer_{188.0f};
	double safetyBuffer_{50.0f};
	
	double netRobotRadius_{robotRadius_ + safetyBuffer_};
	
	float maxSpeed_{};
	
	RVO::Vector2 velocity;

	double timeStep_{0.12f};
	
	double timeHorizonAgent_{5.0f};
	
	double timeHorizonObstacle_{800.00f};

	double vPrefScalingFactor_{5.0};
	
	std::string robotName_ {"turtlebot3_waffle"};
	
	RVO::Vector2 robotStart_{RVO::Vector2(-600, 0.00f)};
	 
	RVO::Vector2 robotGoal_{RVO::Vector2(500.0f,0.00f)};
	// RVO::Vector2 robotGoal_{RVO::Vector2(-150.00f, 000.00f)};

	// Obstacle Data
	
	std::string obsName_ {"unit_cylinder"};
	
	double centerX_{0.00f}; 
	
	double centerY_{0.00f};
	
	// int angularDivisions_{360};
	
	// double radiusObstacle_{120.0f}; // in cm

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
	
	


	//* Helper member functions and variables
	bool b_obstacleInitialized_{false};

	ros::Time startTimeSimulation_;
	
	ros::Time endTimeSimulation_;

	RVO::Vector2 robotCurrentPosition_{robotStart_};		
	
	std::vector<RVO::Vector2> obstData_;
	
	std::vector<RVO::Vector2> obstDataFinal_;
	
	void velocityPublisher_(const geometry_msgs::Twist&); 							// Alters velocity to ensure drift proof motion & finally publishes it

	/********************************/

	// Nagarajun's

	// void forwardSimulationPosition(const geometry_msgs::Twist&, double timeHorizonObstacle_, float timeStep_, const nav_msgs::OccupancyGrid::ConstPtr& grid);
	void forwardSimulationPosition(const geometry_msgs::Twist&, double timeHorizonObstacle_, float timeStep_);


	// void CheckInObstacleSpace(RVO::Vector2 postion_t1, const nav_msgs::OccupancyGrid::ConstPtr& grid );
	void CheckInObstacleSpace(RVO::Vector2 postion_t1);

	Eigen::Vector3d _convertFromWorldFrame(Eigen::Vector3d pointInWorldFrame);

	/**
	 * //brief      initialize the transformation matrix from Gazebo world
     *             to opencv image frame
	 */
    void _initializeTransformationMatrix();

	void displayGlobalGrid();

	std::string _windowName{"Global Occupancy Grid"};

	// void TurtleBotCurrentPosition(const gazebo_msgs::ModelStates::ConstPtr&);

    
	/****************************/

	tf::Vector3 transformVelocity_(tf::Matrix3x3& mat, tf::Vector3& vel);			// Tranforms vel wrt the transformation mat

    geometry_msgs::Twist transformVelToRobotFrame_(geometry_msgs::Twist&);  		// Vel transformation [map -> base_footprint]

	void dispTransformationMat_(tf::Matrix3x3& mat);								// Prints transfomation matrix

	void printObstacleVector_ (std::vector<RVO::Vector2>& );						// Prints obstacle vector

	double norm2_(RVO::Vector2& currPoint, RVO::Vector2& prevPoint);     			// Computes distance between two scan points squared

	double minScan_{10.0f};	

	// *****************************
	/* Nagarjun's Code*/
	
	RVO::Vector2 agentPosition_;
	
	
	// transformation matrix from (Gazebo's) world  frame to OpenCv image frame
    Eigen::Matrix3d _T_World_Image; 

	// costmap properties exhibited by RVIZ
    int _costmapSize{100};
    double _costmapOriginX{0.0f};
    double _costmapOriginY{0.0f};
    double _costmapResolution{0.0f};

	// *****************************

	public:


	Test_Sim();

	Test_Sim(ros::NodeHandle & nh);

	~Test_Sim();

	RVO::Vector2 getRobotCurrentPosition();					// returns current robot position as RVO::Vector2
	
	RVO::Vector2 transformPointToWorldFrame(tf::Matrix3x3& , const tf::Vector3& , const tf::Vector3& );
	

	std::vector<RVO::Vector2> left_wall {
		RVO::Vector2(710,610), 
		RVO::Vector2(-620,610)
	};

	std::vector<RVO::Vector2> bottom_wall {
		RVO::Vector2(-620,610),
		RVO::Vector2(-620,-580)
	};

	std::vector<RVO::Vector2> right_wall{
		RVO::Vector2(-620,-580), 
		RVO::Vector2(710,-580)

	};

	std::vector<RVO::Vector2> top_wall{
		RVO::Vector2(710,-580),
		RVO::Vector2(710,610)
	};

	std::vector<RVO::Vector2> box_top{
		RVO::Vector2(510,-340),
		RVO::Vector2(410,-340),
		RVO::Vector2(410,-440),
		RVO::Vector2(510,-440)
	};

	std::vector<RVO::Vector2> box_center{
		RVO::Vector2(70,-330),
		RVO::Vector2(-30,-330),
		RVO::Vector2(-30,-430),
		RVO::Vector2(70,-430)
	};

	std::vector<RVO::Vector2> box_bottom{
		RVO::Vector2(-315,-345),
		RVO::Vector2(-415,-345),
		RVO::Vector2(-415,-445),
		RVO::Vector2(-315,-445)
	};

	std::vector<RVO::Vector2> center_circle;


	// hospital objects
	
	std::vector<RVO::Vector2> room1_left{  // door not included
		RVO::Vector2(-300,-1100),
		RVO::Vector2(-300,-300),
		RVO::Vector2(-900,-300),
		RVO::Vector2(-900,-900),
		RVO::Vector2(-700,-900),
		RVO::Vector2(-600,-1000),
		RVO::Vector2(-600,-1100),
	};

	std::vector<RVO::Vector2> room2_left{  // door not included
		RVO::Vector2(-300,0.0),
		RVO::Vector2(-300,600),
		RVO::Vector2(-500,600),
		RVO::Vector2(-600,700),
		RVO::Vector2(-600,800),
		RVO::Vector2(-900,800),
		RVO::Vector2(-900,0.0),
	};

	std::vector<RVO::Vector2> room1_right{
		RVO::Vector2(0.0,-1100),
		RVO::Vector2(600,-1100),
		RVO::Vector2(600,-550),
		RVO::Vector2(0.0,-550)
	};

	std::vector<RVO::Vector2> room2_right{
		RVO::Vector2(0.0,-550),
		RVO::Vector2(600,-550),
		RVO::Vector2(600,0.0),
		RVO::Vector2(0.0,0.0)
	};

	std::vector<RVO::Vector2> room3_right{
		RVO::Vector2(0.0, 00),
		RVO::Vector2(600,0.0),
		RVO::Vector2(600,800),
		RVO::Vector2(0.0,800)
	};

	
	
		// objects
	std::vector<RVO::Vector2> tram{
		RVO::Vector2(-600,-100),
		RVO::Vector2(-400,-100),
		RVO::Vector2(-400,0.0),
		RVO::Vector2(-600,0.0)
	};

	std::vector<RVO::Vector2> tram2{
		RVO::Vector2(0.0,200),
		RVO::Vector2(0.0,340),
		RVO::Vector2(-50,340),
		RVO::Vector2(-50,200)
	};

	std::vector<RVO::Vector2> seat1{
		RVO::Vector2(-300,-1000),
		RVO::Vector2(-200,-1000),
		RVO::Vector2(-200,-800),
		RVO::Vector2(-300,-800)
	};

	std::vector<RVO::Vector2> seat2{
		RVO::Vector2(-100,-500),
		RVO::Vector2(-100,-700),
		RVO::Vector2(0.0, -700),
		RVO::Vector2(0,-500.0)
	};

	std::vector<RVO::Vector2> seat3{
		RVO::Vector2(-300,50),
		RVO::Vector2(-200,50),
		RVO::Vector2(-200,210),
		RVO::Vector2(-300,210)
	};

	std::vector<RVO::Vector2> seat4{
		RVO::Vector2(-100,400),
		RVO::Vector2(-0.0,400),
		RVO::Vector2(-0.0,580),
		RVO::Vector2(-100,580)
	};

	std::vector<RVO::Vector2> circularBric;

	// std::vector<RVO::Vector2> d{
	// 	RVO::Vector2(-0.351529,-3.88318), 
	// 	RVO::Vector2(-0.325635,-4.29018)
	// };

	// std::vector<RVO::Vector2> d2{
	// 	RVO::Vector2(-0.325188,-3.34342), 
	// 	RVO::Vector2(-0.325762,-3.39504)
	// };

	// std::vector<RVO::Vector2> e2{
	// 	RVO::Vector2(4.07345,-3.70187), 
	// 	RVO::Vector2(4.06114,-3.80345)
	// };


};



#endif
