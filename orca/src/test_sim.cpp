/*
 * This file provides an API to interact with the RVO2 (ORCA) Library
 * which has been modified to work in ROS
 * 
 */

#include "orca/test_sim.h"
// #include "orca/GlobalPlanner.h"



/*
 * Constructor 
 * Input : NodeHandle
 * 
 * brief : initializes member variables, subscribers and publishers
 * 		   and finally runs ORCA
 * 
 */ 
Test_Sim::Test_Sim(ros::NodeHandle& nh) : nh_(nh){
	
	
	std::cout << "Test_Sim\n";
	log = std::ofstream("/home/kshah/ros_ws/orca_ws/src/orca/logs/data_gazebo.txt", std::ofstream::out | std::ofstream::trunc);
	log << "Test_Sim Constructor \n";

	// static obstacle subscriber
	static_obstacle_sh_ = nh_.subscribe("/scan" , 1, &Test_Sim::staticObstaclesCallBackFunction_,this);
	

	sim = new RVO::RVOSimulator();
	

	// wait till obstacle info from surrounding is obtained
	while(!b_obstacleInitialized_){
		ROS_INFO("Waiting to Receive Laser Scan CallBacks");
		ros::Duration(1.0).sleep();
		ros::spinOnce();
	}


	// publish velocity
	velocity_pb_ = nh_.advertise <geometry_msgs :: Twist> ( "cmd_vel" , 10);


	// obtain model state for the robot (pose)info from gazebo 
	model_state_sh_ = nh_.subscribe("/gazebo/model_states", 1, &Test_Sim::modelStatesCallbackFunction_, this);

	_occupancyGridSubscriber = nh_.subscribe("/move_base/global_costmap/costmap", 1,  &Test_Sim::_occupancyGridCallback, this);

	// ros::Subscriber _odomSub = nh_.subscribe("/odom", 1, &GlobalPlanner::_odomCallback, this);


	// obtain agent state information published (ideally by human detection node)
	agent_state_sh_ = nh_.subscribe("/agents",1, &Test_Sim::agentStateCallbackFunction_, this);

	// publishes robot (pose) info needed by Agent.cpp
	my_robot_state_pb_ = nh_.advertise<orca_msgs::AgentState>("/my_robot/modelStates",1);

	
	setupScenario_();
	
	startTimeSimulation_ = ros::Time::now();
	runORCA_();	
}






/*
 * Destructor  
 * 
 * brief : Stop robot  & return resources
 */
Test_Sim::~Test_Sim(){
	
	std::cout << "Stopping the robot \n";

	int count{10};
	while(--count>0 && ros::ok()){
		geometry_msgs::Twist zeroVelocity;
		velocity_pb_.publish(zeroVelocity);
		ros::Duration(0.01).sleep();
	}

	delete sim;

	endTimeSimulation_ = ros::Time::now();

	std::cout << "Start time = " << startTimeSimulation_.toSec() << "\n";
	std::cout << "End time = " << endTimeSimulation_.toSec() << "\n";
	std::cout << "minScan_ = " << minScan_ << "\n";
	std::cout << "Total Time to completion = " << endTimeSimulation_.toSec() - startTimeSimulation_.toSec() << " seconds \n";
	std::cout << "XXXXXXX END OF ORCA XXXXXXXXXX "  << "\n";

}






/*
 * setupScenario_()
 * Input : -
 * Output : void
 * 
 * brief :  Sets up time-step and agent
 */
void Test_Sim::setupScenario_()
{

	// std::cout << "setupScenario_\n";

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
	std::srand(static_cast<unsigned int>(std::time(NULL)));
#endif

	/* Specify the global time step of the simulation. */
	sim->setTimeStep(timeStep_);

	/* Setup agent attributes*/
	setupAgent_();

	
}






/*
 * setupAgent_()
 * Input : -
 * Output : void
 * 	
 *  brief : setup agent ORCA parameters  , add start 
 * 			and goal for robot
 */
void Test_Sim::setupAgent_(){

	// std::cout << "setupAgent_\n";

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, timeHorizonAgent_, timeHorizonObstacle_, netRobotRadius_, timeStep_ * vPrefScalingFactor_ * 100.0f);


	// Adding Robot as an agent
	sim->addAgent(robotStart_);
	goals_.push_back(robotGoal_);

}






/*
 * setupObstacle_() 
 * Input : -
 * Output : void
 * 
 * brief : Processes the obstacles obtained from laser 
 * 		   scan. Obstacle points are in Odom Frame
 */
void Test_Sim::setupObstacle_(){

	// std::cout << "setupObstacle_\n";

	auto setupObstacleStartTime = ros::Time::now();
	

	// open spaces

	sim->addObstacle(left_wall);
	sim->addObstacle(bottom_wall);
	sim->addObstacle(right_wall);
	sim->addObstacle(top_wall);

	sim->addObstacle(box_top);
	sim->addObstacle(box_center);
	sim->addObstacle(box_bottom);

	for (int i=0; i<360; i++){
		center_circle.push_back(RVO::Vector2(-100 * sin(i*M_PI/180.0f), 100 * cos(i*M_PI/180.f) ));
	}
	sim->addObstacle(center_circle);


	// hospital

	// sim->addObstacle(room1_left);
	// sim->addObstacle(room2_left);

	// sim->addObstacle(room1_right);
	// sim->addObstacle(room2_right);
	// sim->addObstacle(room3_right);

	// sim->addObstacle(seat1);
	// sim->addObstacle(seat2);
	// sim->addObstacle(seat3);
	// sim->addObstacle(seat4);

	// sim->addObstacle(tram);
	// sim->addObstacle(tram2);
	

	// for (int i=0; i<360; i++){
	// 	circularBric.push_back(RVO::Vector2(-300 -50 * sin(i*M_PI/180.0f),-250 + 50 * cos(i*M_PI/180.f) ));
	// }
	// sim->addObstacle(circularBric);

	/* Process the obstacles so that they are accounted for in the simulation. */
	sim->processObstacles();

	
	auto setupObstacleEndTime = ros::Time::now();

	// std::cout << "Time to execute setupObstacle : " << (setupObstacleEndTime-setupObstacleStartTime).toSec() <<"\n";
	// log << "Time to execute setupObstacle : " << (setupObstacleEndTime-setupObstacleStartTime).toSec() <<"\n";
}







#if RVO_OUTPUT_TIME_AND_POSITIONS
void Test_Sim::updateVisualization_()
{
	// std::cout << "updateVisualization_\n";
	// std::cout << "updateVisualization \n";
	log << "updateVisualization \n";

	/* Output the current global time. */
	// std::cout << "GlobalTime: " << sim->getGlobalTime() << "\n";
	//log	  << "GlobalTime: " << sim->getGlobalTime() << "\n";

	
	//std::cout << "Num of agents = " << sim->getNumAgents() << "\n";
	/* Output the current position of all the agents. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		RVO::Vector2 agent_position = sim->getAgentPosition(i);
		// std::cout << " " << agent_position << "\n";
	}

	std::cout << std::endl;
}
#endif






/*
 * setPreferredVelocities_()
 * Input : -
 * Output : void
 * 
 * brief : Set the preferred velocity to be a vector of unit 
 * 		   magnitude (speed) in the direction of the goal.
 */
void Test_Sim::setPreferredVelocities_()
{
	// std::cout << "setPreferredVelocities_\n";
	
#ifdef _OPENMP
#pragma omp parallel for
#endif

	for (int i = 0; i <static_cast<int>(sim->getNumAgents()); ++i) {
		

		// get goal direction as position vector from current location to goal
		RVO::Vector2 goalVector = goals_[i] - sim->getAgentPosition(i);


		// converted it into a unit vector of norm2 more than 1
		if (RVO::absSq(goalVector) > 1.0f) 
			goalVector = RVO::normalize(goalVector) * vPrefScalingFactor_;
		 

		sim->setAgentPrefVelocity(i, goalVector);
		std::cout << "Prfer velocity: "<<goalVector << "\n";

		
		
		
		//Perturb a little to avoid deadlocks due to perfect symmetry.
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		// float dist = std::rand() * 0.0001f / RAND_MAX;
		float dist = std::rand() * 0.0001 / RAND_MAX;
	
		RVO::Vector2 agent_change_in_vel = dist * RVO::Vector2 ( std::cos (angle) , std::sin (angle) );
		

		// add the change in agent velocity to current velocity 
		RVO::Vector2 agent_updated_pref_velocity = sim->getAgentPrefVelocity(i) + agent_change_in_vel;
		std::cout << " change in Velocity : " << agent_change_in_vel << "\n";
		sim->setAgentPrefVelocity(i, agent_updated_pref_velocity);
		std::cout << "After Manipulating Vel : " <<agent_updated_pref_velocity << "\n";
	}
}







/*
 * runORCA_()
 * Input : -
 * Output : void
 * 
 * brief : loop to execute ORCA
 *     
 */
bool Test_Sim::runORCA_(){

	ros::Time currTime;
	ros::Time prevTime;
	std::cout << "Run ORCA\n";
	setupObstacle_();
	// ORCA main loop
	do {

		currTime = ros::Time::now();

		// std::cout << "Delta T : " << (currTime.toSec() - prevTime.toSec()) << "\n";
		// setup obstacles at every step
		// setupObstacle_();


		// update pref velocities of agents
		setPreferredVelocities_();


		// transform from /map frame to /base_footprint
		try{ 

			listener1_.waitForTransform("/base_footprint", "/odom", 
			ros::Time(0), ros::Duration(10.0));
			
			listener1_.lookupTransform("/base_footprint", "/odom",
			ros::Time(0), transform1_);

		}
		catch (tf::TransformException &ex) {

			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}


		// computes new agent velocities
		sim->doStep();


		// newly computed agent velocities returned are RVO::Vector2
		RVO::Vector2 agentVelocityInWorld_RVO = sim->getAgentVelocity(0);


		/*Important*/
		//agent_pref_velocities are in the global frame and need to converted into the frame of the robot

		geometry_msgs::Twist agentVelInWorld;
		agentVelInWorld.linear.x = agentVelocityInWorld_RVO.x();
		agentVelInWorld.linear.y = agentVelocityInWorld_RVO.y();


		geometry_msgs::Twist agentVelInRobotFrame = transformVelToRobotFrame_(agentVelInWorld);
		// std::cout << "Robot Altered Velocity:  " << agentVelInRobotFrame.linear.x << "   " << agentVelInRobotFrame.linear.y << " " << agentVelInRobotFrame.angular.z << "\n";


		velocityPublisher_(agentVelInRobotFrame);

		ros::spinOnce();

		prevTime = currTime;
	}while (!reachedGoal_() && ros::ok());
}







/*
 * reachedGoal_()
 * Input : -
 * Output : bool
 * 
 * 
 * THIS FUNCTION MIGHT NEED SOME TWEAKING AS WE MIGHT 
 * HAVE TO GIVE ROBOT MULTIPLE GOALS
 */
bool Test_Sim::reachedGoal_()
{
	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		
		double currentDistanceToGoal = RVO::absSq(sim->getAgentPosition(i) - goals_[i]);
		
		bool   didNotReachGoal = ( RVO::absSq(sim->getAgentPosition(i) - goals_[i]) > 20.0f * 20.0f );
		
		if (didNotReachGoal) 
			return false;
	}

	
	std::cout << "Successfully Reached Goal \n";
	// log << "Successfully Reached Goal \n";
	return true;
}






/*
 * getRobotCurrentPosition()
 * Input : -
 * Output : void
 * 
 * brief : Returns the current position of the robot
 *     
 * 
 * 			this function is used in agent.cpp
 * 	   		currently unused
 */
RVO::Vector2 Test_Sim::getRobotCurrentPosition(){
	return robotCurrentPosition_;
}






/*
 * modelStatesCallbackFunction_()
 * Input : const ptr to gazebo_msgs::ModelStates msg
 * Output : void
 * 
 * brief : obtains and publishes robot current pose 
 *         used by Agent.cpp
 */
void Test_Sim::modelStatesCallbackFunction_(const gazebo_msgs::ModelStates::ConstPtr& modelStatePtr){

	std::cout << "modelStatesCallbackFunction_\n";
	int obstacleIndex{0}, robotIndex{0};

	for (int i = 0; i < (modelStatePtr -> name).size(); i++){

		if ((modelStatePtr->name)[i] == robotName_)
			robotIndex = i;

		else if( (modelStatePtr->name)[i]== obsName_ )
			obstacleIndex = i;

	}

	/****************** Nagarjun */

	RVO::Vector2 position(modelStatePtr->pose[robotIndex].position.x,modelStatePtr->pose[robotIndex].position.y);
	agentPosition_=position;

	/*****************************/


	auto robotData { (modelStatePtr->pose)[robotIndex] };
	auto obsData   { (modelStatePtr->pose)[obstacleIndex]   };

	// The data obtained is in meters, multiply by 100 to convert it into centimeters
	robotCurrentPosition_ = RVO::Vector2(robotData.position.x*100.0f,
										 robotData.position.y*100.0f);


	// obtain robot velocity  (robot is agent[0])								  		
	RVO::Vector2 robotVelocity = sim->getAgentVelocity(0);


	// create an AgentState object to store robot_state
	orca_msgs::AgentState robotState;
	orca_msgs::DetectedEntity robotStateEntity;

	robotState.header.stamp = ros::Time::now();
	robotState.agent_ID.push_back(0);
	robotStateEntity.pos.x = robotCurrentPosition_.x();
	robotStateEntity.pos.y = robotCurrentPosition_.y();
	robotStateEntity.vel.x = robotVelocity.x();
	robotStateEntity.vel.y = robotVelocity.y();
	robotStateEntity.radius = netRobotRadius_;

	// pushing the robot current state data object into the Robot State msg
	robotState.data.push_back(robotStateEntity);

	// publish the robotState data
	my_robot_state_pb_.publish(robotState);


	// The duration is kept small to ensure latest robot states are used always
	ros::Duration(0.0001).sleep();
}





/*
 * staticObstaclesCallBackFunction_()
 * Input : const ptr to sensor_msgs::LaserScan
 * Output : void
 * 
 * brief : converts laser scan points from robot 
 * 		   base frame into odom (World) frame as
 * 		   required by the ORCA algorithm
 * 			
 * 		   Here there are two clustering logics cluster logic-1 and logic-2. 
 * 		   Both can be used and are similar methods, but Logic-1 yeilds more data points. 
 * 		   Logic-1 is based on taking distance between two points is less than threshold 
 * 		   then considered as obstacle point.
 * 
 */
void Test_Sim::staticObstaclesCallBackFunction_(const sensor_msgs::LaserScanConstPtr& scans){

	// size_t sizeOfLaserScanArray = scans->ranges.size();
	
	std::cout << "staticObstaclesCallBackFunction_\n";
	// clear obstacle vector (Local as well as from ORCA) 
	// every time on receiving new scan
	obstData_.clear();
	// sim->clearObstacleVector();


	// angle increment between two scans in radians
	double d_angle = scans->angle_increment;
	
	
	// create transformation from /base_scan to /odom
    try{

        listener2_.waitForTransform("/odom", "/base_scan", 
        ros::Time(0), ros::Duration(10.0));
        
        listener2_.lookupTransform("/odom", "/base_scan",
        ros::Time(0), transform2_);

    }
    catch (tf::TransformException &ex) {

        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }


	// transformation matrix (Rotational - 3x3)
    tf::Matrix3x3 mat = transform2_.getBasis();

	// origin of odom frame in base frame 
    tf::Vector3 origin{ transform2_.getOrigin() };


	// store current point & previous point
    RVO::Vector2 pointCurr{0.0f, 0.0f},   pointPrev{0.0f, 0.0f}, pointPrPrev{0.0f, 0.0f}; 
	


	// Allowable error is the 'ratio' distance between two 
	// points divided by the first point's distance from 
	// robot's position in the World
	// double allowError{(0.2f/4.0f) + 0.1f}; 
	double allowError{0.5f}; 
	bool b_startingNewCluster{true};
	// // bool b_afterinfCluster{false};


	// create clusters of neighbouring obstacle points and pass it to ORCA

	// std::cout<<"print obs_scan:"<<"\n";
    // // for(int j=sizeOfLaserScanArray-1; j>=0; j--){
		
	// for(int j=0;j<sizeOfLaserScanArray;j++){

	// 	int i = (j + 180) % sizeOfLaserScanArray;

	// 	if(!isinf(scans->ranges[i])) {

	// 		double rayAngle = i * d_angle;
	// 		double laserRange = scans->ranges[i];

	// 		minScan_ = std::min(minScan_ , laserRange);
			// laser point in x,y in /base_scan frame
	// 		double x = (scans->ranges[i]) * cos(rayAngle);
	// 		double y = (scans->ranges[i]) * sin(rayAngle);

			// current point after transforming into World frame

	// 		pointCurr = transformPointToWorldFrame(mat, origin, tf::Vector3(x,y,0) );

			// std::cout<< pointCurr<<"\n";

		
			// double x1 = (scans->ranges[i-1]) * cos(rayAngle);
			// double y1 = (scans->ranges[i-1]) * sin(rayAngle);

			// pointPrev = transformPointToWorldFrame(mat, origin, tf::Vector3(x1,y1,0) );

			// double x2 = (scans->ranges[i-2]) * cos(rayAngle);
			// double y2 = (scans->ranges[i-2]) * sin(rayAngle);

			// pointPrPrev = transformPointToWorldFrame(mat, origin, tf::Vector3(x2,y2,0) );

			// if (norm2_(pointCurr,pointPrev)<=(norm2_(pointPrPrev,pointPrev) + 10*d_angle /*abs(scans->ranges[i]-scans->ranges[i-1])*/)){

			// 	obstData_.emplace_back(pointCurr);

			// }

			// pointPrPrev =pointPrev;
			// pointPrev = pointCurr;




			// Cluster Logic 1

    //      	if(b_startingNewCluster){ // adding new scan to the cluster

    //      		std::cout << "\n Starting new cluster\n";
    //          	obstData_.emplace_back(pointCurr);
    //        		b_startingNewCluster = false;
    //         }
	// 		else{

	// 			double x1 = (scans->ranges[i-1]) * cos(rayAngle);
	// 			double y1 = (scans->ranges[i-1]) * sin(rayAngle);

	// 			pointPrev = transformPointToWorldFrame(mat, origin, tf::Vector3(x1,y1,0) );

	// 			double x2 = (scans->ranges[i-2]) * cos(rayAngle);
	// 			double y2 = (scans->ranges[i-2]) * sin(rayAngle);

	// 			pointPrPrev = transformPointToWorldFrame(mat, origin, tf::Vector3(x2,y2,0) );

	// 			if (norm2_(pointCurr,pointPrev)<=(norm2_(pointPrPrev,pointPrev) + 10*d_angle +0.3f/*abs(scans->ranges[i]-scans->ranges[i-1])*/)){

	// 				obstData_.emplace_back(pointCurr);

	// 			}
	// 			else{
	// 			std::cout << "Cluster Break"<<"\n";
	// 				if(obstData_.size()>1){ // Pass cluster to ORCA 
	// 					sim->addObstacle(obstData_); //obdtsdst.clear
	// 					printObstacleVector_(obstData_);
	// 				}
	// 				obstData_.clear();
	// 				b_startingNewCluster = true;
	// 			}
	// 			std::cout<<"size:"<<obstData_.size()<<"\n";
	// 		}

	// 		pointPrPrev =pointPrev;
	// 		pointPrev = pointCurr;




			// Cluster Logic 2

    //      	// if(b_startingNewCluster){ // adding new scan to the cluster

    //      	// 	std::cout << "\n Starting new cluster\n";
    //         //  	obstData_.emplace_back(pointCurr);
    //        	// 	b_startingNewCluster = false;
    //         // }
	// 		// else{

	// 		// 	double currentError = norm2_(pointCurr, pointPrev);

	// 		// 	if( currentError < allowError){ // Add point to current cluster

	// 		// 		// std::cout << " Cluster Cont ";
	// 		// 		obstData_.emplace_back(pointCurr);
	// 		// 		}
	// 		// 	else{
	// 		// 	std::cout << "Cluster Break"<<"\n";
	// 		// 		if(obstData_.size()>1){ // Pass cluster to ORCA 
	// 		// 		sim->addObstacle(obstData_); //obdtsdst.clear
	// 		// 		}
	// 		// 		obstData_.clear();
	// 		// 		b_startingNewCluster = true;
	// 		// 	}
	// 		// 	std::cout<<"size:"<<obstData_.size()<<"\n";
	// 		// }

	// 		// pointPrev = pointCurr;




	// 	}
	// 	else {

	// 		std::cout<<"inf Detected"<<"\n";
	// 		// b_afterinfCluster =true;

	// 		if(obstData_.size()>1){
	// 			sim->addObstacle(obstData_);
	// 			printinfObstVector_(obstData_);
	// 			obstData_.clear();
	// 		}

	b_obstacleInitialized_ = true;

	// 		continue;
	// 	}

    // }

	// // std::cout<<"Start cluster"<<"\n";

	// // printObstacleVector_(obstData_);

	// // Pass the last set of point cluster to ORCA
	// if(obstData_.size()>1){
	// 	sim->addObstacle(obstData_);
	// 	obstData_.clear();
	// }

	

}




/*
 * agentStateCallbackFunction_()
 * Input : const ptr to orca_msgs::AgentState
 * Output : void
 * 
 * brief : clears agent vector and fills it up with 
 * 		   new agent info (which in turn is published
 * 		   by human detector node)
 */
void Test_Sim::agentStateCallbackFunction_(const orca_msgs::AgentState::ConstPtr& agentStates){
	
}






/*
 * _occupancyGridCallback()
 * Input : const nav_msgs::OccupancyGrid
 * Output : void
 * 
 * brief : checks if the position at t+1 time lies in the 
 * in the threshold color intensity pixel(inside proxemics)
 * 
 * 
 */

 /*Converts the occupancy callback message into 2d CV::Mat based grid */
void Test_Sim::_occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)  {

    // get grid parameters
    int size = (grid->data).size();


    int numRows = grid->info.height;
    int numCols = grid->info.width;


    // set member variables
    _costmapSize = numRows;
    _costmapOriginX = grid->info.origin.position.x;
    _costmapOriginY = grid->info.origin.position.y;
    _costmapResolution = grid->info.resolution;


    // setup blank image
    _occupancyGridImage = cv::Mat(numRows, numCols, CV_8UC1, cv::Scalar(255));

   

    // fillup the image
    for (int k=0; k<size; k++){
        auto intensity = (grid->data[k]) * 255 / 100;
        _occupancyGridImage.at<uchar>(numCols - 1 - k % numCols , numRows - 1 - k / numCols) = intensity;
    }

    // cv::circle(mapImg, cv::Point(numCols - 1 - 147455 % numCols , numRows - 1 - 147455 / numCols), 2, cv::Scalar(0,0,255), -1);

    // if(_b_resizeGrid)
    //     _resizeGrid();

    _b_isOccupancyGridImageInitialized = true;


    // initialize the transformation matrix from Gazebo's world to opencv image frame
    Test_Sim::_initializeTransformationMatrix();


    // display the image
    displayGlobalGrid();
}




/* Initialize the transformation matrix from Gazebo's world frame to opencv image frame */
void Test_Sim::_initializeTransformationMatrix(){


    // origin with effect of resolution
    int worldOriginInPixelUnitsX = _costmapOriginX/_costmapResolution;
    int worldOriginInPixelUnitsY = _costmapOriginY/_costmapResolution;


    // Transform (in pixel coords) from world origin to Rviz origin (image's bottom right corner)
    Eigen::Matrix3d T_World_RVIZ ;
    T_World_RVIZ << 1 , 0, worldOriginInPixelUnitsX,
                    0,  1, worldOriginInPixelUnitsY, 
                    0,  0,                        1;


    // Transform (in pixel coords) from Rviz origin( image's bottom right corner) to OpenCV's image frame (top left corner with inverted axes)
    Eigen::Matrix3d T_RVIZ_Image;
    T_RVIZ_Image << -1 , 0, _costmapSize,
                     0, -1, _costmapSize,
                     0,   0,           1;


    // T02 = T01 * T12
    _T_World_Image = T_World_RVIZ * T_RVIZ_Image;


}

/*
 * displayGlobalGrid()
 * Input : None
 * Output : void
 * 
 * brief : Displays grayscale image of your environment. 
 * 
 * 
 */

/* Displays the global grid */
void Test_Sim::displayGlobalGrid(){

    // wait till callbacks are initialized
    while(!_b_isOccupancyGridImageInitialized && ros::ok()){
        ROS_WARN("OccupancyGrid Not Yet initialized\n");
        ros::Duration(1).sleep();
    }


    // std::cout << "Image rows : " << _occupancyGridImage.rows << "\n";

    // error if image is empty
    if(!_occupancyGridImage.data){
        ROS_ERROR("Could not load the map image");
        return;
    }


    // display image
    cv::namedWindow(_windowName , cv::WINDOW_AUTOSIZE);
    cv::imshow(_windowName, _occupancyGridImage);
    auto k = cv::waitKey(1000);                     // change the constant if you want to see image for long time 

    // destroy image windows
    // if(k==27){
        cv::destroyAllWindows();
        // exit(0);
    // }

    // cv::imwrite("/home/nagarjunvinukonda/global.jpg", _occupancyGridImage);
}




/*
 * velocityPublisher_()
 * Input : geometry_msgs::Twist&
 * Output : void
 * 
 * brief : scales down the (vx,wz) robot velocity to 
 * 		   avoid drift
 */
void Test_Sim::velocityPublisher_(const geometry_msgs::Twist& agentVel){

	std::cout << "velocityPublisher_\n";

	geometry_msgs::Twist velocity;

	if(fabs(agentVel.angular.z) > 1){  						// get the robot orient towards the goal


		// to capture the sense of rotation
		float senseOfRotation = (agentVel.angular.z) / 
								fabs(agentVel.angular.z);


		velocity.linear.x = 0.01;
		velocity.angular.z = 0.4 * senseOfRotation; 

	}
	else{			// clip both velocities at 0.6 units


		velocity.angular.z = (agentVel.angular.z > 0.6) ? 0.6 : agentVel.angular.z;
		velocity.linear.x = (agentVel.linear.x > 0.6) ? 0.6 : agentVel.linear.x;

	}

	float timeStep = sim->getTimeStep();
	
	// const nav_msgs::OccupancyGrid::ConstPtr grid;

	// send t+1 velocity to forwardSimulation check
	forwardSimulationPosition(velocity, timeHorizonObstacle_, timeStep);

	// publish velocity
	velocity_pb_.publish(velocity);

	//sleep
	ros::Duration(0.1).sleep();
	
}


/*
 * forwardSimulationPosition()
 * Input : geometry_msgs::Twist& , time Horizon , float timestep_
 * Output : void
 * 
 * brief : checks if the velocity lies in human and obstacle proxemics
 * 		   
 */

void Test_Sim::forwardSimulationPosition(const geometry_msgs::Twist& velocity, double timeHorizonObstacle_, float timeStep_){

	double angle;
	angle=atan2(velocity.linear.y,velocity.linear.x)*180/M_PI;


	// RVO::Vector2 position_t1(agentPosition_.x() + velocity.linear.x*timeHorizonObstacle_*timeStep_,agentPosition_.y() + velocity.linear.y*timeHorizonObstacle_*timeStep_);
	RVO::Vector2 position_t1(agentPosition_.x() ,agentPosition_.y());


	std::cout<<"New position x:"<<position_t1.x()<<"New position y:"<<position_t1.y()<<std::endl;


	CheckInObstacleSpace(position_t1);
	

}

void Test_Sim::CheckInObstacleSpace(RVO::Vector2 position_t1){

	Eigen::Vector3d position = convertVec2ToVec3(position_t1);

	Eigen::Vector3d OccupancyGridPosition = Test_Sim::_convertFromWorldFrame(position);

	
	// To check if the computed ORCA t+1 velocity lies in human or obstacle  
	if(_occupancyGridImage.at<uchar>(OccupancyGridPosition[0], OccupancyGridPosition[1])>20){
		std::cout<<"Bad velocity"<<std::endl;
	}
	else{

		std::cout<<"Good Velocity"<<std::endl;
	}



}


Eigen::Vector3d Test_Sim::convertVec2ToVec3(const RVO::Vector2& position){


	Eigen::Vector3d position3d(position.x(),position.y(),0);

	// return Eigen::Vector3d(position.x,position.y,0);
	return position3d;

}


/* Converts a point from world frame to frame */
Eigen::Vector3d Test_Sim::_convertFromWorldFrame(Eigen::Vector3d pointInWorldFrame){
    return _T_World_Image.inverse() * pointInWorldFrame;
}




/*
 * transformVelocity()
 * Input : tf::Matrix3x3 [trasformation matrix (/map -> /base_footprint) ], 
 * 		   tf::Vector3 [velocity to be transformed]
 * Output : tf::Vector3 [transformed velocity]
 * 
 * brief : tranforms velocity wrt to the transformation 
 * 		   matrix
 */
tf::Vector3 Test_Sim::transformVelocity_(tf::Matrix3x3& mat, 
										 tf::Vector3& velInWorld){

	// std::cout << "transformVelocity_\n";
	return tf::Vector3(
				   mat.getColumn(0).getX()*velInWorld.getX() + 
				   mat.getColumn(1).getX()*velInWorld.getY() + 
				   mat.getColumn(2).getX()*velInWorld.getZ(),

				   mat.getColumn(0).getY()*velInWorld.getX() + 
				   mat.getColumn(1).getY()*velInWorld.getY() + 
				   mat.getColumn(2).getY()*velInWorld.getZ(),

				   mat.getColumn(0).getZ()*velInWorld.getX() + 
				   mat.getColumn(1).getZ()*velInWorld.getY() + 
				   mat.getColumn(2).getZ()*velInWorld.getZ());
}





/*
 * transformVelToRobotFrame_()
 * Input : geometry_msgs::Twist&
 * Output : geometry_msgs::Twist
 * 
 * brief : transforms velocity from world frame to 
 * 		   robot frame and converts into (vx, wz)
 * 
 */
geometry_msgs::Twist Test_Sim::transformVelToRobotFrame_(geometry_msgs::Twist& msg){
	

	// std::cout << "transformVelToRobotFrame_\n";

	tf::Vector3 velInWorldFrame = tf::Vector3( msg.linear.x, 
											   msg.linear.y, 
											   0.0);
	
	// tranformation matrix (world -> robot base)
	tf::Matrix3x3 matWorldToRobot = transform1_.getBasis();


	tf::Vector3 velRobotFrame = transformVelocity_(matWorldToRobot, 
												  velInWorldFrame);

	

	// convert (vx, vy) into (vx, omega_z) because 
	// that is how Turtlebots work _/\_
	geometry_msgs::Twist velRobotFrame_Modified;



	velRobotFrame_Modified.angular.z = 1.0 * atan2(velRobotFrame.getY(),
                                    velRobotFrame.getX());
	

	double linearVelScalarMultiplier {1.0};
	// if(velRobotFrame_Modified.angular.z > 0.5)
	// 	linearVelScalarMultiplier = 0.3;
    

	double velNorm = sqrt(pow(velRobotFrame.getX(), 2) + 
						  pow(velRobotFrame.getY(), 2));

	velRobotFrame_Modified.linear.x = linearVelScalarMultiplier * velNorm;

	return velRobotFrame_Modified;
}








/*
 * dispTransformationMat_()
 * Input : tf::Matrix3x3
 * Output : void
 * 
 * brief : Display the transformation matrix
 */
void Test_Sim::dispTransformationMat_(tf::Matrix3x3& mat){
	

	std::cout << "Matrix = " << "\n \t" ;
	std::cout << (mat.getColumn(0)).getX() << " \t" 
			  << (mat.getColumn(1)).getX() << " \t"
			  << (mat.getColumn(2)).getX() << " \t" 
			  << transform1_.getOrigin().getX() << "\n\t";

	std::cout << (mat.getColumn(0)).getY() << " \t"
			  << (mat.getColumn(1)).getY() << " \t" 
			  << (mat.getColumn(2)).getY() << " \t" 
			  << transform1_.getOrigin().getY() << "\n\t";

	std::cout << (mat.getColumn(0)).getZ() << " \t"
			  << (mat.getColumn(1)).getZ() << " \t" 
			  << (mat.getColumn(2)).getZ() << "\t" 
			  << transform1_.getOrigin().getZ() << "\n";
}






/*
 * printObstacleVector_()
 * Input  : std::vector for RVO::Vector2 {each element of 
 * 			vector is obstacle point in odom co-ordinate frame}
 * Output : Square of distance between current point and 
 * 			previous point 
 * 
 * brief : This function prints each obstacle point obtained 
 * 		   in robot-frame which got converted in odom frame
 * 
 */
void Test_Sim::printObstacleVector_(std::vector<RVO::Vector2>& obj){
	std::cout << "Printing Obstalces \n Number of obstacle points sent to ORCA : = " << obj.size() << "\n";
	for(int i=0; i<obj.size(); i++){
		std::cout << " " <<obj[i] << "\n";
	}

	std::cout << "\n";
}






/*
 * norm2_()
 * Input  : RVO::Vector2 [current point in world frame],
 * 		    RVO::Vector2 [previous point in world frame]
 * Output : Square of distance between current point and 
 * 			previous point 
 * 
 * brief : Computes relative distance between two points 
 * 
 */
double Test_Sim::norm2_ (RVO::Vector2& currPoint, 
						RVO::Vector2& prevPoint){

	// std::cout << "norm2_\n";
    return pow(currPoint.x() - prevPoint.x(),2) + 
		   pow(currPoint.y() - prevPoint.y(),2);
}






/*
 * transformPointToWorldFrame()
 * Input : tf::Matrix3x3 [trasformation matrix /base_scan -> /map ], 
 * 		   tf::Vector3 [origin of /map in /base_scan] , 
 * 		   tf::Vector3 [point to be transformed]
 * Output : RVO::Vector2 [point in /map frame]
 * 
 * 
 */
RVO::Vector2 Test_Sim::transformPointToWorldFrame( tf::Matrix3x3& mat, 
												   const tf::Vector3& origin , 
												   const tf::Vector3& point) {
	
	// std::cout << "transformPointToWorldFrame\n";
	return RVO::Vector2(
					mat.getColumn(0).getX()*point.getX() + 
					mat.getColumn(1).getX()*point.getY() +
					mat.getColumn(2).getX()*point.getZ() + 
					origin.getX()*1,
					mat.getColumn(0).getY()*point.getX() + 
					mat.getColumn(1).getY()*point.getY() + 
					mat.getColumn(2).getY()*point.getZ() + 
					origin.getY()*1
					);
}







int main(int argc, char**argv)
{		
	ros::init(argc, argv, "simple_obstacle_orca1");
	ros::NodeHandle nh;

	Test_Sim tsim1(nh);

	return 0;
}
