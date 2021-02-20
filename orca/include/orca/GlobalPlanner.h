# pragma once

# include <ros/ros.h>
# include <nav_msgs/OccupancyGrid.h>
# include <nav_msgs/Odometry.h>
# include <opencv2/highgui/highgui.hpp>
# include <opencv2/imgproc/imgproc.hpp>

# include <vector>
# include <string>
# include <exception>
# include <memory>

# include <eigen3/Eigen/Dense>

class GlobalPlanner{

public:

    /**
	 * \brief      default contructor
	 */
    GlobalPlanner();


    /**
	 * \brief      parameterized contructor
     * 
     * \param     nh - accepts nodehandle for initializing ros 
     *                  subscriber
	 */
    GlobalPlanner(ros::NodeHandle& nh);


    /**
	 * \brief      class destructor 
	 */
    ~GlobalPlanner();


    /**
	 * \brief      sets start and goal positions
     * 
     * \param     goal - goal point as a 2 element vector
	 */
    void setGoal( std::vector<double> goal);


    /**
	 * \brief      displays the global grid
     *  
	 */
    void displayGlobalGrid();


    /**
	 * \brief      calls the 'X' algorithm's makePlan function
     *             and receives a std::vector<int> path
	 */
    std::vector<std::vector<double>> makePlan();


private:

    // start and goal in m
    std::vector<double> _startPos_XY;           
    std::vector<double> _goalPos_XY;

    // start and goal in pixels
    std::vector<int> _startPos_IJ;
    std::vector<int> _goalPos_IJ;

    // grid properties set by User in yaml
    int _gridSize{100};
    bool _b_resizeGrid{false};
    float _gridResizeScale{1.0f};
    int _gridConnections{8};
    int _obstacleThreshold{150};

    // costmap properties exhibited by RVIZ
    int _costmapSize{100};
    double _costmapOriginX{0.0f};
    double _costmapOriginY{0.0f};
    double _costmapResolution{0.0f};

    // condition flags to check if subscriber callbacks are initialized
    bool _b_isOccupancyGridImageInitialized{false};
    bool _b_isOdomCallbackInitialized{false};


    ros::NodeHandle _nh;

    std::string _windowName{"Global Occupancy Grid"};

    cv::Mat _occupancyGridImage;

    ros::Subscriber _occupancyGridSubscriber;

    std::vector<std::vector<double>> _bestPathGlobal;

    // transformation matrix from (Gazebo's) world  frame to OpenCv image frame
    Eigen::Matrix3d _T_World_Image;           


    /* @ USER if you plan to use your planning algorithm, create its unique ptr here*/
    //Create a Unique Ptr Instance of the User's algorithm
    std::unique_ptr<AStar> _globalPlannerAStar;


    // name of global planner as stated in planner.yaml
    std::string _globalPlannerName{"unknown"};
   

    /**
	 * \brief      gets the current robot position from ROS
	 */
    void _getCurrentRobotPosition();


    /**
	 * \brief      odometry callback function, extracts robot's pose
	 */
    void _odomCallback(const nav_msgs::Odometry::ConstPtr& odom);


    /**
	 * \brief      converts the occupancy callback message into
     *             2d CV::Mat based grid 
     * 
     * \param     grid - const ptr to nav_msgs::OccupancyGrid 
     *                   (check ROS API docs)
	 */
    void _occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid);


    /**
	 * \brief      loads the rosparams for global planner
	 */
    void _loadParams();  

    /**
	 * \brief      initialize the transformation matrix from Gazebo world
     *             to opencv image frame
	 */
    void _initializeTransformationMatrix();

    /**
	 * \brief      converts a point from openCV frame to 
     *             world frame
     * 
     * \param      pointInImageFrame is a 2 dimensional point in homogeneous
     *              system (in OpenCV frame)
	 */
    Eigen::Vector3d _convertToWorldFrame(Eigen::Vector3d pointInImageFrame); 


    /**
	 * \brief      converts a point from world frame to 
     *             openCv frame
     * 
     * \param      pointInWorldFrame is a 2 dimensional point in homogeneous
     *              system (in world frame)
	 */
    Eigen::Vector3d _convertFromWorldFrame(Eigen::Vector3d pointInWorldFrame);


    /**
	 * \brief      converts an std::vector point into eigen::vector3d object
     * 
     * \param      point - std::vector (i,j) of size 2 representing a point 
	 */
    template<typename T>
    Eigen::Vector3d _getEigenVector3Point(const std::vector<T>& point);


    /**
	 * \brief      converts an eigen::Vector3d point into an std::vector object
     * 
     * \param      eigenPoint - eigen::vector3d (i,j,1) representing a point in 
     *             homogeneous system
	 */
    template<typename T>
    std::vector<T> _getSTDVector3Point(Eigen::Vector3d& eigenPoint);


    /**
	 * \brief      resizes the global grid
     *  
	 */
    void _resizeGrid();


    /**
	 * \brief      initialize boundary points into pixel coordinates
     */    
    void _initializeStartAndGoalInPixel();  

    /**
	 * \brief      converts the global path from pixel coords to meters
     * 
     * \param      bestPath - a 2d vector of path points(i,j)
     */   
    void _extractBestPath(const std::vector<std::vector<int>>& bestPath);
};



// Free function
/**
 * \brief      overloads / operator to perform vector / scalar division
 * 
 * \param      
 */
template<typename T1, typename T2>
std::vector<T1> operator/(const std::vector<T1>& vect, const T2& scalar);


/**
 * \brief      overloads * operator to perform vector * scalar multiplication
 * 
 * \param      
 */
template<typename T1, typename T2>
std::vector<T1> operator*(const std::vector<T1>& vect, const T2& scalar);