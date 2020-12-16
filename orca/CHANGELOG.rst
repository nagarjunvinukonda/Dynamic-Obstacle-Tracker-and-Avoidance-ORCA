^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package orca
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1 (2020-11-19)
-------------------
* Created package for orca
* Added includes folder - contains all the header files from RVO2 library and for new files
* Added src folder - contains the cpp files from RVO2 library and for new files
* Added logs folder - to log the data printed while running orca->test_sim.cpp



1.2 (2020-12-02)
-------------------
* Updated test_sim.cpp file 
	- Made ORCA up and running with static world
	- Removed unnecessary print statements
	- Updated variable naming conventions
	- Updated coding style (camelCasing)
	- Created a new function runORCA_ which contains the loop for ORCA
* Added test_sim_Functional_Information.txt which explains the API for test_sim.cpp
* Added CHANGELOG.rst


1.3 (2020-12-03)
-------------------
* Updated CMakeLists.txt of orca package : 
	- Removed OPENCV_DIR tag
	- Only build executables map_to_odom_publisher & test_sim
	- Added add_dependency tag to test_sim executable
	- Raised the order of orca_msgs includes at the start of all header files using them
	- Made necessary changes to incorporate DetectedEntity arrays in Agent{.h & .cpp} , Test_Sim{.h & .cpp} files
