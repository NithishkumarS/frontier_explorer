# Frontier Explorer

[![Build Status](https://travis-ci.org/nantha007/frontierExplorer.svg?branch=master)](https://travis-ci.org/nantha007/frontierExplorer)
[![Coverage Status](https://coveralls.io/repos/github/nantha007/frontierExplorer/badge.svg?branch=master)](https://coveralls.io/github/nantha007/frontierExplorer?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


## Overview

This Project uses turtlebot to autonomously explore and reconstructing the unknown map using SLAM. Frontiers are locations where explored and unexplored areas of max intersect. This module is capable of autonomously selecting the next target location to explore in an unknown envirnment. It has the following functionalities:

 - Locating frontiers points using the intial map rom the gmapping module. 
 - Clustering of frontiers 
 - Identifying the nearest frontier centroid to start position using the Breadth First Search algorithm
 - Shortest path is also generated using BFS. 
 - Waypoints navigation using move_base package.

This uses ROS features such as Gazebo, gmapping, RVIZ and navigation stack.

The robot is designed to map the radiation levels of the plant autonomously. This radiation map will be used to plan the emergency response action. Turtlebot, which will be used, is a two wheeled differential driven robot with LIDAR and radiation level measurement sensors. The radiation level sensor is modelled to measure the radiation of the localized parts of the map. After nuclear meltdown the plant debris would be a new obstacle for the robot. So, Simultaneous Localization and Mapping (SLAM) based on laser is used to reconstruct the map. SLAM is a widely used technique to construct the map of an unknown environment.

The project was implemented using C++ language utilizing C++11 features, giving an emphasis on object oriented programming and encapsulation. The total project was implemented in the Eclipse IDE platform. It follows C++  Google style guide and cpplint validation.  Project was done in Test Driven development setup, utilizing unit testing via google framework. Cppcheck was also used. It follows Doxygen style comments to enable doxygen documentation.

The project implementation was done following the Solo Iterative Process in a 4 week sprint schedule. It includes one week for ideation part. Initial planning included creation of product backlogs and break them into smaller repeatable iterations. Creation of UML class and activity diagrams to guide the rest of the project was done. Second week included the implementation of the actual class modules with unit testing. Third week was utilized for fixing the ROS pipelines. last week was utilized to implement the navigation stack and other programming aspects. 


# Outputs from the Project:     


Images from RVIZ and gazebo


## About the developers
```
Nithish Kumar
   I am a graduate student in Robotics at the University of Maryland with an interest in control and path planning.

Nantha Kumar Sunder
   I am a graduate student in Robotics at the University of Maryland with an interest in artificial intelligence.
```

## DISCLAIMER

```

BSD 3-Clause License

Copyright (c) 2018, Nantha Kumar Sunder, Nithish Kumar S
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

```


## Dependencies

The following packages are needed to run this program:
* Ubuntu Xenial 16.04
* ROS Kinetic
* Gazebo
* RVIZ
* Movebase

## Documentation

Driver and navigator role is interchanged and changes are commited by the driver.

This project follows SIP. The link for the document can be found out [here](https://docs.google.com/spreadsheets/d/1-KtADCIqD6HAsYGd68oA5P-hs-cmr5l4iTkNGygmUoM/edit?usp=sharing).

The documentation for sprint notes is can be found [here](https://docs.google.com/document/d/1QWXToDZXNtXEsLOP8cRoNLw7FY7OOFWPk1sTGDPnR8Q/edit?usp=sharing).


| Directory | Description 			    |
| --------- | ------------------------------------- |
| `src`	    | Contains implementation of the nodes  |
| `test`    | Contains the test and its launch files|
| `Results` | Contains the results and the bag files|
| `launch`  | Holds the xml launch file 	    |
| `include` | COntains the main directories	    |


## Build Steps

Once catkin workspace is created, open terminal and type the following commands to clone the repository.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone --recursive https://github.com/nantha007/frontier_explorer.git
```

Once repository is cloned, type the following commands to setup the repository.

```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make
```

## Running tests

```
cd ~/catkin_ws/
catkin_make_isolated --make-args run_tests 
```

## Running the Demo through launch file

To run a demo using the following command

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch frontier_explorer frontier_explorer.launch
```


## Rosbag:

To record the bags from the launch, run the following commands.

```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make
roslaunch frontier_explorer frontier_explorer.launch rec:=1 
```

To specify the duration of the record

```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make
roslaunch frontier_explorer frontier_explorer.launch rec:=1 duration:=20
```

Rosbag could be retrieved from the Results folder in frontier_explorer

Navigate to the results folder
```
cd ~/catkin_ws/src/frontier_explorer/Results
rosbag play record.bag
```

## Doxygen Documentation  

To generte the doxygen files

```
sudo apt-get install doxygen
sudo apt-get install doxywizard
doxygen -g
```

Configure the doxyfile and run the the following commands.
```
doxygen Doxyfile
```




