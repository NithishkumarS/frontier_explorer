# Frontier Explorer

[![Build Status](https://travis-ci.org/nantha007/frontierExplorer.svg?branch=master)](https://travis-ci.org/nantha007/frontierExplorer)
[![Coverage Status](https://coveralls.io/repos/github/nantha007/frontierExplorer/badge.svg?branch=master)](https://coveralls.io/github/nantha007/frontierExplorer?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


## Overview

This Project uses turtlebot to autonomously explore and reconstructing the unknown map using SLAM. It also avoids the obstacle in its way. ROS features such as Gazebo, gmapping, etc are used. 


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

## Documentation

This project follows SIP. The link for the document can be found out [here](https://docs.google.com/spreadsheets/d/1-KtADCIqD6HAsYGd68oA5P-hs-cmr5l4iTkNGygmUoM/edit?usp=sharing).

The documentation for sprint notes is can be found [here](https://docs.google.com/document/d/1QWXToDZXNtXEsLOP8cRoNLw7FY7OOFWPk1sTGDPnR8Q/edit?usp=sharing).


## Instructions to create catkin workspace

To create catkin workspace, open terminal and type the following commands
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```


## Instructions to setup catkin workspace

Once catkin workspace is created, open terminal and type the following commands to clone the repository.

``` 
cd ~/catkin_ws/src/
git clone --recursive https://github.com/nantha007/frontierExplorer.git
```

Once repository is cloned, type the following commands to setup the repository.

```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make
```



