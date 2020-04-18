#### Table of Contents:
1. [Introduction](#ros-tutorial-cum-challenge)
1. [Week 1](#week-1)
1. [Week 2](#week-2)


# ROS Tutorial cum Challenge

Hello everyone,
Hope you are safe and doing some productive work at home. If you are enthusiastic in robotics then, Electronics and Robotics Club is here once again to help you out in exploring this field along with ITSP.
Have you ever wondered “How to coordinate between multiple drones? How to simulate a manipulator or robot? How does a robot map the environment and navigate in it?” Answer to each and every question is ROS! These are only some of its applications. ROS applications go far beyond your imagination, above are only some basic things to mention. So what exactly is ROS?

___ROS___, which means the Robot Operating System, is a set of software libraries and tools to help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. The point of ROS is to create a __robotics standard__, so you don't need to __reinvent the wheel anymore when building new robotic software.__


| Sr. | Week   | Tasks |
:---: |:------:|:------|
| 1 | Week 1 | Ubuntu installation, get familiar with Linux commands, ROS setup |
| 2	| Week 2 | Get an overview of ROS Framework and get familiar with Basic terms of ROS framework |
| 3	| Week 3 | Create your first simulation! And perform the task that can be implemented in real life! |
| 4	| Week 4 |Autonomous Navigation and Path Planning |

### Main Objectives of this Tutorial:
1. The objective of this course is to give you the basic tools and knowledge to be able to understand and create any basic ROS related project. You will be able to move robots, read their sensor data, make the robots perform intelligent tasks, see visual representations of complex data such as laser scans and debug errors in the programs.
1. This will allow you to understand the packages that others have done. So you can take ROS code made by others and understand what is happening and how to modify it for your own purposes
1. This can serve as an introduction to be able to understand the ROS documentation of complex ROS packages for object recognition, text to speech, navigation and all the other areas where ROS developed code.

## Week 1
* __Ubuntu Installation__ :
For using Ros framework Ubuntu is necessary:
(It's Preferable that you install Ubuntu 16.04)
Follow this [Tutorial](https://parthvpatil.github.io/tutorial/2019/06/03/ubuntu.html "Ubuntu Installation") to install ubuntu in your laptop.  
<span style="color:red">[WARNING], Do at your own risk! We will be not responsible if you lose your data. __Follow instructions carefully and make backups before you start!__</span>

* __Get familiar with Linux__:
Here are a few resources that you can refer to in order to get familiar with Linux:
	* [Video-based Tutorial](https://www.youtube.com/watch?v=IVquJh3DXUA "Introduction to Linux and Basic Linux Commands for Beginners")
	* [Text-based Tutorial](https://ryanstutorials.net/linuxtutorial/ "Linux Tutorial")

* __ROS Installation/setup__:
	- For Ubuntu 16.04: [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation "kinetic/Installation")
	- For Ubuntu 18.04: [ROS Melodic Morena](http://wiki.ros.org/melodic/Installation "melodic/Installation")    
Go to a particular link and put your first step in the world of ROS.


## Week 2
_Now that the installation is done, let’s dive into ROS!_

##### What is ROS?
ROS is a software framework for writing robot software. The main aim of ROS is to reuse the robotic software across the globe. ROS consists of a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.  
The official definition on ROS WiKi is:  
_ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. ROS is similar in some respects to ‘robot frameworks, such as Player, YARP, Orocos, CARMEN, Orca, MOOS, and Microsoft Robotics Studio._

##### What is a package?
ROS uses __packages__ to organize its programs. You can think of a package as __all the files that a specific ROS program contains__; all its CPP files, python files, configuration files, compilation files, launch files, and parameter files. All those files in the package are organized with the following structure:

* __launch__ folder: Contains launch files
* __src folder__: Source files (CPP, python)
* __CMakeLists.txt__: List of CMake rules for compilation
* __package.xml__: Package information and dependencies

To go to any ROS package, ROS gives you a command named `roscd`. When typing:
```bash
roscd <package_name>
```
It will take you to the path where the package *package_name* is located.  
`roscd` is a command which will get you to a ROS package location. `ls` is a command that lists the content of a folder.  

* Every ROS program that you want to execute is organized in a package
* Every ROS program that you create will have to be organized in a package
* Packages are the main organizational system of ROS programs

##### Create a package
Until now we’ve been checking the structure of an already-built package. But now, let’s create one ourselves. When we want to create packages, we need to work in a very specific ROS workspace, which is known as the __catkin workspace__. The __catkin workspace__ is the directory in your hard disk where your own ROS packages must reside in order to be usable by ROS.Usually, the catkin workspace directory is called *catkin_ws*.  

Usually, the *catkin_ws* is created in the *home* folder of your user account. The catkin workspace has been already created and initialized for you.  

Go to the src folder inside catkin_ws:
```bash
cd ~/catkin_ws/src
```
The *src* directory is the folder that holds created packages. Those could be your own packages or packages that you copied from other sources e.g. A Github Repository.

In order for the ROS system to recognize the packages in your *catkin_ws*, it needs to be on the ROS file path. ROS file path is an Ubuntu environment variable that holds the paths to ROS packages. To add our *catkin_ws* to the ROS file path follow the following instructions.

First, build (compile) your workspace. It’s OK to build the *catkin_ws* even if it has no packages. After the build process, some new folders will appear inside your *catkin_ws*. One of the folders, called *catkin_ws/devel* contains a setup file that will be used to add the path of the *catkin_ws* to the ROS file path. Build the *catkin_ws* using the `catkin build` inside the *catkin_ws*:

```bash
cd ~/catkin_ws	# Navigate to the catkin_ws
catkin build 	# Build
```

Now, let’s add the *catkin_ws* path. Execute the following command while inside *catkin_ws*:
```bash
source devel/setup.bash
```
This adds the *catkin_ws* path in the current terminal session but once you close the terminal window, it forgets it! So, you will have to do it again each time you open a terminal in order for ROS to recognize your workspace! Yeah, I know, that sucks! But no worries, there is a solution. You can automate the execution of the above command each time you open a terminal window. To do that, you want to add the above command to a special file called *.bashrc* that is located inside your home folder.
```bash
cd ~			# go to the home folder
nano .bashrc	# open the .bashrc file
```
Add the command `source ~/catkin_ws/devel/setup.bash` to the end of *.bashrc*.  
Then, hit <kbd>CTRL</kbd>+<kbd>X</kbd>, then, <kbd>Y</kbd>, to save the changes to the file.

Now, let’s create a package:










