#### Table of Contents:
1. [Introduction](#ros-tutorials-and-projects)
1. [Week 1](#week-1)
	* [Assignment 1](#assignment-1)
1. [Week 2](#week-2)
	* [Assignment 2](#assignment-2)
1. [Week 3](#week-3)

# ROS - Tutorials and Projects

Hello everyone,
Hope you are safe and doing some productive work at home. If you are enthusiastic in robotics then, Electronics and Robotics Club is here once again to help you out in exploring this field along with ITSP.
Have you ever wondered “How to coordinate between multiple drones? How to simulate a manipulator or robot? How does a robot map the environment and navigate in it?” Answer to each and every question is ROS! These are only some of its applications. ROS applications go far beyond your imagination, above are only some basic things to mention. So what exactly is ROS?

___ROS___, which means the Robot Operating System, is a set of software libraries and tools to help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. The point of ROS is to create a __robotics standard__, so you don't need to __reinvent the wheel anymore when building new robotic software.__


<!-- | Sr. | Week   | Tasks |
:---: |:------:|:------|
| 1 | Week 1 | Ubuntu installation, get familiar with Linux commands, ROS setup |
| 2	| Week 2 | Get an overview of ROS Framework and get familiar with Basic terms of ROS framework |
| 3	| Week 3 | Create your first simulation! And perform the task that can be implemented in real life! |
| 4	| Week 4 |Autonomous Navigation and Path Planning | -->

### Main Objectives of this Tutorial:
1. The objective of this course is to give you the basic tools and knowledge to be able to understand and create any basic ROS related project. You will be able to move robots, read their sensor data, make the robots perform intelligent tasks, see visual representations of complex data such as laser scans and debug errors in the programs.
1. This will allow you to understand the packages that others have done. So you can take ROS code made by others and understand what is happening and how to modify it for your own purposes
1. This can serve as an introduction to be able to understand the ROS documentation of complex ROS packages for object recognition, text to speech, navigation and all the other areas where ROS developed code.

## Week 1
* __Ubuntu Installation__ :
For using ROS framework Ubuntu is necessary:
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

### **Getting started with the ROS:**

*Now that the installation is done, let’s dive into ROS!*

#### **What is ROS?**

ROS is a software framework for writing robot software. The main aim of ROS is to reuse the robotic software across the globe. ROS consists of a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
The official definition on ROS wiki is:

*ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. ROS is similar in some respects to ‘robot frameworks, such as Player, YARP, Orocos, CARMEN, Orca, MOOS, and Microsoft Robotics Studio.*

#### **Basics of ROS**

First of all, let us start with the basics of ROS.
Go through the beginner level [Tutorials](http://wiki.ros.org/ROS/Tutorials#Beginner_Level). It will cover all the basics materials like how to create a package, what is a node and how to make it, what is a publisher and a subscriber? It will give you a detailed introduction to each and every thing. There are **20** parts to this tutorial, go through them all. Don't go to the intermediate level right now if you get done with the beginner level. We will give you an assignment based on these topics. **Write code on your own. Don't copy paste it directly.**  You will grasp the topics covered better when you try the implementation on your own. Assignment will be releasd soon. It's preferable if you use __Python__ instead of __C++__ as python syntax is easier and more readable and you will need it in future for sure, so, better learn it right now. These assignments will require knowledge of only basic python syntax.

Here, we are briefing about what is a package and how to create a package. This is also given in the tutorials, but we are telling this explicitly because it is the most fundamental thing that you have to do when you start with ROS.

##### **What is a package?** 

ROS uses **packages** to organize its programs. You can think of a package as **all the files that a specific ROS program contains**; all its CPP files, python files, configuration files, compilation files, launch files, and parameter files. All those files in the package are organized with the following structure:

* __launch__ folder: Contains launch files
* __src folder__: Source files (CPP, python)
* __CMakeLists.txt__: List of CMake rules for compilation
* __package.xml__: Package information and dependencies

To go to any ROS package, ROS gives you a command called `roscd`. Type:

`roscd <package_name>`

It will take you to the path where the package *package_name* is located. `roscd`  is a command which will get you to a ROS package location. `ls`is a command that lists the content of a folder.

* Every ROS program that you want to execute is organized in a package
* Every ROS program that you create will have to be organized in a package
* Packages are the main organizational system of ROS programs


##### **Create a package**

Until now we’ve been checking the structure of an already-built package. But now, let’s create one ourselves. When we want to create packages, we need to work in a very specific ROS workspace, which is known as the catkin workspace. The **catkin workspace** is the directory in your hard disk where your own ROS packages must reside in order to be usable by ROS. Usually, the catkin workspace directory is called *catkin_ws* .

Usually, the *catkin_ws* is created in the home folder of your user account. The catkin workspace has been already created and initialized for you.

Go to the src folder inside *catkin_ws* :

```bash
cd ~/catkin_ws /src
```

The *src* directory is the folder that holds created packages. Those could be your own packages or packages that you copied from other sources e.g. A Github Repository.

In order for the ROS system to recognize the packages in your *catkin_ws*, it needs to be on the ROS file path. ROS file path is an Ubuntu environment variable that holds the paths to ROS packages. To add our *catkin_ws* to the ROS file path follow the following instructions.

First, build (compile) your workspace. It’s OK to build the *catkin_ws*  even if it has no packages. After the build process, some new folders will appear inside your *catkin_ws* . One of the folders, called *catkin_ws* /devel contains a setup file that will be used to add the path of the *catkin_ws*  to the ROS file path. Build the *catkin_ws*  using the catkin build inside the *catkin_ws* :

```bash
cd ~/catkin_ws 	# Navigate to the catkin_ws
catkin build 	# Build
```
Now, let’s add the *catkin_ws*  path. Execute the following command while inside *catkin_ws* :

```bash
source devel/setup.bash
```
This adds the *catkin_ws*  path in the current terminal session but once you close the terminal window, it forgets it! So, you will have to do it again each time you open a terminal in order for ROS to recognize your workspace! Yeah, I know, that sucks! But no worries, there is a solution. You can automate the execution of the above command each time you open a terminal window. To do that, you want to add the above command to a special file called .bashrc that is located inside your home folder.

```bash
cd ~		# go to the home folder
nano .bashrc	# open the .bashrc file
```
Add the command `source ~/catkin_ws/devel/setup.bash` to the end of *.bashrc*.  
Then, hit <kbd>CTRL</kbd>+<kbd>X</kbd>, then, <kbd>Y</kbd>, to save the changes to the file.

Now, you can refer to the [tutorials](http://wiki.ros.org/ROS/Tutorials#Beginner_Level) on ROS wiki for further instructions.

### Assignment 1

___

##### Question:
 Create a new package with name in the following format NAME_SURNAME.
  Now, this new package will require **three nodes** and **two launch files**. 

* The first node will publish your *NAME*(string) to the topic **name_listener**

* The second node will publish your *PHONE NUMBER*(int) to the topic **num_listener**

* The third node will subscribe both the above topics

* The first launch file contains two nodes. It contains both of the publisher nodes. 

* The second launch file contains the last subscriber node.

Your task is to print an assimilated string in the following format:

**If you want to contact NAME, then call on this NUMBER.**

NAME and NUMBER should be replaced by your own details.



#### Submission details:
You have to make a .zip file of your package and submit it to **erciitbombay@gmail.com**.  
Submissions of only registered people will be considered.  
**Deadline: 11:59 pm, 29/04/20**

---
## Week 2

What is presented here are the main ROS concepts that are the core of ROS. These are the most important concepts that you have to master. Once you master them, the rest of ROS can follow easily.

During this course, you will learn:

- How **ROS Basic Structure** works.
- What are **ROS Topics** and how to use them.
- What are **ROS Services** and how to use them.
- What are **ROS Actions** and how to use them.
- How to use **ROS Debugging Tools** for finding errors in your programs (especially Rviz).

Note:

  We will use **Python** as the programming language in all the course exercises


#### Install TurtleBot packages


During this tutorial, you will work with a simulated robot called **TurtleBot**, to apply the concepts of ROS. The following image is a picture of the robot you will work with. It is a differential drive robot, that has a Kinect sensor for environmental mapping, wheel encoders for position estimation.

<img src ="https://risc.readthedocs.io/_images/kobuki.jpg" width="250"/>


Open application called **Terminator**, you can install it by running following command in the terminal:  
```bash
sudo apt-get install terminator
```

It's highly recommended to use this application instead of stock Terminal. You can have tabs or split windows into few terminals. To install the required packages, execute the following command.

```bash
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-turtlebot-gazebo -y
```

Just copy and paste it in a terminal.


After the installation is done, check that the simulation works in Gazebo. Execute the following command in a shell terminal.


```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```


You should get something similar to the following.

![env](https://risc.readthedocs.io/_images/turtlebot-gazebo.png )

#### Move the robot


How can you move the Turtlebot?

The easiest way is by executing an existing ROS program to control the robot. A ROS program is executed by using some special files called **launch files**.
Since a previously-made ROS program already exists that allows you to move the robot using the keyboard, let's launch that ROS program to teleoperate the robot.

Execute in a separate terminal:

`roslaunch turtlebot_teleop keyboard_teleop.launch`

Read the instructions on the screen to know which keys to use to move the robot around, and start moving the robot!


Try it! When you're done, you can <kbd>CTRL</kbd>+<kbd>C</kbd> to stop the execution of the program.


#### What is a launch file ?


We've seen that ROS uses launch files in order to execute programs. But... how do they work? Let's have a look.

lets  have a look at a launch file. Open the launch folder inside the ``turtlebot_teleop`` package and check the ``keyboard_teleop.launch`` file.

``` bash

  roscd turtlebot_teleop
  cd launch
  gedit keyboard_teleop.launch

```


You will see:

``` xml
<launch>
    <!-- turtlebot_teleop_key already has its own built in velocity smoother -->
    <node pkg="turtlebot_teleop" type="turtlebot_teleop_key" name="turtlebot_teleop_keyboard"  output="screen">
      <param name="scale_linear" value="0.5" type="double"/>
      <param name="scale_angular" value="1.5" type="double"/>
      <remap from="turtlebot_teleop_keyboard/cmd_vel" to="cmd_vel_mux/input/teleop"/>
    </node>
</launch>
```

In the launch file, you have some extra tags for setting parameters and remaps. For now, don't worry about those tags and focus on the node tag.

All launch files are contained within a ``<launch>`` tag. Inside that tag, you can see a ``<node>`` tag, where we specify the following parameters:

- pkg="``package_name``": Name of the package that contains the code of the ROS program to execute
- type="``python_file_name.py``" : Name of the program file that we want to execute
- name="``node_name``" : Name of the ROS node that will launch our Python file
- output="``type_of_output``" : Through which channel you will print the output of the Python file

Now, lets create a package. Just a revision of your [previous](#create-a-package) tutorial.

___Remember to create ROS packages inside the ``src`` folder___


#### Create a package

```catkin_create_pkg my_package rospy```


This will create, inside our ``src``, directory a new package with some files in it. We'll check this later. Now, let's see how this command is built:

```catkin_create_pkg <package_name> <package_dependecies>```


The **package_name** is the name of the package you want to create, and the **package_dependencies** are the names of other ROS packages that your package depends on.

Now, re-build your catkin_ws and source it as above.

In order to check that our package has been created successfully, we can use some ROS commands related to packages. For example, let's type:

```bash
rospack list
rospack list | grep my_package
roscd my_package
```


``rospack list``: Gives you a list with all of the packages in your ROS system.

``rospack list | grep my_package``: Filters, from all of the packages located in the ROS system, the package named *my_package*.

``roscd my_package``: Takes you to the location in the Hard Drive of the package, named *my_package*.

#### Exercise: Move the Robot

Now you're ready to create your own publisher and make the robot move, so let's go for it!  

First, you need to bring up the robot simulation in Gazebo using the command:

`roslaunch turtlebot_gazebo turtlebot_world.launch`

#### Required information:

* The ``cmd_vel_mux/input/teleop`` topic is the topic used to move the robot. Do a ``rostopic info cmd_vel_mux/input/teleop`` in order to get information about this topic, and identify the message it uses. You have to modify the code to use that message.

* In order to fill the Twist message, you need to create an instance of the message. In Python, this is done like this: ``var = Twist()``

* In order to know the structure of the Twist messages, you need to use the ``rosmsg show`` command, with the type of the message used by the topic ``cmd_vel_mux/input/teleop``.

* In this case, the robot uses a differential drive plugin to move. That is, the robot can only move linearly in the *x* axis, or rotationally in the angular *z* axis. This means that the only values that you need to fill in the Twist message are the linear *x* and the angular *z*.

<img src ="https://risc.readthedocs.io/_images/xyz-frame.jpg" width="150">


The magnitudes of the Twist message are in m/s, so it is recommended to use values between 0 and 1. For example, *0.5 m/s*

#### What to do:

Create a launch file that launches the code ``topic_publisher.py`` (basically the name of the python file stored in your src folder of your package.)

Modify the code you used previously (in the previous week) to publish data to the ``cmd_vel_mux/input/teleop`` topic.

Launch the program and check that the robot moves.

You need the knowledge of attributes of rotopic and rosmsg like rostopic info, rostopic echo, rosmsg show. So, refer to the previous tutorial, if you have any queries with the same.

*At first it may seems to be a bit harder problem but we want you to think first and explore the ROS environment. Use the required information and think about what is happening. We don't expect that you will solve it at your first go. You will get stuck in between various times. This will lead to discussion and you will get to know more as much you discuss on the group. It is a very intersting problem. You will see your bot move autonomously.*

___

__Solution to the exercise__ :

```python
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_node')
pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.5 #Move the robot with a linear velocity in the x axis
move.angular.z = 0.5 #Move the with an angular velocity in the z axis

while not rospy.is_shutdown(): 
  pub.publish(move)
  rate.sleep()
```
___

### Assignment 2

___

In this assignment, you will write code to make the bot avoid walls in front of it. To help you achieve this, let’s divide the project down into smaller units:

* Create a Publisher that writes into the *cmd_vel_mux/input/teleop* topic in order to move the robot.

* Create a Subscriber that reads from the */scan* topic. This is the topic where the laser publishes its data.

Depending on the readings you receive from the laser’s topic, you’ll have to change the data you’re sending to the *cmd_vel_mux/input/teleop* topic, in order to avoid the wall. This means, use the values of the laser to decide.


### Required Information:

* The data that is published into the */scan* topic has a large structure. For this project, you just have to pay attention to the ranges array.
  
  Hint:
  To check the laser message type, execute the following:  `rosmsg show sensor_msgs/LaserScan`

* The *ranges* array has a lot of values. The ones that are in the middle of the array represent the distances that the laser is detecting right in front of it. This means that the values in the middle of the array will be the ones that detect the wall. So in order to avoid the wall, you just have to read these values.

    Hint: The size of the array is 639.  
* The laser has a range of 30m. When you get readings of values around 30, it means that the laser isn’t detecting anything. If you get a value that is under 30, this will mean that the laser is detecting some kind of obstacle in that direction (the wall).

* The scope of the laser is about 180 degrees from right to left. This means that the values at the beginning and at the end of the *ranges* array will be the ones related to the readings on the sides of the laser (left and right), while the values in the middle of the array will be the ones related to the front of the laser.

So, in the end you probably will get something like this:

The robot moves forward until it detects an obstacle in front of it which is closer than 1 meter, so it begins to turn left in order to avoid it.

<!-- Links -->
![Reuqired Output](https://risc.readthedocs.io/_images/mini_project_1.gif)

The robot keeps turning left and moving forward until it detects that it has an obstacle at the right side which is closer than 1 meter, so it stops and turns left in order to avoid it.

![Reuqired Output](https://risc.readthedocs.io/_images/mini_project_2.gif)

__If you are using ROS Melodic, do the following:__  
People having turtlebot3 have to simualte the same task in turtlebot3_house using any of the bots: burger, waffle, waffle-pi. Use all the required information as stated above.

#### Installation problem in turtlebot/turtlebot3:
For **ROS kinetic** do: `sudo apt-get install ros-kinetic-turtlebot-*`

For **ROS mleodic** do: `sudo apt-get install ros-melodic-turtlebot3-*`

If anyone is still facing any issue, then you can ping any of us ASAP.

#### Submission details:
You have to make a .zip file of your package and submit it to **erciitbombay@gmail.com**.  
Submissions of only registered people will be considered.  
**Deadline: 11:59 pm, 07/05/20**


## Week 3

People using ROS-kinetic do:

`sudo apt-get install ros-kinetic-turtlebot3-*`

Those using ROS-melodic do:

`sudo apt-get install ros-melodic-turtlebot3-*`

After the installation:

```bash
cd ~
nano .bashrc
```

At the end of your .bashrc file add this line:

```bash
export TURTLEBOT3_MODEL=waffle
```


We have mentioned waffle here. You can use waffle_pi or burger also.

In the following weeks we will cover these topics:

* SLAM
* Navigation
* Path Planning

Let's start with SLAM:

### SLAM

**SLAM** (Simultaneous Localization and Mapping) is a technique to draw a map by estimating current location in an arbitrary space. 

Now do things step-by-step in different terminals:
```bash
roscore

roslaunch turtlebot3_gazebo turtlebot3_world.launch

roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```

```bash
Control Your TurtleBot3!
  ---------------------------
  Moving around:
          w
     a    s    d
          x

  w/x : increase/decrease linear velocity
  a/d : increase/decrease angular velocity
  space key, s : force stop

  CTRL-C to quit

```

> Practice mapping the environment on your own as you will require it for the assignment.


Now move your bot in entire world and start mapping the world.
It may take a larger time to map the whole world and your PC may also heat up a bit, but don't worry a lot about it.
Sometimes your map will get distorted in between and when you see the terminal, it will show something like this:

 `Scan Matching Failed, using odometry. Likelihood = 0`

This mostly happens when there is some issue with **tf** package. Maneuver your bot multiple times to the entire world until you get a perfect map.
It requires a lot of practice to build a good map, so you may find it difficult to get a good map in first try. 

Once your map is built, i.e. you have mapped the entire world, then for saving this map, execute this:

`rosrun map_server map_saver -f ~/map`

> Remember to save your map after completing mapping, else your entire work will be of no use.

For now, we are done SLAM basics

### Navigation

Just as a demo of how it works, do the following after closing all the previous windows:

```bash
roscore

roslaunch turtlebot3_gazebo turtlebot3_world.launch

roslaunch turtlebot3_gazebo turtlebot3_simulation.launch

roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

```

Now a new window of RViz will popup. **RVIZ** is a ROS graphical interface that allows you to visualize a lot of information, using plugins for many kinds of available topics.

Now till now, you are somewhat familiar with what all the things we are using. Now, turtlebot3 provides its own navigation example. So, first we will simulate it. Close all the terminals except roscore ;)

Now, execute the followin commands:
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

A RViz window will popup, it may take sometime to open. Hold your breathe till that. Now, follow the given below steps.

1. Click on **2D pose estimate** loacted at the top panels and then click on any random point on the map. (The bot will move to that location fastly. It basically sets your initial position)

2. Click on **2D nav goal** loacted side of 2D pose estimate and then click on any point on the graph.(This is your goal location.)

Now, you will see an arrow pointing to your goal location and the bot will start moving slowly towards it. Tha path planning algorithm which we are using here is **DWA Algorithm**. Google it, it's an interesting algorithm which will aviod both static as well as dynamic obstacles.
