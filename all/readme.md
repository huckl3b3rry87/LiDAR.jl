File List
===

CMakeList.txt
---

We use cmake to build a project.

[Here](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i6) is an instruction about building a project with Gazebo and ROS libraries.

echonode.py
---

ROS script, for testing purpose.

It is used to test the ROS Julia interface. Follow [here](https://github.com/jdlangs/RobotOS.jl) to set it up.

Some details about ROS are avaliable [here](http://wiki.ros.org/ROS/Tutorials#Beginner_Level).

test.jl
---

Edit this file to control ROS (and Gazebo) with Julia code.

Search `TODO` for more information.

Setup ROS in Julia:

	Pkg.add("RobotOS")


vel.cc
---

A simple message sender, for testing purpose.


How to use it?

First: Open a terminal and type roscore
Second: Open another terminal and type python echonode.py to set all the nodes
Third: julia handler.jl to set all the julia controls
Last: cmake . && make && gazebo velodyne.world to create the gazebo simulation


velodyne.world
---

A Gazebo world file, for testing purpose.

velodyne_hdl32.tar.gz
---

An example object of Gazebo.

You may extract it into `~/.gazebo/models`.

The object is created following Gazebo's official tutorial.

The file `model.sdf` here is different from the original one. See the `joint` node, we changed its type to "prismatic". You can modify the joint's behavior into what you actually want.

velodyne_plugin.cc
---

A Gazebo plugin to pass messages from ROS to Gazebo.

Function `OnMsg` is the message handler. Search `TODO` in the code for more information.

[Here](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5) is a tutorial about Gazebo plugins.
