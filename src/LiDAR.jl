#!/usr/bin/env julia
module LiDAR

using RobotOS
@rosimport std_msgs.msg: Float64MultiArray
@rosimport geometry_msgs.msg: Point
rostypegen()
using std_msgs.msg       # subscribe to topic
using geometry_msgs.msg

# set up roscore
#@pyimport roslaunch

# setup for the Python script
using PyCall
@pyimport imp
Node=imp.load_source("Node",Pkg.dir("LiDAR/src/NodeModule/Node.py"));

include("utils.jl") # functions

function __init__()
  addprocs(3);

  # start up a roscore
  #roslaunch.roscore

  # run the ptthon script Echo.py to start ROS node to cummunicate between julia and Gazebo
  node=remotecall(Node[:main],2,());
  println("ROS node Loaded!");

  # run the julia script handler.jl to get point cloud information from Gazebo
  data=remotecall(main,3,());
  println("Ready to Collect Data From Gazebo!");

  # start Gazebo
  gazebo=remotecall(run(`gazebo velodyne.world`),4,());
  println("Gazebo LiDAR model running!");

end

end # module
