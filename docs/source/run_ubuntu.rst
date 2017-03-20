****************
Getting Started
*****************
* NOTE: the following is designed for use with Ubuntu 16.04


Basic Usage
############

1. Start ``roscore``
**********************

.. sidebar:: Background Information

  `roscore <http://wiki.ros.org/roscore>`_ is a collection of nodes and progams that are needed to use a ROS system.
    * A roscore must be running for ROS nodes to talk

  Linking GAZEBO to ROS
    * GAZEBO can be linked to ROS using `this link <http://gazebosim.org/tutorials?tut=ros_overview>`_.

  Linking ROS to julia
    * ROS can be linked to julia using `this package <http://www.ros.org/news/2014/12/announcing-robotosjl-an-interface-to-the-julia-language.html>`_.


A. open a new terminal -> ``ctr``+``alt``+``t``

#. in the terminal type ``roscore`` and hit ``enter``


2. Make a ROS node in ``python`` that communicates to a ``julia`` node
***********************************************************************

A. open another terminal -> ``ctr``+``shift``+``t``

#. navigate to the folder containing ``echoinode.py`` (i.e. ``./scr``)

#. in the terminal type ``python echonode.py`` and hit ``enter``


3. Run a ``julia`` function that can communicate with the ``LiDAR`` model
****************************************************************************

A. open another terminal -> ``ctr``+``shift``+``t``

#. navigate to the folder containing ``handler.jl`` (i.e. ``./scr``)

#. in the terminal type ``julia`` and hit ``enter``

#. in the ``julia`` type:
::
  include("handler.jl")


4. run ``gazebo`` model of the LiDAR
**************************************

A. navigate to the folder ``.scr/c_code``

#. in the terminal type ``gazebo velodyne.world`` and hit ``enter``

  * A Gazebo gui should appear on the screen
