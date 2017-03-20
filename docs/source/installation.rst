Installation on Ubuntu 16.04
##################################

1. ``julia`` programs needed
******************************

get and build:

::

  Pkg.add("RobotOS")
  Pkg.add("PyCall")
  Pkg.build("RobotOS")
  Pkg.build("PyCall")

2. Gazebo
*************
Install:
::

  curl -ssL http://get.gazebosim.org | sh

Run:
::

  gazebo

More `information is here <http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install>`_


3. ``ROS``
************

Follow the instructions `here <ttp://wiki.ros.org/kinetic/Installation/Ubuntu>`_

Note: In step # 1.4, run this one (the full one seems to be broken):

::

  sudo apt-get install ros-kinetic-desktop


4. Initialization of Package
******************************

4.1. Run ``cmake .`` and ``make``
--------------------------------
A. open another terminal -> ``ctr``+``shift``+``t``

#. navigate to the folder ``.scr/c_code``

#. in the terminal type ``cmake .`` and hit ``enter``

#. in the terminal type ``make`` and hit ``enter``

4.2. Set path for gazebo plugins
----------------------------------
Not sure if this needs to be done:

A. in the terminal type (only should have to do during initialization):
::

  export GAZEBO_PLUGIN_PATH=$HOME/gazebo_plugin_tutorial/build:$GAZEBO_PLUGIN_PATH

  TODO: need to change path


4.3 Get the ``velodyne_hdl32`` model for ``Gazebo``
----------------------------------------------------

Either:

A. Fork the gazebo_models database by visiting https://bitbucket.org/osrf/gazebo_models/fork.

**OR**

B. Just copy the folder ``./LiDAR/scr/gazebo/gazebo_models`` into ``$HOME.gazebo/models/``

.. sidebar:: Note

  press ``ctr`` + ``h`` to show hidden folders (..like .gazebo)


Potential Issues
****************

1. running ``using RobotOS`` in ``julia`` -> fails
-----------------------------------------------------

This may be due to path issues was fixed `here <https://github.com/jdlangs/RobotOS.jl/issues/23>`_.

  * Additional `info here <http://answers.ros.org/question/39657/importerror-no-module-named-rospkg/>`_
