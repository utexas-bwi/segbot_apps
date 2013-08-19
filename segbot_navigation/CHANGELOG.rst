^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2013-08-12)
------------------
* added a real world navigation file
* updated and cleaned up visualization config
* cleaned up launch and configuration files. closes `#6 <https://github.com/utexas-bwi/segbot_apps/issues/6>`_
* removed confusing map_namespace parameter
* added map_topic parameter for multi-robot scenarios

0.1.3 (2013-07-16)
------------------
* added missing dependencies (closes `#4 <https://github.com/utexas-bwi/segbot_apps/issues/4>`_). Cleaned up package and cmake files.
* changed costmap visualization to occupancy grid in rviz configuration

0.1.2 (2013-07-13)
------------------
* removed dependency on navigation meta-package. progress towards `#3 <https://github.com/utexas-bwi/segbot_apps/issues/3>`_

0.1.1 (2013-07-10)
------------------
* navigation has been released as a system dependency

0.1.0 (2013-06-28)
------------------
* removed redundant doc file
* uncommented runtime launch dependencies on navigation and eband_local_planner as they have not been released into hydro yet
* catkinized segbot_apps
* updating the eband visualization configuration for hydro-devel
* increased footprint size to produce an inscribed radius of 0.3
* fixed footprint location while waiting for `ros-planning/navigation#63 <https://github.com/ros-planning/navigation/issues/63>`_ to be fixed
* commenting out hydro-devel navigation test code. This should not be checked in until navigation through hydro-devel is fixed
* changes to prepare for the catkinization of eband_local_planner against hydro-devel in navigation
* some improvements to navigation
* fixed a bug in the eband trajectory controller
* in-place rotation at goal now supported
* merged goal tolerance parameters between local planner and trajectory controller.
* add launch for e-band navigation
* fixed for the regular nav stack launch file as well. closes `#1 <https://github.com/utexas-bwi/segbot_apps/issues/1>`_
* hmm not sure why this file was here
* fix for the eband costmap having an incorrect topic. `#1 <https://github.com/utexas-bwi/segbot_apps/issues/1>`_
* updated launch file to use any visualization configuration + reorganized eband configuration file
* checking in new parameters for the eband local planner
* inital differential drive trajectory controller - looks pretty good. needs a bit more code improvent, dynamic reconfigure and stricter obstacle testing
* removed some unnecessary launch files and added an rviz configuration + launch file for testing autonomous navigation
* basic amcl + move base demo works (but is not very good)
* removed old ens basement maps from the repo
* removed joy gmapping file - joystick control not directly supported
* removed redundant sensor files (moved to segbot_sensors)
* initial port of of navigation and controller code from the svn repository
