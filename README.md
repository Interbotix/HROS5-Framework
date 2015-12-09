HROS5-Framework
===============

HROS5-Framework, based on Darwin-OP framework, is a collaborative software project based on the open source Darwin-OP Framework, with the intent to continue development, implement an API to expose higher level functions of the framework, and develop additional features. 

This Framework was intended to be deployed on the [HR-OS5 Humanoid Research Robot](http://www.trossenrobotics.com/HR-OS5).

**Please visit the [HROS5-Framework Github Wiki](https://github.com/Interbotix/HROS5-Framework/wiki) for more information on using this software and robotics platform.**

Contributions & authoring to codebase provided by Farrell Robotics, Interbotix, DresnerRobotics, Zerom, ROBOTIS, KevinO, KurtE. A special thanks to Farrell Robotics, a massive contributor and resource for humanoid robots & additions to the Darwin-OP framework. 

All code within project is GPL GNU v3.

Original source code can be found here:
https://sourceforge.net/projects/darwinop/

Maintainer
=================
* Name: Andrew Dresner
* Email: andrew@trossenrobotics.com
* Github: DresnerRobotics
* Website: http://www.trossenrobotics.com

Notes
==================
* Operating systems supported & verified: Ubuntu 12.04 & 14.04 LTS, Yocto Poky 1.6 OpenEmbedded Linux
* This modified framework now only supports the Arbotix-Pro subcontroller, support of the Arbotix Pro has been discontinued. 
* ps3_demo - Triangle button initializes into Walk-Ready, which can then be used as a starting point for action motions. PS3 button layout in wiki.
* ps3_demo does not currently work for BlueZ5. For BlueZ4, sixad -s must be launched at startup (ie: rc.local). Requires pairing via sixpair. See Wiki for more information. PS3controller library authored by Farrell Robotics.
* rme (robot motion editor) is an improved version of action_editor,  authored by Farrell Robotics. Additional features such as individual limb on/off torque control implemented (see wiki). Robot MUST be in sitting position and/or spotted when launching rme, as servos go into low-torque mode upon launching rme followed by the robot sitting down. Currently rme is the only way to create motion pages/files.

TODO
=================
* Test/Finalize api_wrapper & node.js server
* ROS integration


