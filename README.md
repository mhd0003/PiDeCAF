Decentralized Collision Avoidance Framework for RPi
=======================================

This repository contains a package for ROS groovy that implements a decentralized collision avoidance framework for use on a Raspberry Pi.  The framework was developed as a part of the Auburn University ATTRACT program in the summer of 2013. Visit our website at https://sites.google.com/site/auburnuavreu2013team1/home for more details. The following documentation will be split up into two major components:

1. Hardware - Setting up the Pi, subsystems on the plane

2. Software - How our framework works, how to slot in another algorithm


Hardware
=======




Software
=======



















Helpful commands:

CA_ON
rostopic pub /gcs_commands au_uav_ros/Command '{seq:  0, stamp: 10 ,  frame_id: "o"}' 255 0 2 2 999 555 1 1
CA_OFF
rostopic pub /gcs_commands au_uav_ros/Command '{seq:  0, stamp: 10 ,  frame_id: "o"}' 255 0 2 2 999 444 1 1
STOP
rostopic pub /gcs_commands au_uav_ros/Command '{seq:  0, stamp: 10 ,  frame_id: "o"}' 255 0 2 2 999 777 1 1

Publishing goal waypoint
rostopic pub /my_mav_telemetry au_uav_ros/Telemetry '{seq:  0, stamp: 10 ,  frame_id: "o"}' 26 0 X X X 200 200 200 3 3 4 5
rostopic pub /my_mav_telemetry au_uav_ros/Telemetry '{seq:  0, stamp: 10 ,  frame_id: "o"}' 26 0 32.602597 -- -85.488859 100 200 200 200 3 3 4 5

