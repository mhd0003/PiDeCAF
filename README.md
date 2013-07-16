Decentralized Collision Avoidance Framework for RPi
=======================================

This repository contains a package for ROS groovy that implements a decentralized collision avoidance framework for use on a Raspberry Pi.  The framework was developed as a part of the Auburn University ATTRACT program in the summer of 2013. Visit our website at https://sites.google.com/site/auburnuavreu2013team1/home for more details. The following documentation will be split up into two major components:

1. Hardware - Setting up the Pi, subsystems on the plane

2. Software - How our framework works, how to slot in another algorithm


Hardware
=======




Software
=======






![My image](https://0d9aa83c-a-62cb3a1a-s-sites.googlegroups.com/site/auburnuavreu2013team1/PiDeCAF.png?attachauth=ANoY7cplKmbZcYVVpBwpfJMWCUCy5ydfAghsI4wjWjYU3Uwm4khZ8q9-inrlYKmuPjDGuM5nFaXXuXm8w0ZLKmSXIH6D9YPEoyJII-g9FNV6azWnJHuqObGu4BsK7wwUFM3705k8lTw32SL1uQdXl07nLG9sRNr5l37N5BtTUljClmrxa3em3EdNXNFKoIOjQIpn47CUSJJRgi6V1iqYWz8e0sOQysAcxQ%3D%3D&attredirects=0)



















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

