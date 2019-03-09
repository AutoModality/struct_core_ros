# Structure-Core-ROS
ROS nodelet for Occipital Structure Core 

**System Requirements**

This code has been tested on Ubuntu 16.04 x86_64 and arm64 using the StructureSDK-CrossPlatform-0.6.1-2.0 package (firmware version 0.9.3).

**Prerequisites**

* Install and build the linux version of the Structure SDK as documented in the StructureSDK-CrossPlatform-0.6.1-2.0 package
* It is strongly recommended to run the Recorder sample application on the target system to confirm everything is working
* libStructureLinux.so from the Occipital release should be in the library path (e.g. /usr/local/lib)
* Header files from the Occipital release should be copied into the include/ST directory

**Running**

The code is structured as a ROS nodelet. A launch file (sc.launch) is provided that launches both the manager and the nodelet. The launch file also loads a yaml file (sc.yaml) that controls the configuration.

`$ roslaunch struct_core_ros sc.launch`

