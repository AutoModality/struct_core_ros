# Structure-Core-ROS
ROS nodelet for Occipital Structure Core 

**System Requirements**

This code has been tested on Ubuntu 16.04 x86_64 and arm64 using the PerceptionEngine-Linux-Streaming-0.5.2 package (firmware version 0.8.6).

**Prerequists**

* StructureLinux.so should be in the library path (e.g. /usr/local/bin).
* Non-root users should be granted permission to use the Structure Core. 

It is strongly reccomended to build and run the Recorder sample application on the target system to confirm everything is working.

**Running**

The code is structured as a ROS nodelet. A launch file (sc.launch) is provided that launches both the manager and the nodelet. The launch file also loads a yaml file (sc.yaml) that controls the configuration.

`$ roslaunch struct_core_ros pc.launch`

