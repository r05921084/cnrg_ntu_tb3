IPEM module for ROS
===================
This is a ported version of a module in IPEM toolbox from Matlab and C to python module and ROS node.
This package provides computational model for Auditory Peripheral Module (APM) in human auditory system.
The node takes binaural audio signal (raw pcm) and converts it into Auditory Nerve Image which corresponds to the neural activities at peripheral auditory system.

IPEM Toolbox
------------
> An open source auditory toolbox for perception-based music analysis.
<https://github.com/IPEM/IPEMToolbox>

IPEM_pipe module
------------------------------------------

###### IPEM\_pipe\_reactor.py

This module bases on the core module (APM) of IPEM Toolbox.
We modified the source code (in C language) to better meet our purpose and wrapped it in python.

1. No extra file will be generated during the excution.
2. Some code pieces are rearranged for better readability and understading.
3. The I/O of the module has been swapped to named pipes for continous streaming operation.
4. A python based class wrapper is created for easy developing and usage.
5. Still improving...


IPEM_pipe_reactor node
--------------------------

###### IPEM\_pipe\_reactor.py

A ROS node subscribes to topic `audio_stream_raw`, processes it with IPEM\_pipe module and publishes the result to `audio_stream_ani_L` and `audio_stream_ani_R`.

Usage
-----
```$roslaunch roslaunch ipem_module basic.launch```
