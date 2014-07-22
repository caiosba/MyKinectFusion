This is a port of Márcio's KinectFusion to Linux. Tested on Debian Squeeze and Ubuntu 12.04.
Please remember that you must have a graphics card with CUDA support and a fair amount of memory.

Installation on Debian Squeeze: Install all required libraries and run "make". Then execute "KinectFusion".

Installation on Ubuntu 12.04: Run the install.ubuntu.sh script, then execute "KinectFusion". If you have some trouble on
running "make", try removing /usr/include/pcl/gpu, running make, putting the "gpu" directory back and running "make" again.

You may also need to set the path to libcuda.so, which can change from system to system depending on the driver version.

This code can read transformations from YML file, augmented reality glasses and also use a .oni file as input stream instead
of the live stream from a real Kinect. But you can disable one or all of them: In order to disable...

* YML files: Turn variable "enableCalibrationFile" into "false" around line 36 on src/Reconstruction.cpp
* Augmented reality glasses: Turn all "enable" variables into "false" around line 32 on src/Reconstruction.cpp
* ONI file: Variable "live", around line 88 of file src/main.cpp must be "true"  

On the other hand, if you want to use a calibration file, it should be named "calibration.yml" and be located on the root of this
application. If you want to use an .oni file instead of a live stream, it should be named "stream.oni" and be located on the root
of this repository. If you want to use augmented reality glasses, it should write to an UDP socket at port 6001 with format "G Y P R",
where "G" is a static char that represents the command "glasses", and Y, P and R are integer values that represent the degrees (not
radians) of yaw, pitch and roll, respectively. You can change the socket port by changing variable "socketPort" around line 43 on
file src/main.cpp.

Todo:

* Improve stereo calibration

References:
https://github.com/MarcioCerqueira/MyKinectFusion
