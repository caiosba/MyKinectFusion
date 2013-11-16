This is a port of Márcio's KinectFusion to Linux. Tested on Debian Squeeze and Ubuntu 12.04.
Please remember that you must have a graphics card with CUDA support and a fair amount of memory.

Installation on Debian Squeeze: Install all required libraries and run "make". Then execute "KinectFusion".

Installation on Ubuntu 12.04: Run the install.ubuntu.sh script, then execute "KinectFusion". If you have some trouble on
running "make", try removing /usr/include/pcl/gpu, running make, putting the "gpu" directory back and running "make" again.

You may also need to set the path to libcuda.so, which can change from system to system depending on the driver version.

References:
https://github.com/MarcioCerqueira/MyKinectFusion
