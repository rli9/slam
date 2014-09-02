slam
====

#Install rgbdslam in hydro (ubuntu 12.04)#


##Prepare Workspace##

source /opt/ros/hydro/setup.bash

mkdir -p ~/rgbdslam_catkin_ws/src

cd ~/rgbdslam_catkin_ws/src

catkin_init_workspace

cd ~/rgbdslam_catkin_ws/

catkin_make

source devel/setup.bash


##Get RGBDSLAM##

cd ~/rgbdslam_catkin_ws/src

wget -q http://github.com/felixendres/rgbdslam_v2/archive/hydro.zip

unzip -q hydro.zip

cd ~/rgbdslam_catkin_ws/

##Install##

rosdep update

rosdep install rgbdslam

catkin_make
 
##ERRORs and Solution##

1. when excute catkin_make follow steps above,there will be errors like" Error: no such instruction: `vfmadd312ss (%r15),%xmm0,%xmm1'"

solutions:

1. cd  ~/catkin_ws/src/rgbdslam_v2-hydro/

2.mkdir build

3. cd build

4. cmake ..

5. make -j8

##GPU OPTION##

if there is no a good gpu in your machine.

change USE_SIFT_GPU to 0 in CMkeLists.txt

#Install freenect stack or OpenNI camera package on ROS hydro#

##Install Freenect stack##

$ sudo apt-get install ros-hydro-freenect-stack

##Launch freenect node##

$ roslaunch freenect_stack freenect.launch

##Install OpenNI camera##

$ sudo apt-get install ros-hydro-openni-camera

##Launch openni node##

$ roslaunch openni_launch openni.launch