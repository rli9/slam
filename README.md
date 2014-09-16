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

`$ cd  ~/catkin_ws/src/rgbdslam_v2-hydro/
 $ mkdir build
 $ cd build
 $ cmake ..
 $ make -j8`

##GPU OPTION##

if there is no a good gpu in your machine.

change USE_SIFT_GPU to 0 in CMkeLists.txt

#Install OpenNI package#

`$ sudo apt-get install ros-hydro-openni-camera
 $ sudo apt-get install ros-hydro-openni-launch
 $ roslaunch openni_launch openni.launch`

##Configure openni node##

Edit launch file -- openni.launch in /opt/ros/hydro/share/openni_launch/launch/openni.launch

add the following lines to change depth_mode and rgb_mode (lower the resolution of rgb image and depth image to 320 * 240 for performance):

`
<param name="/$(arg camera)/driver/image_mode" value="5" />
<param name="/$(arg camera)/driver/depth_mode" value="5" />
`

insert them after the line where argument "depth_frame_id" is set.

##Run the opnni node##

Before running the openni node, make sure environment variable -- ROS_MASTER_URI was set correctly, and roscore is running at the master host.

type the command below:

`$ roslaunch openni_launch openni.launch`


#Setup Yocto + ROS on Galileo board#

refer to http://wiki.ros.org/IntelGalileo/How%20to%20install%20ROS%20Hydro%20on%20Intel%20Galileo

Some instructions in this section: http://wiki.ros.org/IntelGalileo/IntelGalileoYoctoSetupOnGalileo may cause problems when running roscore on Galileo board. 

They are:

_TIMEOUT_MASTER_START = 20.0 ,  in /usr/lib/python2.7/site-packages/roslaunch/launch.py

this value should be set to a much larger value than 20, such as 500.

and the instruction "$ mv /usr/lib/ros* /opt/ros/lib "  should not be executed.
