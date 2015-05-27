#!/bin/bash

source /home/flex/slam/devel/setup.bash

chmod +x /home/flex/slam/src/cherokey_car/scripts/car.py
rosrun cherokey_car car.py &
sleep 5

rostopic pub -1 /car_control std_msgs/String "direction follow, object_x 640, object_y 320"
rostopic pub -1 /car_control std_msgs/String "direction follow, object_x 390, object_y 190"
rostopic pub -1 /car_control std_msgs/String "direction follow, object_x 640, object_y 190"
rostopic pub -1 /car_control std_msgs/String "direction follow, object_x 640, object_y 320"

