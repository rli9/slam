echo starting home setup env
source /home/flex/ros_catkin_ws/devel/setup.bash
echo [ok]
echo starting car_control_manual env
source /home/flex/slam/src/car_control_manual/scripts/prepare.sh
echo [ok]

echo Running car_control_manual car control script
python /home/flex/slam/src/car_control_manual/scripts/car.py
echo [ok]

echo Running cherokey_car car control script
python /home/flex/slam/src/cherokey_car/scripts/car.py
echo [ok]
