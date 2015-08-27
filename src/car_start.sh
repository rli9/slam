echo "source ros_catkin_ws setup.bash..."
source /home/flex/ros_catkin_ws/devel/setup.bash
if [ $? -ne 0 ];then
  echo "error: source ros_catkin_ws/devel/setup.bash failed"
  exist 1
fi

echo "source car control manual environment..."
source /home/flex/slam/src/car_control_manual/scripts/prepare.sh
if [ $? -ne 0 ];then
  echo "error: source car_control_manual/scripts/prepare.sh failed"
  exist 1
fi

echo "Running car control module..."
python /home/flex/slam/src/car_control_manual/scripts/car.py
if [ $? -ne 0 ];then
  echo "python car_control_manual/scripts/car.py failed"
  exist 1
fi

echo "Running tracking control module..."
python /home/flex/slam/src/cherokey_car/scripts/car.py
if [ $? -ne 0 ]; then
  echo "python cherokey_car/scripts/car.py failed"
  exist 1
fi

echo "Waiting for GIPO Response.."
exist 0
