Car Control Manual Module


Running Step


On host:

1. roscore
2. sh repub.sh - For decompile ros image and republish as topic /camera/decomp_image
3. python scratch/scratch.py server   -  start host server for listening scratch msg and publish command to car
4. python scratch/scratch.py client   - start host client for watching *.sb2 file created and send command to server
5. python scripts/image_subscriber.py - (optional) show video from phone(republish.sh)



On Ros Car:

1. sudo su - change user to root for write to pins
2. source scripts/prepare.sh   - for export some var
3. python scripts/car.py - starting



Using scripts:

On host:
1. source /home/flex/slam/src/car_control_manual/start.sh
2. python scratch/scratch.py server
3. python scratch/scratch.py client

On Ros Car
1. sudo su
2. source src/car_control_manual/car_start.sh 
