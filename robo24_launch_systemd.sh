#!/bin/bash

echo "robo24_systemd.sh launch robo24 ##STARTING##" | systemd-cat -p info

cd /home/mike/ros2_ws

exec 'su mike -c "source install/setup.bash;ros2 launch robo24_stuff robo24_bringup_launch.py "'

while :
do
echo "robo24_systemd.sh launch robo24 ##RUNNING##" | systemd-cat -p info
sleep 60
done
