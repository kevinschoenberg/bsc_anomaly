# bsc_anomaly

Terminal 1:

cd sim_ws/src/f1tenth_gym_ros/
sudo docker-compose up


Site:
http://localhost:8080/vnc.html


Terminal 2:

cd sim_ws/
sudo docker exec -it f1tenth_gym_ros_sim_1 /bin/bash


source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py


Terminal 3:

cd sim_ws/
ros2 run bsc_anomaly LSDNode


Build:
cd sim_ws/
sudo colcon build
