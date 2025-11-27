# F1Tenth-AndresLuque
Proyecto del primer parcial
Instructions to run the project
1. F1Tenth simulator

Before starting it's necessary that you install the simulator. You can find a tutorial on this repository: https://github.com/widegonz/F1Tenth-Repository
2. Clone repository

Now that you already have the dependencies, and all, you can clone this actual repository in /home.

git clone [https://github.com/AndresLuqueP/F1Tenth-AndresLuque.git]

2. Access to the repository location

cd F1Tenth-Repository

3. Compile the workspace

colcon build
source install/setup.bash

4. Run the simulator and the controller

First, we launch the simulator. You must see the enviroment in Rviz.

ros2 launch f1tenth_gym_ros gym_bridge_launch.py

Then, there's going to be one controller that you can run.

# This one is a simple FTG algorithm, used to explore the map without obstacles
ros2 run andres_project FollowTheGap_node.py
5. Change the map (Optional)

The map that shows up by default is going to be the Spielberg obstacle map, so if you want to see the Spielberg map you must do some changes in the file 'sim.yaml'

cd F1Tenth-Repository/src/controllers/f1tenth_gym_ros/config
nano sim.yaml
#On line 45 change this
map_path: '/home/maria/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Spielberg_obs_map'
#For this
map_path: '/home/maria/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Spielberg_map'

Results
You will see the car moving around the selected map
