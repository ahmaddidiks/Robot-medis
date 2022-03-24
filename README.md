# Robot Medis
<img src="https://github.com/ahmaddidiks/Robot-medis/blob/master/assets/rviz.png"/>  


# Yey! Now this project support MoveIt 1
https://user-images.githubusercontent.com/23350636/159868220-f4da2066-9d9f-4e7d-b0a5-7bd5f39748fb.mp4

## ROS Packages for Robot Medis
### 1. [Install and setup ROS noetic on your environment](http://wiki.ros.org/noetic/Installation)

### 2. [Install the Python Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

### 3. Make your project folder and clone this Github repository, then build the ROS pakage
```
cd ~
mkdir robot_medis_ws
cd robot_medis_ws
git clone https://github.com/ahmaddidiks/Robot-medis.git
mv Robot-medis src
catkin_make
```

### 4. Source ROS for current project instead of 'setup.bash' forever LOL, [Please read the thread](https://answers.ros.org/question/206876/how-often-do-i-need-to-source-setupbash/)
```
echo 'source ~/robot_medis_ws/devel/setup.bash' >> ~/.bashrc
```

### 5. Test the Robot with forward kinematics
```
roslaunch robot_launch forward_kinematics.launch
```

### 6. Test Robot Simulation with MoveIt
```
roslaunch moveit_config demo.launch
```
