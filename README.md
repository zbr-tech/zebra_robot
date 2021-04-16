### Environment
- ubuntu 18.04 LTS
- ros melodic
- python 2.7.17

### Install
- git clone this in {your workspace}/src/  
- Please rewrite eigen path in quadruped_ctrl/Cmakelist.txt
- sudo apt-get install ros-melodic-joy
- sudo apt-get install ros-melodic-joystick-drivers
- pip install pyquaternion pcl pybullet
- pip install numpy --upgrade
- sudo apt-get install can-utils 
- pip install python-can 
- pip install pyserial 

### Warning
#### Memo for real robot
- Please set each motor's CAN ID as 0~12 (you should use serial setting command )
- Please set CAN-CAN mbed's CAN ID as 13~ (you should change program of mbed IDE)


### Running
- Please see params in quadruped_ctrl/config
- sudo chmod 666 /dev/ttyACM0
- roslaunch quadruped_ctrl quadruped_ctrl.launch
- roslaunch gamepad_ctrl gamepad_ctrl.launch

### Reference
- https://python-can.readthedocs.io/en/2.1.0/installation.html
- https://github.com/INNO-MAKER/usb2can
- https://python-can.readthedocs.io/en/2.1.0/installation.html
- https://os.mbed.com/users/benkatz/code/CanMaster/




### Memo
run the gamepad node to control robot:
```
roslaunch gamepad_ctrl gamepad_ctrl.launch
```
run the controller in simulator:  
```
roslaunch quadruped_ctrl quadruped_ctrl.launch
```

switch the camera on / off:
camera set ```True``` or ```False``` in ```config/quadruped_ctrl_config.yaml```, then launch the rviz to see the point cloud:
```
roslaunch quadruped_ctrl vision.launch
```

also can switch the gait type:  
```
rosservice call /gait_type "cmd: 1"
```

gait type:
```
0:trot
1:bunding
2:pronking
3:random
4:standing
5:trotRunning
6:random2
7:galloping
8:pacing
9:trot (same as 0)
10:walking
11:walking2
```


### Terrain
you can modify the ```config/quadruped_ctrl_cinfig.yaml/terrain``` to deploy different terrains, there are four terrains supported in the simulator now, for example:
```
"plane"
"stairs"
"random1"
"random2"
"racetrack"
```