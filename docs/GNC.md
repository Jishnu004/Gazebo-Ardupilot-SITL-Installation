## This is Guide to start Guidance Navigation and Control(No error 2025)

## Make sure you have a text editor
As this is the first tutorial that we will be coding please make sure you have a text editor. My prefered text editor is sublime. You can download it by running the below commands
```
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text
```

## Clone the IQ GNC ROS package

First, we will clone the IQ GNC ROS package. This ROS package comes with my GNC API that will make scripting the drone easy. It will also come with a worked out solution to this tutorial 
```
git clone https://github.com/Intelligent-Quads/iq_gnc.git
```

## Build code
```
cd ~/catkin_ws
catkin build
source ~/.bashrc
```
Navigate to your MAVROS launch directory (usually inside your catkin workspace):

```
cd ~/catkin_ws/src/mavros/mavros/launch
nano apm.launch
```

## Correct the fcu_url and startsitl Format

Change it to this:

```
<arg name="fcu_url" default="udp://:14551@127.0.0.1:14551" />
```

Change your SITL starts script:

```
cd ~
nano startsitl.sh
```

and paste this

```
#!/bin/bash
cd ~/ardupilot/ArduCopter/ && \
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

## Final Corrected Setup

### Terminal 1 (Gazebo):

```
roslaunch iq_sim runway.launch
```
### Terminal 2 (ArduPilot SITL):

```
cd ~
./startsitl.sh
```

### Terminal 3 (MAVROS - Modified fcu_url):

```
roslaunch mavros apm.launch fcu_url:="udp://:14551@127.0.0.1:14551"
```
or just run 

```
roslaunch mavros apm.launch
```
both should work

### Terminal 4 (Run IQ_GNC):

```
rosrun iq_gnc square
```

## Expected Success Indicators

```
rostopic echo /mavros/state shows:
```
Run the above to check if you are connected as shown below</br>before running mavros

```
connected: True
armed: False
mode: "GUIDED"
```

NOTE** you can tile gnome terminals by pressing `ctrl + shift + t`

Finally run the mission by changing the flight mode to guided in the MAVproxy terminal by running 
```
mode guided 
```

### This should solve most of the problem
