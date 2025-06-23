# Gazebo-Ardupilot-SITL-Installation
This is a Beginners guide to install and run Gazebo with ROS and ardupilot SITL for advanced drone simulations, This will also go through VTOL or quadplane simulation and solve possible errors (Note: Iam preparing this doc with the help of Intelligent-Quads, So if you want to simulate other vehicle models, you can visit there Github page
[Intelligent-Quads](https://github.com/Intelligent-Quads/iq_tutorials/tree/master))
# Installation Guide for Gazebo + Ardupilot setup
(You need a Ubuntu 20.04 booted in)</br>(Can use WSL for windows and download Ubuntu 20.04LTS from microsoft store !not recommended)

## GPU setup

If your computer has a GPU like Nvidia Graphics card, follow the instructions given below to accelerate the performance of Gazebo.</br>
(If your computer doesnt have a GPU continue with the Gazebo installation!)

Check GPU info

```bash
lspci | grep VGA
```

This shows which GPU(s) are detected, e.g., NVIDIA, AMD, or Intel.

For NVIDIA GPU

```bash
nvidia-smi
```

Shows real-time GPU usage. Requires NVIDIA drivers to be installed. If it shows memory usage, processes, etc., it’s being used.

Check if GPU is used for rendering

```bash
glxinfo | grep "OpenGL renderer"
```

(You may need to install mesa-utils first: sudo apt install mesa-utils)</br>
If it says something like NVIDIA, AMD, or Intel, that’s your rendering GPU.

Install NVIDIA Drivers (if not installed)

```bash
sudo ubuntu-drivers autoinstall
```

Then reboot:

```bash
sudo reboot
```

Enable Prime GPU Switching:</br>
Ubuntu uses NVIDIA Prime to switch between integrated and dedicated GPUs.

Check current mode:

```bash
prime-select query
```

If your system is currently set to on-demand mode, which means:</br> The integrated AMD or Intel GPU is used by default for most tasks (to save power).

Switch to NVIDIA GPU:

```bash
sudo prime-select nvidia
```

Later if you want switch back to integrated AMD or Intel GPU(to save power):

```bash
sudo prime-select amd
sudo prime-select intel
```
Then reboot for changes to take effect.

Verify GPU in Use
After rebooting:

```bash
glxinfo | grep "OpenGL renderer"
```
Or use:

```bash
nvidia-smi
```
(should show activity if NVIDIA is active)

(Optional Step to check the graphics performance)

Install and run glxgears:
1. Install the package (comes with mesa-utils):

```bash
sudo apt install mesa-utils(If not installed previosly)
```

Run glxgears:

```bash
glxgears
```

Now you are set on the Graphics setup

## Gazebo Installation

Install Dependencies

```bash
sudo apt update

sudo apt install -y git wget curl python3-pip python-is-python3 \
  python3-empy python3-dev python3-opencv python3-numpy \
  python3-setuptools python3-wheel libtool build-essential \
  cmake pkg-config genromfs libncurses5-dev libncursesw5-dev \
  autoconf automake texinfo libftdi-dev zlib1g-dev \
  libxml2-utils xsltproc gawk unzip python3-matplotlib

sudo pip install testresources
```

Setup your computer 

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:

```bash
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:

```bash
sudo apt update
```

Install Gazebo Ubuntu [20.04] :

```bash
sudo apt-get install gazebo11 libgazebo11-dev
```

Check Gazebo if installed:

```bash
gazebo
```

This whould run gazebo and open a empty world.

Install Gazebo plugin for APM (ArduPilot Master) :

```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

build and install plugin

```bash
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```

Set paths for models:

```bash
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

## Install ROS and Setup Catkin

## Install ROS

Setup your sources.list</br>
Setup your computer to accept software from packages.ros.org.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

Desktop-Full Install: (Recommended)</br>Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages

```bash
sudo apt install ros-noetic-desktop-full
```

Environment setup

```bash
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Set Up Catkin workspace

Install necessary dependencies:

```bash
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```

Then, initialize the catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

Dependencies installation: Install mavros and mavlink from source:

```bash
cd ~/catkin_ws
wstool init ~/catkin_ws/src
```

```bash
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y
```

(note:if an error pops that says this "Cannot locate rosdep definition for [geographic_msgs]" then proceed with this steps, if no error come up,continue with "catkin workspace")

Install geographic_msgs manually

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-geographic-msgs
```
Replace $ROS_DISTRO with your ROS version explicitly if needed (e.g., noetic, melodic, etc.).

To check your ROS version:

```bash
echo $ROS_DISTRO
```
If noetic:

```bash
sudo apt-get install ros-noetic-geographic-msgs
```

Update rosdep and initialize again

```bash
sudo rosdep init
rosdep update
```

Then try running your original command again:

```bash
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y
```

or try with version(eg: if noetic)

```bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
```

# catkin workspace

```bash
catkin build
```

Add a line to end of ~/.bashrc by running the following command:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables

```bash
source ~/.bashrc
```

install geographiclib dependancy

```bash
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

Clone IQ Simulation ROS package

```bash
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
```

Inside catkin_ws, run catkin build:

```bash
cd ~/catkin_ws
catkin build
```

update global variables

```bash
source ~/.bashrc
```

## If your Successfull with all these steps you are good to run Gazebo + SITL

In your HOME directory do these steps:

```bash
cd ~
```

Launch Gazebo World:

```bash
roslaunch iq_sim runway.launch
```

Simple shell script file to run SITL

```bash
cp ~/catkin_ws/src/iq_sim/scripts/startsitl.sh ~
```

Run SITL

```bash
~/startsitl.sh
```

Done now you will see a copter in Gazebo world and also the SITL will be connected automatically to Gazebo.

Now if you want to run different models of Aircraft like VTOL for example, follow the steps given below:

## QuadPlane model simulation

```bash
cd ardupilot/Tools/autotest/pysim
nano vehicleinfo.py 
```

Go to a specific line in the Nano editor</br>
Press Ctrl + / and type 278 to go to that line.</br>
You will see something like this:

```bash
   "ArduPlane": {
        "default_frame": "plane",
        "frames": {
            # PLANE
            "quadplane-tilttri": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/quadplane.parm",
                                            "default_params/quadplane-tilttri.parm"],
            },
            #After this you have to past these lines
            "gazebo-quadplane": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/gazebo_quadplane.parm",
            },
```

Then copy the file</br>***iq_sim/scripts/vtol-params/gazebo_quadplane.parm*** to </br>***ardupilot/Tools/autotest/default_params/gazebo_quadplane.parm***, </br>you can do this in your file explorer

## Run Simulation VTOL

First terminal

```bash
roslaunch iq_sim vtol.launch
```

Second terminal

```bash
sim_vehicle.py -v ArduPlane -f gazebo-quadplane  -m --mav10 --console -I0
```

(Note:Possiple errors may pop-up if so follow the below steps)

Error:

```bash
AttributeError: 'NoneType' object has no attribute 'update'
SIM_VEHICLE: MAVProxy exited
SIM_VEHICLE: Killing tasks
```

In your Home directory make shell script file

```bash
cd ~
nano run_vtolsitl.sh
```

Then paste these lines there:

```bash
# Start SITL with no MAVProxy
sim_vehicle.py -v ArduPlane -f gazebo-quadplane -m --mav10 --console -I0 --no-mavproxy &

# Wait a few seconds to let SITL initialize
sleep 5

# Start MAVProxy manually without the waypoint module
mavproxy.py --master=tcp:127.0.0.1:5760 --out=127.0.0.1:14550 --load-module console
```

Make It Executable

```bash
chmod +x run_quadplane.sh
```

Run It

```bash
./run_quadplane.sh
```

This will solve the error.




