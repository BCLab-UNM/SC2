# SC2 Getting Started
Swarmathon Team Code for the NASA Space Challenge 2 Competition

## Hardware Requirements
NVidia Graphics Card

## Hardware Recommendations
* 2.6 Ghz I-7 Processor
* 8 Gb Ram (more is better)

## Software Requirements
* Operating System: [Ubuntu 18.04](https://releases.ubuntu.com/18.04.4/)
* Docker Nvidia Support (see below for specific instructions) [Docker General Instructions](https://docs.docker.com/engine/install/ubuntu/#installation-methods
)
* Robot Operating System [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
  * Getting Started with ROS in C [Guide](https://www.cse.sc.edu/~jokane/agitr/agitr-letter.pdf)

## Software Installation
Setup Ubuntu 18.04 as the operating system. It is not recommended to install within a virtual machine.  

#### Update and Upgrade current software packages
```
sudo apt-get update
sudo apt-get upgrade
```
#### Change to the official Nvidia Driver
* Left click Activities in the upper left from the desktop
* Search for Software & Updates and select the file
* Left click the Additional Drivers Tab, it will take a moment to search for the drivers
* Select the option for the Nvidia driver metapackage <version> (proprietary, tested)
* Apply Changes
* Note, if you are experiences Ubuntu locking up, do this first.

#### Install Docker
If installing docker for the first time (if not, you made need to upgrade or remove and install, see general docker instructions above).  
Make sure the Team Leader has added you to the Docker Hub
```
sudo apt install docker.io
```
#### Update Nvidia drivers to work with Docker
Install Curl
```
sudo apt install curl
```
Add repositories
```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```
Install Nvidia Docker support packages
```
 sudo apt update 
 sudo apt install nvidia-container-toolkit
```
Restart Docker daemon with Nvidia support
```
sudo systemctl restart docker
```
#### Pull docker image
```
sudo usermod -aG docker ${USER}
```
* If you are asked for a password here, this is your sudo password
* Open a new terminal window and type
```
docker login -u swarmathon
```
* Use the SC2 swarmathon team password

#### Setup gitlab ssh keys
We will use ssh keys to access the gitlab submodule. Follow these instructions: https://docs.gitlab.com/ee/ssh/

#### Clone the Github repository
```BASH
git clone git@github.com:BCLab-UNM/SC2.git #ssh
#or
git clone https://github.com/BCLab-UNM/SC2.git #https

cd SC2 # enter the repo directory
git submodule update --init --recursive #After cloning this repository you need to initialize and update the submodule(s)
```

#### Download files and start simulation
* In terminal move to the srcp2-competitors folder (default command below, if you put it somewhere else, go there)
```
cd srcp2-competitors
```
* Download and simulation
```
./docker/scripts/launch/roslaunch_docker --run-round 1
```
* Download may take a while, to watch, start a new terminal window and type
```
tail -f /tmp/srcp2/simulation/docker.log
```
* When completed a Gazebo window will open with base and robots loaded.
* Press the single arrow on the bottom of the Gazebo screen (most left arrow) to start running

#### Getting ROS topics from container
Make sure ROS (Melodic) is installed, if not see [Software Requirements, ROS section](#software-requirements)  
Note: There are a lot of topics
```
ROS_MASTER_URI=http://localhost:11311 

source ./docker/scripts/srcp2_setup.bash
rostopic list
```
