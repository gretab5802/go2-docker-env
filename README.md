# Unitree Go2 Docker Environment and Remote Control
Steps to configure Docker container for Ubuntu 20.04 and connect to a Unitree Go2 remotely through network connection

## Resources:
1. [Unitree GitHub (useful for installing libraries and dependencies)](https://github.com/unitreerobotics/unitree_ros2?tab=readme-ov-file)
2. [Unitree Development Guide (useful for network connection)](https://support.unitree.com/home/en/developer/Quick_start)
3. [Installing ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
4. [Unitree Go2 Tutorials](https://www.docs.quadruped.de/projects/go2/html/go2_driver.html)
5. [Installing Docker on Ubuntu 22.04 or 24.04](https://docs.docker.com/engine/install/ubuntu/)
6. [Go2 Controller info](https://shop.unitree.com/products/go2-controller)

## Prequisites:
### Hardware needed:
* Unitree Go2
* Ethernet cable + ethernet to USB adapter
* Laptop with Ubuntu (this was tested on Ubuntu 24.04)

# Let's begin!
## On Laptop
We are going to make a directory in root to hold a Dockerfile, which will contain instructions to create the specific Docker image we need. Open a terminal. `Ctrl + Alt + T`
```
mkdir ~/unitree_ros2_docker && cd ~/unitree_ros2_docker
```
Note: You can name it something other than unitree_ros2_docker if you'd like.

Now we will create a Dockerfile with our requirements:
```
nano Dockerfile
```

Now within the file, paste the following (`Ctrl + Shift + V`):
```                                                                
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt update && apt install -y \
    locales \
    lsb-release \
    curl \
    wget \
    git \
    nano \
    sudo \
    build-essential \
    cmake \
    libbullet-dev \
    libpython3-dev \
    python3-pip \
    python3-pyqt5 \
    libasio-dev \
    libtinyxml2-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    software-properties-common \
    python3-argcomplete \
    && locale-gen en_US en_US.UTF-8 && update-locale

# Python toolchain setup
RUN apt install -y python3-distutils python3-pip && \
    python3 -m pip install --no-cache-dir "setuptools<60" "importlib-metadata<5" numpy

# Remove broken system colcon and replace with pip version
RUN apt purge -y python3-colcon-common-extensions colcon-common-extensions || true && \
    python3 -m pip install --no-cache-dir colcon-common-extensions

# Fix expired ROS key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 Foxy source
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt upgrade -y

RUN apt install -y ros-foxy-desktop python3-argcomplete

RUN apt install -y ros-dev-tools
```

Build image:
```
sudo docker build -t unitree_ros2_env .
```
Note: You can name it something other than unitree_go2_env if you would like

Run image:
```
sudo docker run -it --name unitree_go2_env --net=host --privileged unitree_ros2_env bash
```

You can exit with `exit`

To re-enter later:
```
sudo docker start -ai unitree_go2_env
```

## Within the container
**Following along: https://github.com/unitreerobotics/unitree_ros2?tab=readme-ov-file**
```
apt update && apt upgrade
```

Make sure there aren't any ROS1 or ROS2 distros being sourced. Go into bashrc:
```
nano ~/.bashrc
```
And scroll to the bottom. If there is anything resembling `source /opt/ros/DISTRO/setup.bash` or anything mentioning ROS1 or ROS2, delete it.

```
git clone https://github.com/unitreerobotics/unitree_ros2
```
```
apt install ros-foxy-rmw-cyclonedds-cpp -y
```
^ Do the above and below commands separately, for whatever reason it aborts any time I try to do them at the same time.
```
apt install ros-foxy-rosidl-generator-dds-idl
```
Note: When in Docker container, don't use sudo since you're always a root user
```
cd /unitree_ros2/cyclonedds_ws/src
```
Make sure cd worked and you are in the right directory
```
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone --branch 0.8.0 https://github.com/eclipse-cyclonedds/cyclonedds.git
cd ..
colcon build --packages-select cyclonedds #Compile cyclone-dds package
```
```
source /opt/ros/foxy/setup.bash # source ROS2 environment
cd /unitree_ros2
```
Make sure you are in unitree_ros2 directory for colcon build
```
colcon build # Compile all packages in the workspace
```

Source:
```
source /opt/ros/foxy/setup.bash
source /unitree_ros2/cyclonedds_ws/install/setup.bash
```

## Network configuration
(first two steps taken directly from [here](https://github.com/unitreerobotics/unitree_ros2?tab=readme-ov-file#connect-to-unitree-robot))

Connect Unitree robot and the computer using Ethernet cable. Then, use ifconfig to view the network interface that the robot connected.

Next, open the network settings, find the network interface that the robot connected.
In IPv4 setting, change the IPv4 mode to manual, set the address to 192.168.123.99, and set the mask to 255.255.255.0. After completion, click apply and wait for the network to reconnect.

Edit setup.sh
```
nano /unitree_ros2/setup.sh
```
```             
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
source /unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
```

Edit cyclonedds.xml
```
nano /unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
```
```                   
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <NetworkInterfaceAddress>enx00e04c680aff</NetworkInterfaceAddress>
        </General>
    </Domain>
</CycloneDDS>
```

Source everything!!
```
source /opt/ros/foxy/setup.bash
source /unitree_ros2/cyclonedds_ws/install/setup.bash
source /unitree_ros2/install/setup.bash
source /unitree_ros2/setup.sh
```

To see ros2 topics:
```
ros2 topic list
```
To see velocities and other data:
```
ros2 topic echo /sportmodestate
```

## SSHing
**Following along: https://www.docs.quadruped.de/projects/go2/html/go2_driver.html#go2-tele-operation**
Since we're in a docker container and don't have any libraries installed, we need to get the one that lets us ping:
```
apt update && apt install -y iputils-ping
```
Now let's ping:
```
ping 192.168.123.18
```
A successful try will look like:
<pre>
unitree@ubuntu:~$ ping 192.168.123.18
PING 192.168.123.18 (192.168.123.18) 56(84) bytes of data.
64 bytes from 192.168.123.18: icmp_seq=1 ttl=64 time=0.027 ms
64 bytes from 192.168.123.18: icmp_seq=2 ttl=64 time=0.037 ms
64 bytes from 192.168.123.18: icmp_seq=3 ttl=64 time=0.023 ms
64 bytes from 192.168.123.18: icmp_seq=4 ttl=64 time=0.036 ms
^C
--- 192.168.123.18 ping statistics ---
4 packets transmitted, 4 received, 0% packet loss, time 3080ms
rtt min/avg/max/mdev = 0.023/0.030/0.037/0.006 ms
</pre>
SSH into robot, default password is `123`:
```
ssh -X unitree@192.168.123.18
```
Type `1` and enter if you've followed along with Foxy, which is for ROS2, Ubuntu 20.04. `2` is for Noetic which is for ROS1, Ubuntu 20.04.

Test out echoing the sportmodestate topic:
```
ros2 topic echo /sportmodestate
```
You may need to source first, then try again:
```
source unitree_ros2/setup.sh
```
or something similar.

To open another terminal we need to add the exec command, otherwise it will just mimic the other one you already have open.
Opening a new terminal while another one is already running in docker, SSHing or not, looks like:
```
sudo docker exec -it unitree_go2_env bash
```

## Sending stand command from computer
**Following along: https://support.unitree.com/home/en/developer/Quick_start**
Forewarning: when I run this example, I can't control it with the remote afterwards unless I power cycle the Go2.
Also, **make sure the robot is lying down before you send the command**.

Get ip link for connecting to your computer:
```
ip link show
```
The one for me showed eth0. So run this for stand up example:
```
cd ~/unitree_sdk2/build/bin
sudo ./stand_example_go2 eth0
```
If it says disable action mode, run:
```
./sport_client_example enx00e04c680aff
```
And then `Ctrl+C` out of it. Try `sudo ./stand_example_go2 eth0` again.
