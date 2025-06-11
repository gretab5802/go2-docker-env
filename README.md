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
