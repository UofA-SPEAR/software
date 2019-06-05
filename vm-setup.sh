# Install script for SPEAR virtual machine.

# Makes sure Ubuntu is up to date.
sudo apt-get update
sudo apt-get -y upgrade

# Installs ROS Kinetic.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full

# Sets up ROS.
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install -y python-rosinstall \
                        python-rosinstall-generator \
                        python-wstool \
                        build-essential

# Installs software repository dependencies.
sudo apt-get install -y qt4-default \
                        libx264-dev \
                        ros-kinetic-rqt \
                        ros-kinetic-gscam \
                        ros-kinetic-catch-ros \
                        ros-kinetic-tf \
                        ros-kinetic-opencv3 \
                        ros-kinetic-cv-bridge \
						ros-kinetic-ros-control \
						ros-kinetic-ros-controllers \
                        gstreamer-0.1 \
                        libgstreamer0.10-dev \
                        libgstreamer-plugins-base0.10-dev \
                        gstreamer0.10-plugins-good \
                        python-pip

echo "Done installing dependencies! Please restart your terminal for changes to take effect."
