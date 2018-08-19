###### Get script location
# https://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"

###### Setup ros path
cd ~
mkdir -p ~/ros/src
cd ~/ros
catkin_make
echo "source ~/ros/devel/setup.bash" >> ~/.bashrc
source ~/ros/devel/setup.bash

###### Install nimbro networks
cd ~/ros/src
git clone https://github.com/AIS-Bonn/nimbro_network.git
mv nimbro_network/*/ .
rm -rf nimbro_network

###### Link our packages
ln -s $DIR/arm_controls
ln -s $DIR/camera_controls
ln -s $DIR/drive_controls
ln -s $DIR/sensors_controls
ln -s $DIR/station2

###### Build everything
cd ~/ros
catkin_make

###### Finish
printf "Thanks for unpacking!\nNow that your enviroment is setup, you should never have to do this again."
