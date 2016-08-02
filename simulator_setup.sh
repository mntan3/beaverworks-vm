sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-indigo-controller-manager ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs ros-indigo-joint-state-controller ros-indigo-effort-controllers
