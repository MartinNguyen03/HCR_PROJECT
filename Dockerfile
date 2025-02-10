# Base image
FROM osrf/ros:kinetic-desktop-full
ENV ROS_DISTRO=kinetic
SHELL ["/bin/bash", "-c"]

# Set non-interactive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive


# Update and install dependencies in a single step
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim wget libprotobuf-dev protobuf-compiler \
    python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
    python-tk python-catkin-tools build-essential \
    && if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then rosdep init; fi \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages for NaoQi Driver and simulation
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-catkin ros-${ROS_DISTRO}-rostest ros-${ROS_DISTRO}-roslaunch \
    libboost-all-dev ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-kdl-parser ros-${ROS_DISTRO}-naoqi-bridge-msgs ros-${ROS_DISTRO}-naoqi-libqi \
    ros-${ROS_DISTRO}-naoqi-libqicore ros-${ROS_DISTRO}-orocos-kdl ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rosbag-storage ros-${ROS_DISTRO}-rosconsole ros-${ROS_DISTRO}-rosgraph-msgs \
    ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs ros-${ROS_DISTRO}-tf2-msgs \
    ros-${ROS_DISTRO}-tf2-ros  ros-${ROS_DISTRO}-rospy ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-roscpp \
    ros-${ROS_DISTRO}-std-srvs libgl1-mesa-glx libgl1-mesa-dri openjdk-8-jdk\
    && rm -rf /var/lib/apt/lists/*



# Install Pepper Gazebo simulation dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-gazebo-ros-control ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-ddynamic-reconfigure-python \
    ros-${ROS_DISTRO}-gmapping ros-${ROS_DISTRO}-map-server ros-${ROS_DISTRO}-amcl \
    ros-${ROS_DISTRO}-move-base \
    && rm -rf /var/lib/apt/lists/*

# Install additional Pepper Robot packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-naoqi-driver ros-${ROS_DISTRO}-naoqi-driver-py ros-${ROS_DISTRO}-naoqi-pose \
    ros-${ROS_DISTRO}-naoqi-sensors-py ros-${ROS_DISTRO}-pepper-sensors-py ros-${ROS_DISTRO}-pepper-description \
    ros-${ROS_DISTRO}-rgbd-launch ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-pepper-robot ros-${ROS_DISTRO}-pepper-control ros-${ROS_DISTRO}-naoqi-dcm-driver \
    && rm -rf /var/lib/apt/lists/*

# Install MoveIt for Pepper
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-planners-ompl ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-ros-planning ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-moveit-simple-controller-manager ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-moveit-fake-controller-manager \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# START PEPPER MESH PREP
# Add the actual ros package repo
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Add keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update
# END PEPPER MESH PREP
# install pepper meshes. We have to pipe this into 'yes' to agree to the license. otherwise docker build get's stuck on this step...
# we also have these debian environment params, otherwise the yes still gets stuck on the prompt in the mesh installation
# ?? pick one of these
ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype
RUN yes | apt-get install ros-${ROS_DISTRO}-pepper-meshes

# Source ROS environment on startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install Python 2 pip manually (last supported version)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-pip && rm -rf /var/lib/apt/lists/* \
    && python -m pip install --no-cache-dir "pip==20.3.4"


# Install Python packages
RUN python -m pip install --no-cache-dir \
    opencv-contrib-python-headless pyyaml scipy scikit-image matplotlib 


# Set entrypoint to a bash shell
ENTRYPOINT [""]
WORKDIR /catkin_ws
CMD /bin/bash
