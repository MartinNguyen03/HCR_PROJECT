# Variables
IMAGE_NAME := martinnguyen03/hope
ROS_DISTRO := noetic
ROS_IP := 127.0.0.1
NAO_IP := 192.168.1.101
NAO_USERNAME := nao
NAO_PASSWORD := BioARTLab123
CONTAINER := hopeContainer
dockerPush:
	docker commit ${CONTAINER} ${IMAGE_NAME}:latest
	docker tag ${IMAGE_NAME}:latest ${IMAGE_NAME}:latest
	docker push ${IMAGE_NAME}:latest

# Build the Docker image
build:
	docker build -t $(IMAGE_NAME) .
	docker tag martinnguyen03/kope martinnguyen03/kope:latest
	docker push martinnguyen03/kope:latest

build_baseline:
	docker pull ${IMAGE_NAME}:latest
	@${MAKE} .compile

build_baseline_gpu:
	docker pull ${IMAGE_NAME}:latest
	@${MAKE} .compile_nvidia

.compile:
	if [-d ${PWD}/catkin_ws]; then rm -rf ${PWD}/catkin_ws; fi
	mkdir -p ${PWD}/catkin_ws/src
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/naoqi_driver.git
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_meshes
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_robot.git tmp_pepper_robot && \
	mv ${PWD}/catkin_ws/src/tmp_pepper_robot/* ${PWD}/catkin_ws/src/ && \
	rm -rf ${PWD}/catkin_ws/src/tmp_pepper_robot
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_moveit_config.git
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_virtual.git tmp_pepper_virtual && \
	mv ${PWD}/catkin_ws/src/tmp_pepper_virtual/* ${PWD}/catkin_ws/src/ && \
	rm -rf ${PWD}/catkin_ws/src/tmp_pepper_virtual
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_dcm_robot.git
	git -C ${PWD}/catkin_ws/src clone https://github.com/robopeak/rplidar_ros.git 
	git -C ${PWD}/catkin_ws/src clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
	docker run \
		-it \
		-e ROS_IP="${ROS_IP}" \
		-e ROS_MASTER_URI="http://${ROS_IP}:11311" \
		-e DISPLAY \
		-v /dev:/dev \
		-v ${PWD}/catkin_ws:/catkin_ws:rw \
		--detach \
		--privileged \
		--network host \
		--name ${CONTAINER} \
		${IMAGE_NAME}:latest
	docker exec ${CONTAINER} bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make"
	docker container stop ${CONTAINER} 


	.compile_nvidia:
		docker run \
			-it \
			-e ROS_IP="${ROS_IP}" \
			-e ROS_MASTER_URI="http://${ROS_IP}:11311" \
			-e DISPLAY \
			-e QT_X11_NO_MITSHM=1 \
			-e NVIDIA_DRIVER_CAPABILITIES=all \
			-e NVIDIA_VISIBLE_DEVICES=all \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v /dev:/dev \
			-v ${PWD}/catkin_ws:/catkin_ws:rw \
			-v /run/udev:/run/udev:ro \
			--device=/dev/ttyUSB0 \
			--detach \
			--privileged \
			--runtime nvidia \
			--network host \
			--gpus all \
			--name ${CONTAINER} \
			${IMAGE_NAME}:latest
		docker exec ${CONTAINER} bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make"
		docker container stop ${CONTAINER}



rplLaunch:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	sleep 1
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch rplidar_ros rplidar.launch"
	docker stop ${CONTAINER}

exampleDepthLaunch:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch depthai_examples rgb_stereo_node.launch camera_model:=OAK-D"

depthLaunch:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch depthai_ros_driver camera.launch"

slamLaunch:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	sleep 1
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch hector_slam_launch tutorial.launch"
	docker stop ${CONTAINER}

amclLaunch:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch amcl amcl_hope.launch scan_topic:=scan"
	docker stop ${CONTAINER}

rplScan:
	xhost +local:docker
	docker start ${CONTAINER}
	sleep 1
	docker exec -it ${CONTAINER} bash -c "sudo chmod 666 /dev/ttyUSB0 && source devel/setup.bash && roslaunch rplidar_ros view_rplidar.launch"
	docker stop ${CONTAINER}

saveMap:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	sleep 1
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && rosrun map_server map_saver -f my_map"
	docker stop ${CONTAINER}

movehope:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch move_base move_base.launch"
	docker stop ${CONTAINER}

saveGeotiff:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && rostopic pub syscommand std_msgs/String "savegeotiff""
	docker stop ${CONTAINER}

rviz:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && rviz"
	docker stop ${CONTAINER}
roscore:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roscore"
	docker stop ${CONTAINER}

loadMap:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && rosrun map_server map_server my_map.yaml"
	docker stop ${CONTAINER}

loadHome:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && rosrun map_server map_server my_room.yaml"
	docker stop ${CONTAINER}
	
demo:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	sleep 1
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch pepper_moveit_config demo.launch"
	docker stop ${CONTAINER} 
	
real_robot:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	export NAO_IP=${NAO_IP} 
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch pepper_dcm_bringup pepper_bringup.launch" && \
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch pepper_moveit_config moveit_planner.launch"
	docker stop ${CONTAINER}

globalLocalise:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && rosservice call global_localization"
	
start:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "ssh nao@192.168.1.101 'qicli call ALAutonomousLife.setState disabled'"
	sleep 2
	docker exec -it ${CONTAINER} bash -c "ssh nao@192.168.1.101 'qicli call ALMotion.wakeUp'"
	docker stop ${CONTAINER}

wake:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "ssh nao@192.168.1.101 'qicli call ALMotion.wakeUp'"
	docker stop ${CONTAINER}
pepperSettings:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec it ${CONTAINER} bash -c "qicli call ALTabletService._openSettings"
	docker stop ${CONTAINER}
	
ssh:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "ssh nao@192.168.1.101"

connectNao:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	sleep 2
	docker exec -it ${CONTAINER} /bin/bash -c "\
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        source devel/setup.bash && \
        roslaunch naoqi_driver naoqi_driver.launch nao_ip:=192.168.1.101 network_interface:=wlp9s0\
    "
	
remote:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	ssh jetson@192.168.1.102
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && export ROS_MASTER_URI=http://jetson:$1"

naoStatus:
	xhost +local:docker
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && rosnode info /naoqi_driver"
	docker stop ${CONTAINER}

recompile:
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make"
	docker stop ${CONTAINER}
bringup:
	docker start kopeContainer
	docker exec -it kopeContainer bash -c "source devel/setup.bash && roslaunch pepper_bringup pepper_full.launch nao_ip:=192.168.1.101 network_interface:=wlp9s0"
	

dcmBringup:
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:=192.168.1.101 network_interface:=wlp9s0"
	docker stop ${CONTAINER}

dcmControl:
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source devel/setup.bash && roslaunch pepper_dcm_bringup pepper_dcm_bringup_position.launch robot_ip:=192.168.1.101 network_interface:=wlp9s0"
	docker stop ${CONTAINER}

install_dependencies:
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src -r -y"
	docker stop ${CONTAINER}

check_dependencies:
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep check --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}"
	docker stop ${CONTAINER}

terminal:
	xhost +si:localuser:root >> /dev/null
	docker start ${CONTAINER}
	docker exec -it ${CONTAINER} bash

.install_packages:
	if [-d ${PWD}/catkin_ws]; then rm -rf ${PWD}/catkin_ws; fi
	mkdir -p ${PWD}/catkin_ws/src
	git -C ${PWD}/catkin_ws/src clone --branch ${ROS_DISTRO}-devel --single-branch https://github.com/ros-planning/navigation.git
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/naoqi_driver.git
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_meshes
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_robot.git tmp_pepper_robot && \
	mv ${PWD}/catkin_ws/src/tmp_pepper_robot/* ${PWD}/catkin_ws/src/ && \
	rm -rf ${PWD}/catkin_ws/src/tmp_pepper_robot
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_moveit_config.git
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_virtual.git tmp_pepper_virtual && \
	mv ${PWD}/catkin_ws/src/tmp_pepper_virtual/* ${PWD}/catkin_ws/src/ && \
	rm -rf ${PWD}/catkin_ws/src/tmp_pepper_virtual
	git -C ${PWD}/catkin_ws/src clone https://github.com/ros-naoqi/pepper_dcm_robot.git
	git -C ${PWD}/catkin_ws/src clone https://github.com/robopeak/rplidar_ros.git 
	git -C ${PWD}/catkin_ws/src clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
	git -C ${PWD}/catkin_ws/src clone --branch ${ROS_DISTRO} https://github.com/luxonis/depthai-ros.git