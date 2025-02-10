# Variables
IMAGE_NAME := martinnguyen03/hope
CONTAINER_NAME := ros_noetic_container
ROS_IP := 127.0.0.1
NAO_IP := 192.168.1.101
NAO_USERNAME := nao
NAO_PASSWORD := BioARTLab123
# Build the Docker image
build:
	docker build -t $(IMAGE_NAME) .
	docker tag martinnguyen03/hope martinnguyen03/hope:latest
	docker push martinnguyen03/hope:latest

build_baseline:
	docker pull martinnguyen03/hope:latest
	@${MAKE} .compile

build_baseline_gpu:
	docker pull martinnguyen03/hope:latest
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
		--name hopeContainer \
		martinnguyen03/hope:latest
	docker exec hopeContainer bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"
	docker container stop hopeContainer 


.compile_nvidia:
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
		-e NVIDIA_DRIVER_CAPABILITIES=all \
		-e NVIDIA_VISIBLE_DEVICES=all \
    	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /dev:/dev \
		-v ${PWD}/catkin_ws:/catkin_ws:rw \
		--device=/dev/ttyUSB0 \
		--detach \
		--privileged \
		--runtime nvidia \
		--network host \
  		--gpus all \
		--name hopeContainer \
		martinnguyen03/hope:latest
	docker exec hopeContainer bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"
	docker container stop hopeContainer

rplLaunch:
	xhost +si:localuser:root >> /dev/null
	docker start hopeContainer
	sleep 1
	docker exec -it hopeContainer bash -c "source devel/setup.bash && roslaunch rplidar_ros rplidar.launch"
	docker stop hopeContainer

slamLaunch:
	xhost +si:localuser:root >> /dev/null
	docker start hopeContainer
	sleep 1
	docker exec -it hopeContainer bash -c "source devel/setup.bash && roslaunch hector_slam_launch tutorial.launch"
	docker stop hopeContainer

	
rplScan:
	xhost +si:localuser:root >> /dev/null
	docker start hopeContainer
	sleep 1
	docker exec -it hopeContainer bash -c "sudo chmod 666 /dev/ttyUSB0 && source devel/setup.bash && roslaunch rplidar_ros view_rplidar.launch"
	docker stop hopeContainer

demo:
	xhost +si:localuser:root >> /dev/null
	docker start hopeContainer
	sleep 1
	docker exec -it hopeContainer bash -c "source devel/setup.bash && roslaunch pepper_moveit_config demo.launch"
	docker container stop hopeContainer 
	
real_robot:
	xhost +si:localuser:root >> /dev/null
	docker start hopeContainer
	export NAO_IP=192.168.1.101
	docker exec -it hopeContainer bash -c "source devel/setup.bash && roslaunch pepper_dcm_bringup pepper_bringup.launch" && \
	docker exec -it hopeContainer bash -c "source devel/setup.bash && roslaunch pepper_moveit_config moveit_planner.launch"
	docker stop hopeContainer


connect:
	xhost +si:localuser:root >> /dev/null
	docker start hopeContainer
	docker exec -it hopeContainer bash -c "source devel/setup.bash && roslaunch naoqi_driver naoqi_driver.launch nao_ip:=${NAO_IP}  network_interface:=wlp9s0 username:=${NAO_USERNAME} password:=${NAO_PASSWORD}"
	

recompile:
	docker start hopeContainer
	docker exec -it hopeContainer bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"
	docker stop hopeContainer

install_dependencies:
	docker start hopeContainer
	docker exec -it hopeContainer bash -c "source /opt/ros/kinetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"
	docker stop hopeContainer
# Authenticate with Docker Hub
login:
	docker login

# Push the Docker image to Docker Hub
push:
	docker push $(IMAGE_NAME)
