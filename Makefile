# Variables
IMAGE_NAME := martinnguyen03/hope
CONTAINER_NAME := ros_noetic_container
ROS_IP := 127.0.0.1

# Build the Docker image
build:
	docker build -t $(IMAGE_NAME) .
	docker tag martinnguyen03/hope martinnguyen03/hope:latest
	docker push martinnguyen03/hope:latest

build_baseline:
	docker pull martinnguyen03/hope:latest
	@${MAKE} .compile


.compile:
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
	docker container stop hopeContainer || true && docker container rm hopeContainer || true
	docker run \
		-it \
		-e ROS_IP="${ROS_IP}" \
		-e ROS_MASTER_URI="http://${ROS_IP}:11311" \
		-e DISPLAY \
		-v /dev:/dev \
		-v ${PWD}/catkin_ws:/catkin_ws:rw \
		--detach \
		--privileged \
		--runtime nvidia \
		--network host \
  		--gpus all \
		--name hopeContainer \
		martinnguyen03/hope:latest
	docker exec hopeContainer bash -c "source /opt/ros/kinetic/setup.bash && catkin build"
	docker exec hopeContainer bash -c "source devel/setup.bash && cd /catkin_ws/devel && make pepper_meshes_meshes"
	docker container stop hopeContainer 

demo:
	xhost +si:localuser:root >> /dev/null
	docker start hopeContainer
	sleep 1
	docker exec -it hopeContainer bash -c "source devel/setup.bash && roslaunch pepper_moveit_config demo.launch"
	docker container stop hopeContainer 
	
# Run the container interactively
run:
	docker run -it --rm --name $(CONTAINER_NAME) \
		--network host \
		--gpus all \
		--privileged \
		-v $(PWD)/catkin_ws:/catkin_ws:rw \
		$(IMAGE_NAME) /bin/bash

# Start the container in detached mode
start:
	docker run -dit --name $(CONTAINER_NAME) \
		--network host \
		--gpus all \
		--privileged \
		-v $(PWD)/catkin_ws:/catkin_ws:rw \
		$(IMAGE_NAME)

# Attach to the running container
attach:
	docker exec -it $(CONTAINER_NAME) /bin/bash

# Install ROS naoqi_driver inside the running container
install-naoqi:
	docker exec -it $(CONTAINER_NAME) bash -c "\
	    source /opt/ros/noetic/setup.bash && \
	    mkdir -p /catkin_ws/src && cd /catkin_ws/src && \
	    git clone --branch noetic-devel https://github.com/ros-naoqi/naoqi_driver.git && \
	    git clone --branch noetic-devel https://github.com/ros-naoqi/naoqi_libqi.git && \
	    git clone --branch noetic-devel https://github.com/ros-naoqi/naoqi_bridge_msgs.git && \
	    git clone --branch noetic-devel https://github.com/ros-naoqi/naoqi_tools.git && \
	    cd /catkin_ws && catkin_make && \
	    echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc"

# clone the required repos into a catkin workspace
# RUN git clone -b correct_chain_model_and_gazebo_enabled https://github.com/awesomebytes/pepper_robot /catkin_ws/src/pepper_robot
# RUN git clone -b simulation_that_works https://github.com/awesomebytes/pepper_virtual /catkin_ws/src/pepper_virtual
# RUN git clone https://github.com/awesomebytes/gazebo_model_velocity_plugin /catkin_ws/src/gazebo_model_velociy_plugin


# # we add these two commands to the bashrc in the container, so that the entrypoint and workspacea will be sourced,
# # whenever a new bash session is instantiated in the container
# RUN echo 'source /ros_entrypoint.sh' >>  /root/.bashrc
# RUN echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc

# Stop the container
stop:
	docker stop $(CONTAINER_NAME)

# Remove the container
clean:
	docker rm -f $(CONTAINER_NAME)

# Remove the Docker image
clean-image:
	docker rmi -f $(IMAGE_NAME)

recompile:
	docker start hopeContainer
	docker exec -it hopeContainer bash -c "source /opt/ros/kinetic/setup.bash && catkin build"
	docker stop hopeContainer

# Authenticate with Docker Hub
login:
	docker login

# Push the Docker image to Docker Hub
push:
	docker push $(IMAGE_NAME)
