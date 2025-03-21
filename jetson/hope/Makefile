WORKDIR := $(shell pwd)/catkin_ws

startNao:
	cd $(WORKDIR) && roslaunch naoqi_driver naoqi_driver.launch nao_ip:=192.168.1.101 network_interface:=wlan0

settings:
	ssh nao@192.168.1.101 'qicli call ALTabletService._openSettings'
wake:
	ssh nao@192.168.1.101 'qicli call ALMotion.wakeUp'
	ssh nao@192.168.1.101 'qicli call ALMotion.setExternalCollisionProtectionEnabled "All" 0'

moveHope:
	cd $(WORKDIR) && roslaunch move_base move_base.launch

loadMap:
	cd $(WORKDIR) && rosrun map_server map_server my_map.yaml

localize:
	cd $(WORKDIR) && rosservice call global_localization

initPose:
	rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: { frame_id: "map" }, pose: { pose: { position: { x: -1.7, y: -0.1, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } }, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] }}'

goal0:
	@rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: \"map\"}, pose: {position: {x: -1.15, y: -0.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

goal1:
	@rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: \"map\"}, pose: {position: {x: 1.080, y: -0.3, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}}}"

goal2:
	@rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: \"map\"}, pose: {position: {x: 5.926, y: -2.924, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.946, w: 0.323}}}"

goals:
	@rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: \"map\"}, pose: {position: {x: 5.926, y: -0.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

roscore:
	cd $(WORKDIR) && roscore
recompile:
	cd $(WORKDIR) && catkin_make
checkDependencies:
	cd $(WORKDIR) && rosdep check --from-paths src --ignore-src --rosdistro noetic
installDependencies:
	cd $(WORKDIR) && rosdep install --from-paths src --ignore-src -r -y
ssh:
	ssh nao@192.168.1.101 
pepperStart:
	ssh nao@192.168.1.101 'qicli call ALAutonomousLife.setState disabled'
	ssh nao@192.168.1.101 'qicli call ALMotion.wakeUp'
	
