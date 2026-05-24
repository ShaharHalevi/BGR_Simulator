SHELL := /bin/bash

.PHONY: all clean build gz ctrl reset_position

# this will build the workspace, wait until the build is finished, then 
# open two gnome terminals, one for gazebo and one for the controller, they will run in parallel because of the '&'
all: 
	make build
	gnome-terminal -- bash -c "make gz; exec bash" & 
	gnome-terminal -- bash -c "make ctrl; exec bash"

clean:
	rm -rf build/

build: 
	colcon build

gz:
	source install/setup.bash && ros2 launch bgr_description gazebo.launch.py world_name:=CompetitionMap1.world headless:=${HEADLESS:-false}

ctrl:
	source install/setup.bash && ros2 launch bgr_controller controller.launch.py

reset_position:
	gz service -s /world/empty/set_pose \
		--reqtype gz.msgs.Pose \
		--reptype gz.msgs.Boolean \
		--timeout 2000 \
		--req "name: 'bgr', position: {x: 45.751, y: 81.3, z: 1.0}, orientation: {x: -0.0001, y: -0.0007, z: 1.232886347679596e-06, w: 0.9999}"

