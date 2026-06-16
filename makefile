SHELL := /bin/bash

.PHONY: all clean build gz reset_position docker test

# Run the interactive simulation launch manager
all:
	./main-launch.sh

# Run the interactive simulation launch manager inside Docker
docker:
	docker compose up

clean:
	rm -rf build/ install/ log/

build: 
	colcon build

gz:
	ROS_DOMAIN_ID=0 source install/setup.bash && ros2 launch bgr_description gazebo.launch.py world_name:=${WORLD:-Map1Opt.world} headless:=${HEADLESS:-false}

# Reset car position (requires simulation docker containers to be running)
reset_position:
	source install/setup.bash && ros2 service call /reset_car std_srvs/srv/Trigger {}

test:
	./scripts/test.sh


