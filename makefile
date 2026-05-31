SHELL := /bin/bash

.PHONY: all clean build gz reset_position docker test

# Run the interactive simulation launch manager
all:
	./main-launch.sh

# Run the interactive simulation launch manager inside Docker
docker:
	docker compose run simulator

clean:
	rm -rf build/ install/ log/

build: 
	colcon build

gz:
	source install/setup.bash && ros2 launch bgr_description gazebo.launch.py world_name:=${WORLD:-Map1Opt.world} headless:=${HEADLESS:-false}

reset_position:
	source install/setup.bash && ros2 run bgr_description reset_car.py

test:
	colcon build --packages-select bgr_description
	-source install/setup.bash && colcon test --packages-select bgr_description --event-handlers console_direct+
	colcon test-result --all --verbose


