SHELL := /bin/bash

.PHONY: all clean build gz reset_position

# Run the interactive simulation launch manager
all:
	./main-launch.sh

clean:
	rm -rf build/ install/ log/

build: 
	colcon build

gz:
	source install/setup.bash && ros2 launch bgr_description gazebo.launch.py world_name:=${WORLD:-Map1Opt.world} headless:=${HEADLESS:-false}

reset_position:
	source install/setup.bash && ros2 run bgr_description reset_car.py

