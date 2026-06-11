cmake_minimum_required(VERSION 3.8)
project(bgr_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  config/bgr_controllers.yaml
  config/joy_config.yaml
  config/joy_teleop.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  bgr_controller/joy_array_bridge.py
  bgr_controller/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
