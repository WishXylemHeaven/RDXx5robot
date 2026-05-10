cmake_minimum_required(VERSION 3.8)
project(digua_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/go_to_pose.py
  scripts/save_named_pose.py
  scripts/go_to_named_pose.py
  scripts/list_named_poses.py
  scripts/auto_initial_pose.py
  scripts/wait_for_tf.py
  scripts/follow_named_route.py
  scripts/start_nav_from_home.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()