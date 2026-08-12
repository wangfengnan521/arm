cmake_minimum_required(VERSION 3.8)
project(x5a_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/plan_joint_path.py scripts/fk_ik_test.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
