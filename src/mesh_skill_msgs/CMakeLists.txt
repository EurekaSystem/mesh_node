cmake_minimum_required(VERSION 3.8)
project(mesh_skill_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Mesh.action"
  DEPENDENCIES action_msgs std_skills
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)