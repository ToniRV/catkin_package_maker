cmake_minimum_required(VERSION 2.8.3)
project(@project_name@)

find_package(catkin_simple REQUIRED)

catkin_python_setup()

catkin_simple()

cs_add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp)

cs_install()

cs_install_scripts(scripts/@project_name@_script)

cs_export()
