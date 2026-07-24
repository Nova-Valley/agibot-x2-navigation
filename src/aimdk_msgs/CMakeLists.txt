cmake_minimum_required(VERSION 3.8)
project(aimdk_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 如果存在 src/interface 目录, 首先执行脚本分发协议文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/interface)
  execute_process(
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tools/distribute_proto.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE SCRIPT_RESULT)
endif()

# 扫描 srv 目录下所有的 .srv 文件
file(GLOB_RECURSE SRV_FILES_ABS "interface/*.srv")
get_filename_component(SRV_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/interface" ABSOLUTE)
set(SRV_FILES_REL "")
foreach(ABS_PATH ${SRV_FILES_ABS})
  file(RELATIVE_PATH REL_PATH "${SRV_BASE_DIR}" "${ABS_PATH}")
  list(APPEND SRV_FILES_REL "interface/${REL_PATH}")
endforeach()

# 扫描 msg 目录下所有的 .msg 文件
file(GLOB_RECURSE MSG_FILES_ABS "interface/*.msg")
get_filename_component(MSG_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/interface" ABSOLUTE)
set(MSG_FILES_REL "")
foreach(ABS_PATH ${MSG_FILES_ABS})
  file(RELATIVE_PATH REL_PATH "${MSG_BASE_DIR}" "${ABS_PATH}")
  list(APPEND MSG_FILES_REL "interface/${REL_PATH}")
endforeach()

message(STATUS "SRV files: ${SRV_FILES_REL}")
message(STATUS "MSG files: ${MSG_FILES_REL}")

# 生成接口文件
rosidl_generate_interfaces(${PROJECT_NAME} ${SRV_FILES_REL} ${MSG_FILES_REL} DEPENDENCIES geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
