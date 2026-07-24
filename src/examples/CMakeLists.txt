cmake_minimum_required(VERSION 3.8)
project(examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------- 基础依赖 ----------------
find_package(ament_cmake REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ruckig REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(CURL REQUIRED)

# ---------------- 键盘功能开关 & ncurses 探测 ----------------
option(ENABLE_KEYBOARD "Build executables that depend on curses" ON)
if(DEFINED ENV{ENABLE_KEYBOARD})
  set(ENABLE_KEYBOARD
      "$ENV{ENABLE_KEYBOARD}"
      CACHE BOOL "" FORCE)
endif()

set(HAVE_CURSES FALSE)
if(ENABLE_KEYBOARD)
  # 1) 先尝试 CMake 自带的 FindCurses
  find_package(Curses QUIET) # 提供: CURSES_FOUND / CURSES_INCLUDE_DIR / CURSES_LIBRARIES
  if(CURSES_FOUND)
    set(HAVE_CURSES TRUE)
  else()
    # 2) 用 pkg-config 兜底，同时兼容 ncursesw 与 ncurses
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
      pkg_check_modules(NCURSES QUIET ncursesw)
      if(NOT NCURSES_FOUND)
        pkg_check_modules(NCURSES QUIET ncurses)
      endif()
      if(NCURSES_FOUND)
        set(HAVE_CURSES TRUE)
        set(CURSES_INCLUDE_DIR ${NCURSES_INCLUDE_DIRS})
        set(CURSES_LIBRARIES ${NCURSES_LINK_LIBRARIES})
      endif()
    endif()
  endif()
endif()

# ---------------- HAL 可执行程序 ----------------
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/src/hal/*.cpp)
foreach(file ${HAL_SOURCES})
  message(STATUS "add executable for hal: ${file}")
  get_filename_component(filename ${file} NAME_WE)
  add_executable(${filename} ${file})
  target_link_libraries(${filename} ruckig::ruckig)
  ament_target_dependencies(${filename} rclcpp aimdk_msgs sensor_msgs cv_bridge)

  # 若该节点用到了 OpenCV（示例：take_photo, echo_camera_rgbd, echo_camera_stereo），显式链接
  if("${filename}" STREQUAL "take_photo")
    message(STATUS "link OpenCV & libcurl to ${filename}")
    target_include_directories(${filename} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${filename} ${OpenCV_LIBS} CURL::libcurl)
  elseif(
    "${filename}" STREQUAL "echo_camera_rgbd"
    OR "${filename}" STREQUAL "echo_camera_stereo"
    OR "${filename}" STREQUAL "echo_camera_head_rear")
    message(STATUS "link OpenCV to ${filename}")
    target_include_directories(${filename} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${filename} ${OpenCV_LIBS})
  endif()

  install(TARGETS ${filename} DESTINATION lib/${PROJECT_NAME})
endforeach()

# ---------------- MC 可执行程序 ----------------
file(GLOB MC_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/src/mc/*.cpp)

# 列出需要 ncurses 的节点名（与源文件名一致，不含扩展名）
set(CURSES_TARGETS keyboard)

foreach(file ${MC_SOURCES})
  get_filename_component(filename ${file} NAME_WE)

  # 没有 ncurses 时，跳过这些目标 —— 避免包含 <curses.h> 导致编译报错
  if("${filename}" IN_LIST CURSES_TARGETS AND (NOT ENABLE_KEYBOARD OR NOT HAVE_CURSES))
    message(WARNING "Skipping ${filename}: curses backend not available")
    continue()
  endif()

  message(STATUS "add executable for mc: ${file}")
  add_executable(${filename} ${file})
  ament_target_dependencies(${filename} rclcpp aimdk_msgs)

  # 仅对依赖 ncurses 的目标添加包含与链接
  if("${filename}" IN_LIST CURSES_TARGETS)
    if(CURSES_INCLUDE_DIR)
      target_include_directories(${filename} PRIVATE ${CURSES_INCLUDE_DIR})
    endif()
    if(CURSES_LIBRARIES)
      target_link_libraries(${filename} ${CURSES_LIBRARIES})
    endif()
    # 个别发行版需要额外链接 tinfo
    if(DEFINED CURSES_TINFO_LIBRARY)
      target_link_libraries(${filename} ${CURSES_TINFO_LIBRARY})
    endif()
    target_compile_definitions(${filename} PRIVATE NCURSES_WIDECHAR=1)
  endif()

  install(TARGETS ${filename} DESTINATION lib/${PROJECT_NAME})
endforeach()

#  --------------- interaction 可执行程序 ----------------
file(GLOB INTERACTION_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/src/interaction/*.cpp)
foreach(file ${INTERACTION_SOURCES})
  message(STATUS "add executable for interaction: ${file}")
  get_filename_component(filename ${file} NAME_WE)
  add_executable(${filename} ${file})
  ament_target_dependencies(${filename} rclcpp aimdk_msgs)
  install(TARGETS ${filename} DESTINATION lib/${PROJECT_NAME})
endforeach()

# ---------------- robot 模块 ----------------
#add_executable(robot src/robot/main.cpp src/robot/robot.cpp src/robot/robot.h)
#ament_target_dependencies(robot rclcpp aimdk_msgs)
#install(TARGETS robot DESTINATION lib/${PROJECT_NAME})

# ---------------- 测试 ----------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
