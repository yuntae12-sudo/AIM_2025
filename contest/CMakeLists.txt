cmake_minimum_required(VERSION 3.0.2)
# 패키지 이름 설정 (원하는 이름으로 변경 가능)
project(contest)

# PCL 라이브러리 사용을 위해 C++14 표준 적용
add_compile_options(-std=c++14)

# --------------------------------------------------------
# 1. 의존성 패키지 찾기 (Find Packages)
# --------------------------------------------------------
# 두 파일의 의존성 패키지를 모두 합쳤습니다.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  roslib
  pcl_ros
  pcl_conversions
  lidar_code
  # morai_msgs가 커스텀 메시지 패키지라면 워크스페이스에 있어야 합니다.
  morai_msgs
  rospy
  message_generation 
)

# 외부 라이브러리 (PCL, Eigen, OpenMP)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)


## 2. 커스텀 메시지 빌드 설정
add_message_files(
  FILES
  LaneInfo.msg
)

## 3. 메시지 생성 실행
generate_messages(
  DEPENDENCIES
  std_msgs
)

# --------------------------------------------------------
# 2. Catkin 패키지 설정
# --------------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp std_msgs geometry_msgs sensor_msgs nav_msgs 
    visualization_msgs roslib pcl_ros pcl_conversions morai_msgs
    rospy std_msgs sensor_msgs message_runtime
)

# --------------------------------------------------------
# 3. Include 경로 설정
# --------------------------------------------------------
include_directories(
  include
  src/localization
  src/lshape
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar/hungarian
)

# ========================================================
# 4. 빌드 타겟 1: Control Node (기존 main)
# ========================================================
add_definitions(${PCL_DEFINITIONS})
add_executable(main
  src/main.cpp
  src/planning/localization/localization.cpp
  src/planning/objective_function/objective_function.cpp
  src/planning/path/coner.cpp
  src/planning/path/followPoint.cpp
  src/control/steering/steering.cpp
  src/control/velocity/velocity.cpp
  src/visualization/visualization.cpp
  src/planning/gps_jamming/jamming.cpp
)
add_dependencies(main
  lidar_code_generate_messages_cpp
  morai_msgs_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(main
  ${catkin_LIBRARIES}
)

add_executable(path_maker src/path_maker.cpp)

## 라이브러리 링크 (ROS 라이브러리 연결)
target_link_libraries(path_maker
  ${catkin_LIBRARIES}
)

## 의존성 설정 (메시지 생성 순서 보장)
add_dependencies(path_maker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  morai_msgs_generate_messages_cpp
)