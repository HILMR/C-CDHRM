cmake_minimum_required(VERSION 3.0.2)
project(solver_ftl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES snake_planning
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
 DEPENDS system_lib
)

add_library(KM_CCDHRM SHARED IMPORTED)
set_target_properties(KM_CCDHRM PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libKM_CCDHRM.so)
add_library(FTLSolver_SCP SHARED IMPORTED)
set_target_properties(FTLSolver_SCP PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libFTLSolver_SCP.so)

add_executable(ccdhrm_ftl_test test/CCDHRM_FTL_test.cpp
src/FTL_CCDHRM.cpp
src/FTLSolver.cpp
)
target_link_libraries(ccdhrm_ftl_test ${catkin_LIBRARIES} KM_CCDHRM FTLSolver_SCP)