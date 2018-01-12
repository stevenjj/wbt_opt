cmake_minimum_required(VERSION 2.8.3)
project(wbt_opt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
tf
val_viz
val_desc
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lcp_val_simulator
   CATKIN_DEPENDS 
       sensor_msgs
       tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
# After installing the sejong library via sudo make install, we have to find the installed libraries and all the included files

set(Sejong_Library_Path /usr/local/lib/SDC_Toolkit)
set(Sejong_Include_Path /usr/local/include/SDC_Toolkit/)
set(Sejong_Eigen_Path ~/Sejong_Dynamic_Control_Toolkit/ExternalSource)

find_library(SJUtils NAMES libSJutils.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJurdf NAMES libSJurdf.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJrbdl NAMES libSJrbdl.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJMobyLCP NAMES libSJMobyLCP.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJValkyrie_Controller NAMES libValkyrie_Controller.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJsnopt NAMES libSJsnopt.so PATHS ${Sejong_Library_Path}  REQUIRED)

find_library(Valkyrie_model NAMES libValkyrie_model.so PATHS ${Sejong_Library_Path} REQUIRED)
include_directories(${Sejong_Include_Path})
include_directories(${Sejong_Eigen_Path})

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

include_directories(include)
include_directories(src/valkyrie_dynamic_model)
set(val_sim_model_sources src/valkyrie_dynamic_model/valkyrie_definition.h
						  src/valkyrie_dynamic_model/Valkyrie_Dyn_Model.hpp						  
						  src/valkyrie_dynamic_model/Valkyrie_Dyn_Model.cpp
						  src/valkyrie_dynamic_model/Valkyrie_Kin_Model.hpp						  
						  src/valkyrie_dynamic_model/Valkyrie_Kin_Model.cpp						  
						  src/valkyrie_dynamic_model/RobotModel.hpp
						  src/valkyrie_dynamic_model/RobotModel.cpp)					


set(wbt_opt_sources     src/main.cpp
						src/wbt_optimization.cpp
						src/traj_solver.cpp
						src/wbt_task.cpp)	

add_executable(traj_gen_main ${wbt_opt_sources} ${val_sim_model_sources})
target_link_libraries(traj_gen_main ${catkin_LIBRARIES} ${SJUtils}
  								  					       ${SJurdf}
														   ${SJrbdl}
														   ${SJsnopt})


set(wbt_containers_sources  src/wbt_src/containers/wbt_opt_variable.cpp
							src/wbt_src/containers/wbt_wholebody_task_list.cpp)	 

set(wbt_task_sources src/wbt_src/tasks/wbt_task_com.cpp
					 src/wbt_src/tasks/wbt_task_leftfoot.cpp
					 src/wbt_src/tasks/wbt_task_rightfoot_righthand.cpp)

set(wbt_soft_constraint_sources src/wbt_src/soft_constraints/wbt_keyframe_position.cpp
								src/wbt_src/soft_constraints/wbt_keyframe_orientation.cpp)

add_executable(wbt_main src/wbt_src/main.cpp 
						${wbt_containers_sources}
						${wbt_task_sources}
						${wbt_soft_constraint_sources}
						${val_sim_model_sources})
target_link_libraries(wbt_main ${catkin_LIBRARIES} ${SJUtils}
  								  				   ${SJurdf}
												   ${SJrbdl}
												   ${SJsnopt})



SET (THIS_PACKAGE_PATH "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/model_config.h.cmake ${PROJECT_SOURCE_DIR}/include/model_config.h)