cmake_minimum_required(VERSION 3.0.2)
project(auvsl_dynamics)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  auvsl_control
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  CATKIN_DEPENDS roscpp rospy std_msgs auvsl_control message_runtime
)

###########
## Build ##
###########

include_directories(
  include/auvsl_dynamics
  include/auvsl_dynamics/generated
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)







add_executable(test_dataset_node
  src/test_dataset.cpp
  src/HybridDynamics.cpp
  src/utils.cpp
  src/TireNetwork.cpp
  src/generated/forward_dynamics.cpp
  src/generated/inertia_properties.cpp
  src/generated/inverse_dynamics.cpp
  src/generated/jacobians.cpp
  src/generated/jsim.cpp
  src/generated/miscellaneous.cpp
  src/generated/transforms.cpp
)

target_link_libraries(test_dataset_node ${catkin_LIBRARIES})
set_target_properties(test_dataset_node PROPERTIES COMPILE_FLAGS "-O3 -g -Wall -Wno-undef")






add_executable(test_dynamics_node
  src/test_dynamics_node.cpp
  src/HybridDynamics.cpp
  src/utils.cpp
  src/TireNetwork.cpp
  src/generated/forward_dynamics.cpp
  src/generated/inertia_properties.cpp
  src/generated/inverse_dynamics.cpp
  src/generated/jacobians.cpp
  src/generated/jsim.cpp
  src/generated/miscellaneous.cpp
  src/generated/transforms.cpp
)

target_link_libraries(test_dynamics_node ${catkin_LIBRARIES})
set_target_properties(test_dynamics_node PROPERTIES COMPILE_FLAGS "-O3 -g -Wall -Wno-undef")





add_executable(verify_hybrid_model_node
  src/verify_hybrid_model.cpp
  src/HybridDynamics.cpp
  src/utils.cpp
  src/TireNetwork.cpp
  src/VerifyModel.cpp
  src/generated/forward_dynamics.cpp
  src/generated/inertia_properties.cpp
  src/generated/inverse_dynamics.cpp
  src/generated/jacobians.cpp
  src/generated/jsim.cpp
  src/generated/miscellaneous.cpp
  src/generated/transforms.cpp
)

target_link_libraries(verify_hybrid_model_node ${catkin_LIBRARIES})
set_target_properties(verify_hybrid_model_node PROPERTIES COMPILE_FLAGS "-O3 -g -Wall -Wno-undef")




add_executable(search_bekker_params_node
  src/search_bekker_params.cpp
  src/HybridDynamics.cpp
  src/utils.cpp
  src/TireNetwork.cpp
  src/VerifyModel.cpp
  src/generated/forward_dynamics.cpp
  src/generated/inertia_properties.cpp
  src/generated/inverse_dynamics.cpp
  src/generated/jacobians.cpp
  src/generated/jsim.cpp
  src/generated/miscellaneous.cpp
  src/generated/transforms.cpp
)

target_link_libraries(search_bekker_params_node ${catkin_LIBRARIES})
set_target_properties(search_bekker_params_node PROPERTIES COMPILE_FLAGS "-O3 -g -Wall -Wno-undef")




add_executable(train_hybrid_model_node
  src/train_hybrid_model.cpp
  src/HybridDynamics.cpp
  src/utils.cpp
  src/TireNetwork.cpp
  src/VerifyModel.cpp
  src/generated/forward_dynamics.cpp
  src/generated/inertia_properties.cpp
  src/generated/inverse_dynamics.cpp
  src/generated/jacobians.cpp
  src/generated/jsim.cpp
  src/generated/miscellaneous.cpp
  src/generated/transforms.cpp
)

target_link_libraries(train_hybrid_model_node ${catkin_LIBRARIES})
set_target_properties(train_hybrid_model_node PROPERTIES COMPILE_FLAGS "-O3 -DNDEBUG -march=native -Wall -Wno-undef")
#set_target_properties(train_hybrid_model_node PROPERTIES COMPILE_FLAGS "-g -Wall -Wno-undef")
