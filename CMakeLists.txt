cmake_minimum_required(VERSION 3.5.1)
project(CarClient)
find_package(Protobuf REQUIRED)
set(CMAKE_BUILD_TYPE DEBUG)

# set(CMAKE_C_COMPILER /usr/bin/gcc-7)
# set(CMAKE_CXX_COMPILE /usr/bin/g++-7)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -fPIC -O3 -DNDEBUG  -Wall -Wextra" CACHE STRING "" FORCE)
#find_package(Boost REQUIRED )

#include_directories(${Boost_INCLUDE_DIRS}) 

include_directories(/home/codewalker/carla/Build/rpclib-v2.2.1_c5-c8-libcxx-install/include
/home/codewalker/new_carla/CppAPI/cppclient/libcarla-install/include/system
/home/codewalker/carla/Build/recast-0b13b0-c8-install/include
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/include/Planning
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/include/Common/math
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/include/proto
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/modules/Planning/tasks/dp_path/include                              
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/modules/Planning/tasks/dp_speed/include
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/modules/Planning/tasks/qp_path/include
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/modules/Planning/tasks/qp_speed/include
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/modules/Planning/tasks/planning_start_point/include
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/3rd_party/osqp-eigen/include
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/modules/Planning/planner/em_planner/include
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/3rd_party/osqp/include
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/3rd_party/eigen
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/3rd_party/eigen
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/modules/Common
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/include/Controlling
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/modules/Planning/pnc_map/include  
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/include/Planning
/home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/include/proto/common

)

include_directories(libcarla-install/include )

set(CARLA_LIB_DIR /home/codewalker/new_carla/CppAPI/cppclient/libcarla-install/lib)

link_directories( ${CARLA_LIB_DIR}) 

message(" Boost_LIBRARY_DIRS is : " ${Boost_LIBRARY_DIRS})


add_executable(test town3_drive.cpp)

target_include_directories(test
    PUBLIC
    ${PROTOBUF_INCLUDE_DIRS}
)



target_link_libraries(test
${CARLA_LIB_DIR}/libcarla_client.a
 ${CARLA_LIB_DIR}/librpc.a 
 ${CARLA_LIB_DIR}/libpng.so  
 /home/codewalker/carla/Build/boost-1.80.0-c8-install/lib/libboost_filesystem.so.1.80.0
  ${CARLA_LIB_DIR}/libboost_atomic.so.1.80.0
  ${CARLA_LIB_DIR}/libDetourCrowd.a  
  ${CARLA_LIB_DIR}/libDetour.a 
  ${CARLA_LIB_DIR}/libRecast.a 
  ${CARLA_LIB_DIR}/libboost_system.a
  ${CARLA_LIB_DIR}/libDebugUtils.a 
  /usr/lib/x86_64-linux-gnu/libjpeg.so 
  /usr/lib/x86_64-linux-gnu/libtiff.so
  /home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/lib/libProtoCollection.a
  /home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/lib/libplanning_common.so
  /home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/lib/libreference_line.so
  /home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/lib/libmath.so
  /home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/lib/libem_planner.so
  /home/codewalker/workspace/APP/3rd-party/Linearx_agt/install/lib/libmpc_controller.so
  ${PROTOBUF_LIBRARIES}
 )


# target_link_directories()




