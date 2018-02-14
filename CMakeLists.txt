project(MPC)

cmake_minimum_required (VERSION 3.5)
set(CMAKE_BUILD_TYPE Debug)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp)
#set(sources2 src/MPC.cpp src/test_mpc.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})
target_link_libraries(mpc ipopt z ssl uv uWS)

#add_executable(mpc_test ${sources2})
#target_link_libraries(mpc_test ipopt z ssl uv uWS)

