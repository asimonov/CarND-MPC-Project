project(mpc)

#option(ENABLE_VISUALIZATION "Enable visualization, requires OpenCV3" ON)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
add_definitions(-std=c++11)
# turn on -O3 for best performance
add_definitions(-O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp src/HiResTimer.cpp src/poly.cpp src/poly.h)

include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})
target_link_libraries(mpc ipopt z ssl uv uWS python)

#if (ENABLE_VISUALIZATION)
#    message(STATUS "OpenCV Visualization Enabled")
#    add_definitions(-DENABLE_VISUALIZATION)
#    target_sources(mpc PUBLIC src/opencv.cpp)
    #link_directories(/usr/local/Cellar/opencv/2.4.13.2/lib)
    #link_directories(/usr/local/Cellar/opencv3/3.2.0//lib)
#    target_link_libraries(mpc opencv_core opencv_highgui opencv_imgproc)
#endif(ENABLE_VISUALIZATION)

