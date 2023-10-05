cmake_minimum_required(VERSION 2.8.3)
project(gtsam_demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
find_package(Boost REQUIRED COMPONENTS timer thread serialization chrono system filesystem date_time)
find_package(GTSAM REQUIRED QUIET)
include_directories( 
    include
    ${GTSAM_INCLUDE_DIR})
link_directories(
        include
      ${GTSAM_LIBRARY_DIRS}
    )
    
add_executable(ternaryfactors_test ternaryfactors_test.cpp)
target_link_libraries(ternaryfactors_test gtsam)
add_executable(curve_fit curve_fit.cpp)
target_link_libraries(curve_fit gtsam)