cmake_minimum_required(VERSION 2.8.3)
project(dvs_global_flow)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )

find_package(OpenCV REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# Make the executable
file(GLOB SOURCES src/*.cpp)
cs_add_executable(${PROJECT_NAME} ${SOURCES})

# Link the executable to the necessary libs
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${gsl_LIBRARIES}
)

cs_install()
cs_export()
