cmake_minimum_required(VERSION 2.6.0)

project(ICPHW)

# Add local path for finding packages, set the local version first
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(icpcuda REQUIRED)
find_package(freenect2 REQUIRED)
find_package(opt REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Sophus_INCLUDE_DIR})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${ICPCUDA_INCLUDE_DIR})
include_directories(${FREENECT2_INCLUDE_DIR})
include_directories(${OPT_INCLUDE_DIR})

file(GLOB srcs src/*.cpp)

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11")

add_executable(ICPHW
               ${srcs}
)

target_link_libraries(ICPHW
                      ${CUDA_LIBRARIES}
                      #${Eigen_LIBRARIES}
                      #${Sophus_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      ${ICPCUDA_LIBRARIES}
                      ${FREENECT2_LIBRARIES}
                      ${OPT_LIBRARIES}
)
