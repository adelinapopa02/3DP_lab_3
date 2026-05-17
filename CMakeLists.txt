cmake_minimum_required(VERSION 3.20)
project(mvs_reg)

set(CMAKE_CXX_STANDARD 23)
include_directories(src)
find_package(Open3D REQUIRED)
find_package(Ceres REQUIRED)
aux_source_directory( src SRC )
add_library(registration_lib STATIC ${SRC})
target_link_libraries(registration_lib PUBLIC Open3D::Open3D ${CERES_LIBRARIES})

add_executable(registration registration_trial.cpp)
target_sources(registration PRIVATE registration_trial.cpp)
target_link_libraries(registration PRIVATE Open3D::Open3D  registration_lib )
