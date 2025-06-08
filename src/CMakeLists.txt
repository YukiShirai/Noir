cmake_minimum_required(VERSION 3.15)

set(SRCDIR ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE SRCS ${SRCDIR}/*.cpp)

# Create a library from the source files
add_library(noir STATIC ${SRCS})
add_library(noir::noir ALIAS ${LIBRARY})
target_include_directories(noir PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

target_link_libraries(noir PRIVATE
        Eigen3::Eigen
)
