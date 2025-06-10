cmake_minimum_required(VERSION 3.22)
project(noir VERSION 1.0 DESCRIPTION "Implementation for Numerical Optimization for Robotics in C++")

option(ENABLE_COVERAGE "Enable coverage reporting" OFF)
if(ENABLE_COVERAGE)
    message(STATUS "Coverage enabled")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
else()
    # Keep your original flags
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -DDEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -g")
endif()


set(PROJECT NOIR)
set(LIBRARY noir)

# specifies that the project use the C++20 standard
set(CMAKE_CXX_STANDARD 20)
# specifies that the project should NOT use features from specific compilers
set(CMAKE_CXX_EXTENSIONS OFF)
# ensures that the specified C++ standard is mandatory.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# fPIC. Necessary for dynamic library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# generates json file, which is used often by IDE to provide features like code analysis, autocompletion, and linting
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Enables colored output in the terminal
set(CMAKE_COLOR_MAKEFILE ON)


# Allow user to optionally override Eigen path
set(EIGEN3_DIR "" CACHE PATH "Path to Eigen include directory or config")

# Try using CMake config mode first
find_package(Eigen3 QUIET
        HINTS ${EIGEN3_DIR}
        PATHS /usr/local /usr /opt/homebrew
        PATH_SUFFIXES include/eigen3
)

# Fallback: include manually
if(NOT TARGET Eigen3::Eigen)
    if(EXISTS "${EIGEN3_DIR}")
        message(WARNING "Falling back to manual Eigen include path: ${EIGEN3_DIR}")
        include_directories(${EIGEN3_DIR})
    else()
        message(FATAL_ERROR "Could not find Eigen3. Please specify EIGEN3_DIR.")
    endif()
endif()

#find_package(GTest REQUIRED)


include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(src)
add_subdirectory(example)
add_subdirectory(test)






#add_subdirectory(test)
#add_subdirectory(example)
