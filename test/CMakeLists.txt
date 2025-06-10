set(TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE TESTS RELATIVE ${TESTS_DIR} "${TESTS_DIR}/*.cpp")

# Setup GTest
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()

foreach(TEST ${TESTS})
    get_filename_component(TEST_NAME ${TEST} NAME_WE)
    set(TEST_PATH "${TESTS_DIR}/${TEST}")

    add_executable(${TEST_NAME} ${TEST_PATH})

    # Link libraries
    target_link_libraries(${TEST_NAME} PRIVATE
            Eigen3::Eigen
            noir::noir
            GTest::gtest_main
    )

    # Add coverage flags if enabled
    if(ENABLE_COVERAGE)
        target_compile_options(${TEST_NAME} PRIVATE --coverage)
        target_link_libraries(${TEST_NAME} PRIVATE --coverage)
    endif()

    # Register tests with CTest
    include(GoogleTest)
    gtest_discover_tests(${TEST_NAME})
endforeach()
