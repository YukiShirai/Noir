set(TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE TESTS RELATIVE ${TESTS_DIR} "${TESTS_DIR}/*.cpp")

# setup gtest
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()

foreach(TEST ${TESTS})
    get_filename_component(TEST_NAME ${TEST} NAME_WE)
    set(TEST_PATH "${TESTS_DIR}/${TEST}")

    add_executable(${TEST_NAME} ${TEST_PATH})

    target_link_libraries(${TEST_NAME} PRIVATE
            Eigen3::Eigen
            noir::noir
            GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(${TEST_NAME})



#    # Common settings for both types
#    target_include_directories(${TEST_NAME} PRIVATE
#            ${CMAKE_SOURCE_DIR}/include
#    )
#
#    target_compile_options(${TEST_NAME} PRIVATE
#            $<$<CONFIG:Debug>:-g -O0>
#            $<$<CONFIG:Release>:-O3>
#    )

endforeach()
