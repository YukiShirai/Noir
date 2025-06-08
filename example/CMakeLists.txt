set(EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE EXAMPLES RELATIVE ${EXAMPLES_DIR} "${EXAMPLES_DIR}/*.cpp")

foreach(EXAMPLE ${EXAMPLES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WE)
    set(EXAMPLE_PATH "${EXAMPLES_DIR}/${EXAMPLE}")

    add_executable(${EXAMPLE_NAME} ${EXAMPLE_PATH})

    target_link_libraries(${EXAMPLE_NAME} PRIVATE
            Eigen3::Eigen
    )


    # Common settings for both types
    target_include_directories(${EXAMPLE_NAME} PRIVATE
            ${CMAKE_SOURCE_DIR}/include
    )

    target_compile_options(${EXAMPLE_NAME} PRIVATE
            $<$<CONFIG:Debug>:-g -O0>
            $<$<CONFIG:Release>:-O3>
    )

endforeach()
