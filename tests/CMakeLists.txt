add_library(test_common INTERFACE)
target_link_libraries(
    test_common
    INTERFACE
        umd::device
        umd::Firmware
        gtest_main
        gtest
        pthread
        fmt::fmt-header-only
)
target_include_directories(
    test_common
    INTERFACE
        ${PROJECT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        "$<TARGET_PROPERTY:umd::device,INCLUDE_DIRECTORIES>"
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/api)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/misc)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pcie)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/blackhole)

add_custom_target(
    umd_tests
    DEPENDS
        umd_unit_tests
        api_tests
        umd_misc_tests
)
