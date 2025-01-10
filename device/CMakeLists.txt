set(POSITION_INDEPENDENT_CODE ON)

add_library(device SHARED)
add_library(${PROJECT_NAME}::device ALIAS device)

target_sources(
    device
    PRIVATE
        architecture_implementation.cpp
        chip/chip.cpp
        chip/local_chip.cpp
        chip/mock_chip.cpp
        chip/remote_chip.cpp
        cluster.cpp
        cluster_x280.cpp
        coordinate_manager.cpp
        grayskull/grayskull_implementation.cpp
        wormhole/wormhole_implementation.cpp
        blackhole/blackhole_implementation.cpp
        hugepage.cpp
        pcie/pci_device.cpp
        tlb.cpp
        tt_cluster_descriptor.cpp
        tt_device/blackhole_tt_device.cpp
        tt_device/grayskull_tt_device.cpp
        tt_device/tt_device.cpp
        tt_device/wormhole_tt_device.cpp
        tt_silicon_driver_common.cpp
        tt_soc_descriptor.cpp
        grayskull/grayskull_coordinate_manager.cpp
        wormhole/wormhole_coordinate_manager.cpp
        blackhole/blackhole_coordinate_manager.cpp
        xy_pair.cpp
)
include_directories(..)
include_directories(../common)
target_include_directories(
    device
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/api>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/device>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

target_link_libraries(
    device
    PUBLIC
        umd::Common
    PRIVATE
        umd::Firmware
        rt
        Boost::interprocess
        spdlog::spdlog_header_only
        fmt::fmt-header-only
        yaml-cpp::yaml-cpp
)


# Add a custom command to copy the library to build/lib
add_custom_command(
    TARGET device
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy $<TARGET_FILE:device> ${CMAKE_BINARY_DIR}/lib/
    COMMENT "Copying device library to build/lib"
)

# No separation of public and private header files
# I can only assume everything is public
install(
    DIRECTORY
        ${PROJECT_SOURCE_DIR}/device
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/device
    FILES_MATCHING
    PATTERN
    "*.h"
    PATTERN
    "*.hpp"
)
install(
    DIRECTORY
        ${PROJECT_SOURCE_DIR}/common
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/common
    FILES_MATCHING
    PATTERN
    "*.h"
    PATTERN
    "*.hpp"
)
