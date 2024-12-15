set(UNIT_TESTS_X280_SRCS test_x280.cpp)

add_executable(unit_tests_x280 ${UNIT_TESTS_X280_SRCS})
target_link_libraries(unit_tests_x280 PRIVATE test_common)
set_target_properties(
    unit_tests_x280
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY
            ${CMAKE_BINARY_DIR}/test/umd/x280
        OUTPUT_NAME
            unit_tests
)

add_custom_target(umd_unit_tests2 DEPENDS unit_tests_x280)
