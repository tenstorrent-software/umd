function(fetch_dependencies)
    # Shadow the cache variable with a blank value
    # Placing a no-op .clang-tidy file at the root of CPM cache is insufficient as some projects may define
    # their own .clang-tidy within themselves and still not be clean against it <cough>flatbuffers</cough>
    set(CMAKE_C_CLANG_TIDY "")
    set(CMAKE_CXX_CLANG_TIDY "")
    set(ENV{CPM_SOURCE_CACHE} "${PROJECT_SOURCE_DIR}/.cpmcache")

    include(${PROJECT_SOURCE_DIR}/cmake/CPM.cmake)

    ####################################################################################################################
    # google test
    ####################################################################################################################
    CPMAddPackage(
        NAME googletest
        GITHUB_REPOSITORY google/googletest
        GIT_TAG v1.13.0
        VERSION 1.13.0
        OPTIONS
            "INSTALL_GTEST OFF"
    )

    ####################################################################################################################
    # yaml-cpp
    ####################################################################################################################
    CPMAddPackage(
        NAME yaml-cpp
        GITHUB_REPOSITORY jbeder/yaml-cpp
        GIT_TAG 0.8.0
        OPTIONS
            "YAML_CPP_BUILD_TESTS OFF"
            "YAML_CPP_BUILD_TOOLS OFF"
            "YAML_BUILD_SHARED_LIBS OFF"
    )

    if(yaml-cpp_ADDED)
        set_target_properties(
            yaml-cpp
            PROPERTIES
                DEBUG_POSTFIX
                    ""
        )
    endif()

    ###################################################################################################################
    # boost::interprocess
    ###################################################################################################################
    include(${PROJECT_SOURCE_DIR}/cmake/fetch_boost.cmake)
    fetch_boost_library(interprocess)

    ###################################################################################################################
    # fmt : https://github.com/fmtlib/fmt
    ###################################################################################################################

    CPMAddPackage(NAME fmt GITHUB_REPOSITORY fmtlib/fmt GIT_TAG 11.0.1)

    ####################################################################################################################
    # spdlog
    ####################################################################################################################
    CPMAddPackage(NAME spdlog GITHUB_REPOSITORY gabime/spdlog GIT_TAG v1.14.1 VERSION v1.14.1)
endfunction()
fetch_dependencies()
