cmake_minimum_required(VERSION 3.21)
project(extropian-simulator VERSION 0.1.0 LANGUAGES C CXX
    DESCRIPTION "Extropian Simulator: full scientific simulation workflow"
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ── Dependencies: fetch all Extropian libraries from GitHub ───
# Libraries check `if(NOT TARGET exd::*)` internally, so local overrides
# set by build.sh via FETCHCONTENT_SOURCE_DIR_* take precedence.

if(NOT TARGET exd::core)
    FetchContent_Declare(exd-core
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-core.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-core)
endif()

if(NOT TARGET exd::render)
    FetchContent_Declare(exd-render
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-render.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-render)
endif()

if(NOT TARGET exd::app)
    FetchContent_Declare(exd-app
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-app.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-app)
endif()

if(NOT TARGET exd::physics)
    FetchContent_Declare(exd-physics
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-physics.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-physics)
endif()

# Solver disabled (requires FluidX3D-CLI)
if(FALSE) // if(NOT TARGET #exd::solver::fluidx3d)
#    FetchContent_Declare(exd-solver-fluidx3d
#        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-solver-fluidx3d.git
#        GIT_TAG main
#    )
#    FetchContent_MakeAvailable(exd-solver-fluidx3d)
#endif()

# Find system packages
find_package(OpenGL REQUIRED)
find_package(OpenCL QUIET)

# ── Executable ────────────────────────────────────────
add_executable(ExtropianSimulator
    src/main.cpp
    src/simulator_app.cpp
    src/modes.cpp
)

target_include_directories(ExtropianSimulator PRIVATE src)

target_link_libraries(ExtropianSimulator PRIVATE
    exd::core
    exd::render
    exd::app
    exd::physics
    #exd::solver::fluidx3d
)

# Copy assets to build directory
add_custom_command(TARGET ExtropianSimulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets
        $<TARGET_FILE_DIR:ExtropianSimulator>/assets
)
