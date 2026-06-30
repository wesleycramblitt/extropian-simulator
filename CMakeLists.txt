cmake_minimum_required(VERSION 3.21)
project(extropian-simulator VERSION 0.1.0 LANGUAGES CXX
    DESCRIPTION "Extropian Simulator: full scientific simulation workflow"
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ── Dependencies: fetch all Extropian libraries from GitHub ───
# Set FETCHCONTENT_SOURCE_DIR_* to use local checkouts instead of fetching.
# build.sh does this automatically.

FetchContent_Declare(exd-core
    GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-core.git
    GIT_TAG main
)
FetchContent_Declare(exd-render
    GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-render.git
    GIT_TAG main
)
FetchContent_Declare(exd-app
    GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-app.git
    GIT_TAG main
)
FetchContent_Declare(exd-physics
    GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-physics.git
    GIT_TAG main
)
FetchContent_Declare(exd-solver-fluidx3d
    GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-solver-fluidx3d.git
    GIT_TAG main
)

# Make all available (in dependency order)
FetchContent_MakeAvailable(exd-core)
FetchContent_MakeAvailable(exd-render)
FetchContent_MakeAvailable(exd-app)
FetchContent_MakeAvailable(exd-physics)
FetchContent_MakeAvailable(exd-solver-fluidx3d)

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
    exd::solver::fluidx3d
)

# Copy assets to build directory
add_custom_command(TARGET ExtropianSimulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets
        $<TARGET_FILE_DIR:ExtropianSimulator>/assets
)
