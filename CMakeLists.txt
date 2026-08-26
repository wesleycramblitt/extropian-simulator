cmake_minimum_required(VERSION 3.21)
project(extropian-simulator VERSION 0.1.0 LANGUAGES C CXX
    DESCRIPTION "Extropian Simulator: simulation environment builder"
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# extropian-core builds doctest-based unit tests by default; doctest 2.4.x
# is incompatible with CMake >= 4.0, so disable the dependency's tests.
set(EXT_CORE_BUILD_TESTS OFF CACHE BOOL "Build extropian-core tests" FORCE)

# ── Dependencies: fetch all Extropian libraries from GitHub ───
# Libraries check `if(NOT TARGET exd::*)` internally, so local sibling
# checkouts set by build.sh via FETCHCONTENT_SOURCE_DIR_<NAME> (uppercase,
# e.g. EXD-RENDER) take precedence when present.
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

if(NOT TARGET exd::geometry)
    FetchContent_Declare(exd-geometry
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-geometry.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-geometry)
endif()

# Dear ImGui is provided by exd-render (imgui::imgui, with the
# ImGuiSystem + ImGuiPanelComponent in exd/render/systems/).

# ── Executable ────────────────────────────────────────
add_executable(ExtropianSimulator
    src/main.cpp
    src/simulator_app.cpp
    src/modes.cpp
    src/turbine_system.cpp
)

target_include_directories(ExtropianSimulator PRIVATE src)

target_link_libraries(ExtropianSimulator PRIVATE
    exd::core
    exd::render
    exd::app
    exd::physics
    exd::geometry
)

# Copy assets and shaders to build directory
add_custom_command(TARGET ExtropianSimulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets
        $<TARGET_FILE_DIR:ExtropianSimulator>/assets
)
add_custom_command(TARGET ExtropianSimulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${exd-render_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:ExtropianSimulator>/shaders
)
