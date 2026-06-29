cmake_minimum_required(VERSION 3.21)
project(extropian-simulator VERSION 0.1.0 LANGUAGES CXX
    DESCRIPTION "Extropian Simulator: full scientific simulation workflow"
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────
find_package(ext-core REQUIRED)
find_package(ext-render REQUIRED)
find_package(ext-app REQUIRED)
find_package(ext-physics REQUIRED)

# Solver plugins (add as needed)
find_package(ext-solver-fluidx3d QUIET)

# ── Executable ────────────────────────────────────────
add_executable(ExtropianSimulator
    src/main.cpp
    src/simulator_app.cpp
    src/modes.cpp
    # Add app-specific systems here as they are built:
    # src/systems/geometry_editor.cpp
    # src/systems/bc_editor.cpp
    # src/systems/mesh_preview.cpp
    # src/systems/solver_monitor.cpp
    # src/systems/data_export.cpp
)

target_include_directories(ExtropianSimulator PRIVATE
    src
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(ExtropianSimulator PRIVATE
    ext::core
    ext::render
    ext::app
    ext::physics
)

if(TARGET ext::solver::fluidx3d)
    target_link_libraries(ExtropianSimulator PRIVATE ext::solver::fluidx3d)
endif()

# Copy assets to build directory
add_custom_command(TARGET ExtropianSimulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets
        $<TARGET_FILE_DIR:ExtropianSimulator>/assets
)

# ── Tests ─────────────────────────────────────────────
option(EXT_SIMULATOR_BUILD_TESTS "Build tests" OFF)
if(EXT_SIMULATOR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
