cmake_minimum_required(VERSION 3.21)
project(extropian-simulator VERSION 0.1.0 LANGUAGES C CXX
    DESCRIPTION "Extropian Simulator: ECS-based simulation library + demos"
)

# Content-only repos (extropian-assets) are populated with
# FetchContent_Populate (they have no CMakeLists.txt); CMP0169 (CMake 3.30+)
# deprecates that call, so opt out from the start.
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

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

if(NOT TARGET exd::optimization)
    FetchContent_Declare(exd-optimization
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-optimization.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-optimization)
endif()

# ── Assets: pulled from the extropian-assets repo, never stored here ──
# extropian-assets is a content-only Git repo (no CMakeLists.txt), so it is
# downloaded with FetchContent_Populate and referenced via
# exd-assets_SOURCE_DIR. It is fetched from GitHub only — there is NO
# local-sibling override for it in build.sh by design.
if(NOT exd-assets_POPULATED)
    # Same declaration convention as extropian-render's demo (mirrors it).
    FetchContent_Declare(exd-assets
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-assets.git
        GIT_TAG main
        GIT_SUBMODULES ""
        GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(exd-assets)
    if(NOT exd-assets_POPULATED)
        FetchContent_Populate(exd-assets)
    endif()
endif()

if(NOT TARGET exd::viz)
    FetchContent_Declare(exd-viz
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-viz.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-viz)
endif()

if(NOT TARGET exd::scene_renderer)
    FetchContent_Declare(exd-spatial-ui
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-spatial-ui.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-spatial-ui)
endif()

# Dear ImGui is provided by exd-render (imgui::imgui, with the
# ImGuiSystem + ImGuiPanelComponent in exd/render/systems/).

# ── Library: exd::sim ──────────────────────────────────
# The reusable simulation library. Researchers build their own apps
# against <exd/sim/...> and register systems into an ECS SystemGraph;
# see demos/ for copyable examples and AGENTS.md for the ECS standards.
add_library(exd-sim STATIC
    src/turbine_system.cpp
    src/optimization_system.cpp
    src/solver_run_system.cpp
    src/steam_engine_system.cpp
    src/dashboard_feed_system.cpp
    src/shape_workshop_system.cpp
)
add_library(exd::sim ALIAS exd-sim)

target_include_directories(exd-sim PUBLIC include)

target_link_libraries(exd-sim PUBLIC
    exd::core          # ecs types appear in public headers
    exd::render        # GraphicsContext + render components in public headers
    exd::optimization  # exd::opt in public headers
    exd::physics       # solver run / engine / BEM entry points (public headers)
    exd::viz           # Field3D + slice/streamline/iso cores for visualizers
    exd::scene_renderer # dashboard feed writes DocumentNodeComponent
)
target_link_libraries(exd-sim PRIVATE
    exd::geometry      # mesh builders used in implementations only
)

# ── Demos ───────────────────────────────────────────────
# One executable per registration example (see demos/README).
add_subdirectory(demos)

# Standalone integration test for the optimization library (has its own main()).
add_executable(optimization_test src/optimization_test.cpp)
target_link_libraries(optimization_test PRIVATE exd::optimization)

# Standalone test for the SolverRunSystem run-mapping recipe: tiny coupled
# FDM3 run must validate and be bit-for-bit deterministic (links physics only).
add_executable(solver_run_test src/solver_run_test.cpp)
target_link_libraries(solver_run_test PRIVATE exd::physics)
target_include_directories(solver_run_test PRIVATE src include)

# Standalone test for the steam-engine run recipe (engine_run.hpp).
add_executable(engine_run_test src/engine_run_test.cpp)
target_link_libraries(engine_run_test PRIVATE exd::physics)
target_include_directories(engine_run_test PRIVATE src include)

# Headless test for the spatial-ui dashboard pipeline (feed → resolution).
add_executable(dashboard_feed_test src/dashboard_feed_test.cpp)
target_link_libraries(dashboard_feed_test PRIVATE exd::sim)
target_include_directories(dashboard_feed_test PRIVATE src include)

# Headless test for the shape-workshop recipe dispatch (spec → mesh).
add_executable(shape_workshop_test src/shape_workshop_test.cpp)
target_link_libraries(shape_workshop_test PRIVATE exd::sim exd::geometry)
target_include_directories(shape_workshop_test PRIVATE src include)
