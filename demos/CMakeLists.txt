# All demo executables land in the build root (run.sh references them there).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# ── Demo host shell ────────────────────────────────────────────────
# Shared by all demos: window + render pipeline + ImGui wiring on a
# SystemGraph. Not part of the exd::sim library — it is the starting
# template researchers copy for their own applications.
add_library(demo_host STATIC common/demo_app.cpp)
target_include_directories(demo_host PUBLIC common)
target_link_libraries(demo_host PUBLIC exd::app exd::render)

# ── Demo executables — each one a self-contained registration example ──
add_executable(extropian-sim-turbine turbine/main.cpp)
target_link_libraries(extropian-sim-turbine PRIVATE demo_host exd::sim)

add_executable(extropian-sim-optimize optimization/main.cpp)
target_link_libraries(extropian-sim-optimize PRIVATE demo_host exd::sim)

# Assets + shaders next to every demo binary.
# Assets come from the fetched extropian-assets repo (never stored locally);
# add further asset subdirectories to this list when a demo consumes them.
set(EXD_SIM_DEMO_ASSET_DIRS
    cubemaps
)
foreach(demo IN ITEMS extropian-sim-turbine extropian-sim-optimize)
    foreach(dir IN LISTS EXD_SIM_DEMO_ASSET_DIRS)
        add_custom_command(TARGET ${demo} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${exd-assets_SOURCE_DIR}/${dir}
                $<TARGET_FILE_DIR:${demo}>/assets/${dir}
        )
    endforeach()
    add_custom_command(TARGET ${demo} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${exd-render_SOURCE_DIR}/shaders
            $<TARGET_FILE_DIR:${demo}>/shaders
    )
endforeach()
