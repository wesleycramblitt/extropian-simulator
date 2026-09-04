# ── The interactive workspace app ─────────────────────────────────────
# The product entry point: one main.cpp that boots the workspace UI/UX
# (window + render pipeline + ImGui host) with the ECS seeded with
# placeholder domain state for CAD / simulation / visualization /
# optimization. The real systems and the spatial-ui dashboard layer
# replace the stubs as they land.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(extropian-simulator-workspace main.cpp)
target_link_libraries(extropian-simulator-workspace PRIVATE
    exd::core      # ECS registry, components, SystemGraph
    exd::render    # render pipeline, ImGui host, camera
    exd::app       # SDL3 window + frame loop
)

# ── Assets + shaders next to the binary ──
# Media comes from the fetched extropian-assets repo (never stored here).
set(EXD_WORKSPACE_ASSET_DIRS
    cubemaps
    fonts
)
foreach(dir IN LISTS EXD_WORKSPACE_ASSET_DIRS)
    add_custom_command(TARGET extropian-simulator-workspace POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${exd-assets_SOURCE_DIR}/${dir}
            $<TARGET_FILE_DIR:extropian-simulator-workspace>/assets/${dir}
    )
endforeach()
add_custom_command(TARGET extropian-simulator-workspace POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${exd-render_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:extropian-simulator-workspace>/shaders
)
