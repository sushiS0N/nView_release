cmake_minimum_required(VERSION 4.1)
project(nView CXX)
set (CMAKE_CXX_STANDARD 17)

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/deps/imgui_docking)

add_executable(nView 
    src/main.cpp 
    src/camera.cpp 
    src/gizmo.cpp 
    src/nurbs.cpp src/shader.h)

# Include directories
target_include_directories(nView 
    PRIVATE ${CMAKE_SOURCE_DIR}/deps
    PRIVATE ${IMGUI_DIR})

# Include libraries
target_link_libraries(nView 
    user32 
    gdi32 
    shell32 
    dxgi 
    d3d11)

target_sources(nView PRIVATE
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp)


add_custom_command(
    OUTPUT ${CMAKE_SOURCE_DIR}/src/shader.h
    COMMAND ${CMAKE_SOURCE_DIR}/bin/sokol-shdc.exe --input ${CMAKE_SOURCE_DIR}/src/shader.glsl --output ${CMAKE_SOURCE_DIR}/src/shader.h --slang hlsl5:glsl430 
    DEPENDS ${CMAKE_SOURCE_DIR}/src/shader.glsl
    COMMENT "Compiling shaders..."
    )
