cmake_minimum_required(VERSION 4.1)
project(nView CXX)

add_executable(nView src/main.cpp src/camera.cpp src/gizmo.cpp src/bezier_patch.cpp src/bspline.cpp src/shader.h)
# add_compile_definitions(HANDMADE_MATH_USE_DEGREES)
# Include directories
target_include_directories(nView PRIVATE ${CMAKE_SOURCE_DIR}/deps)

# Include libraries
target_link_libraries(nView user32 gdi32 shell32 dxgi d3d11)

add_custom_command(
    OUTPUT ${CMAKE_SOURCE_DIR}/src/shader.h
    COMMAND ${CMAKE_SOURCE_DIR}/bin/sokol-shdc.exe --input ${CMAKE_SOURCE_DIR}/src/shader.glsl --output ${CMAKE_SOURCE_DIR}/src/shader.h --slang hlsl5:glsl430 
    DEPENDS ${CMAKE_SOURCE_DIR}/src/shader.glsl
    COMMENT "Compiling shaders..."
    )
