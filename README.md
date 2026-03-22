# nView — NURBS Viewer

A real-time NURBS curve and surface viewer built in C++ with interactive control point editing, matcap shading, and live mesh generation.

![C++](https://img.shields.io/badge/C++-11-blue) ![OpenGL](https://img.shields.io/badge/OpenGL-3.3-green)

<img width="3840" height="2088" alt="Git_Page" src="https://github.com/user-attachments/assets/4acec6d7-7d52-450f-91df-eb36110c7c26" />


## Features

- **NURBS curves** — Cox-de Boor evaluation, interactive control point dragging, knot visualization, basis function influence display
- **NURBS surfaces** — Bi-directional evaluation with weighted control points, real-time mesh regeneration on edit
- **Matcap shading** — View-space normal mapping with async texture loading and Gooch fallback
- **Interaction modes** — View (orbit/pan/zoom), Curve Edit (drag/add points), Surface Edit (drag control net)
- **ImGui interface** — Mode switching, display toggles, reset controls

## Dependencies

| Library | Description | Source |
|---------|-------------|--------|
| [sokol](https://github.com/floooh/sokol) | sokol_gfx, sokol_app, sokol_fetch, sokol_glue, sokol_log, sokol_debugtext, sokol_imgui | Single-file headers |
| [HandmadeMath](https://github.com/HandmadeMath/HandmadeMath) | Math library (vectors, matrices, projections) | Single header |
| [Dear ImGui](https://github.com/ocornut/imgui) | Immediate mode GUI (docking branch) | Source files |
| [stb_image](https://github.com/nothings/stb) | Image loading for matcap textures | Single header |
| [sokol-shdc](https://github.com/nicebyte/sokol-shdc) | Shader cross-compiler (GLSL → HLSL/GLSL) | Binary tool |

Place dependencies in a `deps/` folder at the project root, with ImGui (docking branch) in `deps/imgui_docking/`. Place the `sokol-shdc` binary in `bin/`.

## Shader Compilation

`shader.h` is generated from `shader.glsl` by sokol-shdc and is not included in the repo. CMake handles this automatically via a custom command:

```cmake
add_custom_command(
    OUTPUT ${CMAKE_SOURCE_DIR}/src/shader.h
    COMMAND ${CMAKE_SOURCE_DIR}/bin/sokol-shdc
        --input ${CMAKE_SOURCE_DIR}/src/shader.glsl
        --output ${CMAKE_SOURCE_DIR}/src/shader.h
        --slang hlsl5:glsl430
    DEPENDS ${CMAKE_SOURCE_DIR}/src/shader.glsl
    COMMENT "Compiling shaders..."
)
```

On first build, CMake will invoke sokol-shdc to generate `src/shader.h`. It regenerates automatically whenever `shader.glsl` changes.

## Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## Controls

| Input | Action |
|-------|--------|
| RMB | Orbit camera |
| Shift + RMB | Pan camera |
| Scroll | Zoom |
| LMB (edit modes) | Drag control points |
| F | Reset camera |
| 1 / 2 / 3 | Switch mode: View / Curve / Surface |

## Assets

Place a matcap texture at `assets/matcap_2.png` and a font at `assets/Ubuntu-Medium.ttf`.

## References

- Piegl & Tiller, *The NURBS Book* — Algorithm reference for curve/surface evaluation
