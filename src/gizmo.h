#pragma once
#include "HandmadeMath.h"
#include <array>
#include <vector>

enum class ActiveAxis {None, X, Y, Z};

class Gizmo
{
public:
    HMM_Vec3 origin, x_axis, y_axis, z_axis;
    HMM_Mat4 gumball_mvp;
    float world_scale;
    ActiveAxis active_axis = ActiveAxis::None;
    Gizmo();
    ~Gizmo();

    void generate_gizmo();
    void drag_axis(float mouse_dx, float mouse_dy, float screen_w, float screen_h);
    void select_axis(const HMM_Mat4& mvp, float screen_w, float screen_h, float mouse_x, float mouse_y);
    
    HMM_Mat4 set_gumball_mvp(const HMM_Vec3& cam_pos, const HMM_Mat4& view, const HMM_Mat4& proj, float gumball_size);
    // Axis indicator
    void render_gizmo(const HMM_Mat4& mvp);
    void render_gumball(const HMM_Mat4& mvp, bool show);

private:
    // Axis indicator data
    sg_buffer gizmo_vtx_buf, gizmo_idx_buf;
    sg_bindings gizmo_bind;
    std::vector<float> gizmo_verts;
    std::vector<uint16_t> gizmo_indices;

    HMM_Vec2 screen_x, screen_y, screen_z, screen_orig;
    
    void axis_gizmo();
    void create_axis_buffer();
    void update_screen_axes(const HMM_Mat4& mvp, float screen_w, float screen_h);
};