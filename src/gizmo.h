#pragma once
#include "HandmadeMath.h"
#include <array>

enum class ActiveAxis {None, X, Y, Z};

class Gizmo
{
public:
    HMM_Vec3 origin, x_axis, y_axis, z_axis;
    HMM_Mat4 gumball_mvp;
    float scale;
    float world_scale;
    ActiveAxis active_axis = ActiveAxis::None;
    Gizmo(float scale = 1);
    ~Gizmo();

    void generate_gizmo();
    void select_axis(const HMM_Mat4& mvp, float screen_w, float screen_h, float mouse_x, float mouse_y);
    
    HMM_Mat4 set_gumball_mvp(const HMM_Vec3& cam_pos, const HMM_Mat4& view, const HMM_Mat4& proj, float gumball_size);
    // Axis indicator
    void render_gizmo(const HMM_Mat4& mvp);
    void render_gumball(const HMM_Mat4& mvp, bool show);

private:
    // Axis indicator data
    sg_buffer axis_gizmo_buffer;
    sg_bindings axis_gizmo_bind;
    std::array<float, 42> axis_gizmo_verts;

    HMM_Vec2 screen_x, screen_y, screen_z, screen_orig;
    
    void axis_gizmo();
    void create_axis_buffer();
    void update_screen_axes(const HMM_Mat4& mvp, float screen_w, float screen_h);
};