#pragma once
#include "HandmadeMath.h"
#include <array>
#include <vector>

#include "render_utils.h"

enum class ActiveAxis {None, X, Y, Z};

class Gizmo
{
public:
    HMM_Vec3 origin_, x_axis_, y_axis_, z_axis_;
    HMM_Mat4 gumball_mvp_;
    float world_scale_;
    bool show_ = false;
    ActiveAxis active_axis_ = ActiveAxis::None;
    GpuBuffer gizmo_buf_;

    Gizmo();
    ~Gizmo() = default;

    void reset();
    void generate_gizmo();
    void drag_axis(float mouse_dx, float mouse_dy, float screen_w, float screen_h);
    void select_axis(const HMM_Mat4& mvp, float screen_w, float screen_h, float mouse_x, float mouse_y);
    
    void set_gumball_mvp(const HMM_Vec3& cam_pos, const HMM_Mat4& view, const HMM_Mat4& proj, float gumball_size);
    // Axis indicator
    void render_gumball(const HMM_Mat4& mvp, bool show);
    void create_gpubuffer();

private:
    // Axis indicator data
    std::vector<float> gizmo_verts_;
    std::vector<uint16_t> gizmo_indices_;

    HMM_Vec2 screen_x_, screen_y_, screen_z_, screen_orig_;
    
    void axis_gizmo();
    void create_axis_buffer();
    void update_screen_axes(const HMM_Mat4& mvp, float screen_w, float screen_h);
};