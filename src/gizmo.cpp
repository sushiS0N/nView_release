#include "sokol_gfx.h"
#include "sokol_color.h"

#include "shader.h"
#include "gizmo.h"

#include "string.h"
#include <cmath>
#include <algorithm>

// Utilities
HMM_Mat4 extract_rotation(const HMM_Mat4& view)
{
    HMM_Mat4 rotation = view;
    rotation.Elements[3][0] = 0.0f;
    rotation.Elements[3][1] = 0.0f;
    rotation.Elements[3][2] = 0.0f;

    rotation.Elements[1][1] *= -1.0f;

    return rotation;
}

HMM_Mat4 screen_offset()
{
    HMM_Mat4 scr = HMM_M4D(1.0f);
    HMM_Mat4 scale = HMM_Scale(HMM_V3(0.15f,0.15f,0.15f));
    HMM_Mat4 translate = HMM_Translate(HMM_V3(-0.85f,-0.85f, 0.0f));

    return HMM_MulM4(translate, HMM_MulM4(scale, scr));
}

HMM_Vec2 project_to_screen(const HMM_Mat4& mvp, float screen_w, float screen_h, HMM_Vec3 pt)
{
    HMM_Vec4 world_pos = HMM_V4(pt.X, pt.Y, pt.Z, 1.0f);
    HMM_Vec4 clip_pos = HMM_MulM4V4(mvp, world_pos);
    HMM_Vec3 ndc = HMM_V3(clip_pos.X / clip_pos.W, clip_pos.Y / clip_pos.W, clip_pos.Z / clip_pos.W);
    float screen_x = (ndc.X + 1.0f) * screen_w / 2.0f;
    float screen_y = (1.0f - ndc.Y) * screen_h / 2.0f;

    return HMM_V2(screen_x, screen_y);
}

HMM_Vec2 line_closest_point(HMM_Vec2 start, HMM_Vec2 end, HMM_Vec2 pt)
{
    auto line = HMM_SubV2(end, start);
    float len = HMM_Len(line);
    line = HMM_NormV2(line);

    auto v = HMM_SubV2(pt, start);
    auto d = HMM_DotV2(v, line);
    d = std::clamp(d, 0.0f, len);

    return HMM_Add(start, HMM_MulV2F(line, d));
}

//////////////////////////////////////////////////////////////////////////////////////////////
///// Gizmo functions ////////////////////////////////////////////////////////////////////
Gizmo::Gizmo(float scale)
{
    origin = HMM_V3(0.0f, 0.0f, 0.0f);
    x_axis = HMM_MulV3F(HMM_V3(1.0f, 0.0f, 0.0f), scale);
    y_axis = HMM_MulV3F(HMM_V3(0.0f, 1.0f, 0.0f), scale);
    z_axis = HMM_MulV3F(HMM_V3(0.0f, 0.0f, 1.0f), scale);
    generate_gizmo();
    create_axis_buffer();
}

void Gizmo::select_axis(const HMM_Mat4& mvp, float screen_w, float screen_h, float mouse_x, float mouse_y)
{
    update_screen_axes(mvp, screen_w, screen_h);
    std::array<HMM_Vec2,3> screen_gum = {screen_x, screen_y, screen_z};
    HMM_Vec2 mouse_pos = HMM_V2(mouse_x, mouse_y);

    float min_dist = screen_w * 2;
    int selected = -1;

    for (int i = 0; i < screen_gum.size(); i++)
    {
        HMM_Vec2 axis_closest = line_closest_point(screen_orig, screen_orig + screen_gum[i], mouse_pos);
        float pixel_dist = std::sqrt((axis_closest.X - mouse_pos.X) * (axis_closest.X - mouse_pos.X) + (axis_closest.Y - mouse_pos.Y) * (axis_closest.Y - mouse_pos.Y));

        if (pixel_dist < min_dist && pixel_dist < 10.0f)
        {
            selected = i;
            min_dist = pixel_dist;
        }
    }
    printf("Selected axis: %i\n", selected);
    fflush(stdout);

    switch (selected)
    {
    case -1:
        active_axis = ActiveAxis::None;
        break;
    case 0:
        active_axis = ActiveAxis::X;
        break;
    case 1:
        active_axis = ActiveAxis::Y;
        break;
    case 2:
        active_axis = ActiveAxis::Z;
        break;
    }
}

void Gizmo::drag_axis(float mouse_dx, float mouse_dy, float screen_w, float screen_h)
{
    HMM_Vec3 dragging_axis_wrld = {};
    HMM_Vec2 dragging_axis_scr = {};
    switch (active_axis)
    {
    case ActiveAxis::X:
        dragging_axis_wrld = x_axis;
        dragging_axis_scr = screen_x;
        break;
    case ActiveAxis::Y:
        dragging_axis_wrld = y_axis;
        dragging_axis_scr = screen_y;
        break;
    case ActiveAxis::Z:
        dragging_axis_wrld = z_axis;
        dragging_axis_scr = screen_z;
        break;
    default:
        break;
    }
    mouse_dx /= screen_w;
    mouse_dy /= screen_h;
    float drag_amount = HMM_DotV2(dragging_axis_scr, HMM_V2(mouse_dx,mouse_dy));
    origin = HMM_Add(origin, HMM_MulV3F(dragging_axis_wrld, drag_amount));
}

void Gizmo::update_screen_axes(const HMM_Mat4 &mvp, float screen_w, float screen_h)
{
    auto project_to_screen = [&](HMM_Vec3 pt)
    {
        HMM_Vec4 world_pos = HMM_V4(pt.X, pt.Y, pt.Z, 1.0f);
        HMM_Vec4 clip_pos = HMM_MulM4V4(mvp, world_pos);
        HMM_Vec3 ndc = HMM_V3(clip_pos.X / clip_pos.W, clip_pos.Y / clip_pos.W, clip_pos.Z / clip_pos.W);
        float screen_x = (ndc.X + 1.0f) * screen_w / 2.0f;
        float screen_y = (1.0f - ndc.Y) * screen_h / 2.0f;

        return HMM_V2(screen_x, screen_y);
    };
    // Project gumball on screen
    screen_orig = project_to_screen(origin);
    HMM_Vec2 proj_x_tip = project_to_screen(HMM_AddV3(origin, HMM_MulV3F(x_axis, world_scale)));
    HMM_Vec2 proj_y_tip = project_to_screen(HMM_AddV3(origin, HMM_MulV3F(y_axis, world_scale)));
    HMM_Vec2 proj_z_tip = project_to_screen(HMM_AddV3(origin, HMM_MulV3F(z_axis, world_scale)));

    // Get vectors on screen
    screen_x = HMM_Sub(proj_x_tip, screen_orig);
    screen_y = HMM_Sub(proj_y_tip, screen_orig);
    screen_z = HMM_Sub(proj_z_tip, screen_orig);
}

void Gizmo::generate_gizmo()
{
    std::array<sg_color,4> colors = {sg_red, sg_green, sg_blue, sg_white_smoke};
    std::array<HMM_Vec3,3> axes = {x_axis, y_axis, z_axis};
    auto insert_pt = [&](int idx, int col_idx, HMM_Vec3 pt){
        axis_gizmo_verts[idx] = pt.X;
        axis_gizmo_verts[idx+1] = pt.Y;
        axis_gizmo_verts[idx+2] = pt.Z;
        axis_gizmo_verts[idx+3] = colors[col_idx].r;
        axis_gizmo_verts[idx+4] = colors[col_idx].g;
        axis_gizmo_verts[idx+5] = colors[col_idx].b;
        axis_gizmo_verts[idx+6] = colors[col_idx].a;
    };
    for (int i = 0; i < 3; i++)
    {
        int idx = i * 14;
        insert_pt(idx, 3, origin);
        insert_pt(idx+7, i, axes[i]);        
    }
}

// Render functions
HMM_Mat4 Gizmo::set_gumball_mvp(const HMM_Vec3& cam_pos, const HMM_Mat4& view, const HMM_Mat4& proj, float gumball_size)
{
    float dist = HMM_LenV3(HMM_SubV3(origin, cam_pos));
    world_scale = dist * gumball_size;
    HMM_Mat4 gumball_scale = HMM_Scale(HMM_V3(world_scale, world_scale, world_scale));
    HMM_Mat4 gumball_trans = HMM_Translate(origin);
    HMM_Mat4 gumball_model = HMM_MulM4(gumball_trans, gumball_scale);
    gumball_mvp = HMM_MulM4(proj, HMM_MulM4(view, gumball_model));
    return gumball_mvp;
}

void Gizmo::create_axis_buffer()
{
    sg_buffer_desc vbuf_desc={};
    vbuf_desc.data = SG_RANGE(axis_gizmo_verts);
    vbuf_desc.label = "axis_gizmo";
    axis_gizmo_buffer = sg_make_buffer(vbuf_desc);

    axis_gizmo_bind = {};
    axis_gizmo_bind.vertex_buffers[0] = this->axis_gizmo_buffer;
}

void Gizmo::render_gizmo(const HMM_Mat4 &mvp)
{
    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 1.0f;
    params.draw_mode = 0;

    sg_apply_bindings(axis_gizmo_bind);
    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, 6, 1);
}

Gizmo::~Gizmo()
{
    sg_destroy_buffer(axis_gizmo_buffer);
}
