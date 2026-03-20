#include "sokol_gfx.h"
#include "sokol_color.h"

#include "shader.h"
#include "gizmo.h"

#include "string.h"
#include <cmath>
#include <algorithm>

// static uint32_t _sshape_rand_color(uint32_t* xorshift_state) {
//     // xorshift32
//     uint32_t x = *xorshift_state;
//     x ^= x<<13;
//     x ^= x>>17;
//     x ^= x<<5;
//     *xorshift_state = x;
//     // rand => bright color with alpha 1.0
//     x |= 0xFF000000;
//     return x;
// }

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

void make_cylinder(HMM_Vec3 origin, HMM_Vec3 h, HMM_Vec3 right, float height, float radius, float slices, sg_color col)
{
    const float two_pi = 2.0f * HMM_PI;
    // Orient towards h - cylinder axis
    HMM_Vec3 left = HMM_Cross(right, h);
    HMM_Mat4 rot = HMM_M4D(0.0f);
    rot.Columns[0] = HMM_V4(right.X, right.Y, right.Z, 0.0f);
    rot.Columns[1] = HMM_V4(h.X, h.Y, h.Z, 0.0f);
    rot.Columns[2] = HMM_V4(left.X, left.Y, left.Z, 0.0f);
    rot.Columns[3] = HMM_V4(0.0f, 0.0f, 0.0f, 1.0f);

    for (int slice = 0; slice <= slices; slice++) {
        float slice_angle = (two_pi * slice) / slices;
        float sin_slice = sinf(slice_angle);
        float cos_slice = cosf(slice_angle);
        HMM_Vec3 pos_base = HMM_V3(sin_slice * radius, 1.0f, cos_slice * radius);
        HMM_Vec3 pos_top = HMM_V3(sin_slice * radius, 1.0f, cos_slice * radius);

        /// matrix roation
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
///// Gizmo functions ////////////////////////////////////////////////////////////////////
Gizmo::Gizmo()
{
    origin = HMM_V3(0.0f, 0.0f, 0.0f);
    x_axis = HMM_V3(1.0f, 0.0f, 0.0f);
    y_axis = HMM_V3(0.0f, 1.0f, 0.0f);
    z_axis = HMM_V3(0.0f, 0.0f, 1.0f);
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

void Gizmo::generate_gizmo()
{
    std::array<sg_color,4> colors = {sg_red, sg_green, sg_blue, sg_white_smoke};
    std::array<HMM_Vec3,3> axes = {x_axis, y_axis, z_axis};
    const float two_pi = 2.0f * HMM_PI;
    float radius = 0.05f;
    int slices = 6;
    gizmo_verts.resize(slices * 2 * 7 * 3, 0.0f);
    gizmo_indices.resize(slices * 6 * 3, 0);

    auto insert_pt = [&](int idx, int col_idx, HMM_Vec3 pt){
        gizmo_verts[idx] = pt.X;
        gizmo_verts[idx+1] = pt.Y;
        gizmo_verts[idx+2] = pt.Z;
        gizmo_verts[idx+3] = colors[col_idx].r;
        gizmo_verts[idx+4] = colors[col_idx].g;
        gizmo_verts[idx+5] = colors[col_idx].b;
        gizmo_verts[idx+6] = colors[col_idx].a;
    };

    // Generate a cylinder for each axis
    // Vertices
    for (int i = 0; i < 3; i++)
    {
        int cyl_idx = i * 2 * 7 * slices;
        // Cylinder orientation
        const HMM_Vec3 h = axes[i];
        const HMM_Vec3 right = (i<2) ? axes[i+1] : axes[i-1];
        const HMM_Vec3 left = HMM_Cross(right, h);

        HMM_Mat4 rot = HMM_M4D(0.0f);
        rot.Columns[0] = HMM_V4(right.X, right.Y, right.Z, 0.0f);
        rot.Columns[1] = HMM_V4(h.X, h.Y, h.Z, 0.0f);
        rot.Columns[2] = HMM_V4(left.X, left.Y, left.Z, 0.0f);
        rot.Columns[3] = HMM_V4(0.0f, 0.0f, 0.0f, 1.0f);

        // Generate cylinder pole
        for (int slice = 0; slice < slices; slice++)
        {
            int s_idx = cyl_idx + slice * 14;
            float slice_angle = (two_pi * slice) / slices;
            float sin_slice = sinf(slice_angle);
            float cos_slice = cosf(slice_angle);
            HMM_Vec4 pos_base = HMM_V4(sin_slice * radius, 0.0f, cos_slice * radius,0.0f);
            HMM_Vec4 pos_top = HMM_V4(sin_slice * radius, 1.0f, cos_slice * radius, 0.0f);
            
            HMM_Vec4 rot_base = HMM_MulM4V4(rot, pos_base);
            HMM_Vec4 rot_top = HMM_MulM4V4(rot, pos_top);

            insert_pt(s_idx, i, HMM_V3(rot_base.X, rot_base.Y, rot_base.Z));
            insert_pt(s_idx + 7, i, HMM_V3(rot_top.X, rot_top.Y, rot_top.Z));
        }

        // Indices
        for (int j = 0; j < slices; j++)
        {
            int write_pos = i * slices * 6 + j * 6;
            int bv = i * slices * 2 + j * 2;

            if (j == (slices - 1))
            {
                // Tri 1 CCW
                gizmo_indices[write_pos] = bv;
                gizmo_indices[write_pos + 1] = i*slices*2; // cyl[0]
                gizmo_indices[write_pos + 2] = bv + 1;
                // Tri 2 CCW
                gizmo_indices[write_pos + 3] = i*slices*2; // cyl[0]
                gizmo_indices[write_pos + 4] = i*slices*2+1; // cyl[1]
                gizmo_indices[write_pos + 5] = bv + 1;
            }

            else
            {
                // Tri 1 CCW
                gizmo_indices[write_pos] = bv;
                gizmo_indices[write_pos + 1] = bv + 2;
                gizmo_indices[write_pos + 2] = bv + 1;
                // Tri 2 CCW
                gizmo_indices[write_pos + 3] = bv + 2;
                gizmo_indices[write_pos + 4] = bv + 3;
                gizmo_indices[write_pos + 5] = bv + 1;
            }
        }
    }
}

void Gizmo::create_axis_buffer()
{
    // Vertex buffer
    sg_buffer_desc vbuf_desc = {};
    vbuf_desc.data = sg_range{gizmo_verts.data(), gizmo_verts.size() * sizeof(float)}; 
    vbuf_desc.label = "Gizmo_vertices";
    gizmo_vtx_buf = sg_make_buffer(vbuf_desc);

    // Index buffer
    sg_buffer_desc ibuf_desc = {};
    ibuf_desc.usage.index_buffer = true;
    ibuf_desc.data.ptr = gizmo_indices.data();
    ibuf_desc.data.size = gizmo_indices.size() * sizeof(uint16_t);
    ibuf_desc.label = "Gizmo_indices";
    gizmo_idx_buf = sg_make_buffer(ibuf_desc);

    gizmo_bind = {};
    gizmo_bind.vertex_buffers[0] = gizmo_vtx_buf;
    gizmo_bind.index_buffer = gizmo_idx_buf;
}

void Gizmo::render_gizmo(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(gizmo_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.draw_mode = 0;
    params.point_size = 1.0f;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, gizmo_indices.size(), 1);
}

Gizmo::~Gizmo()
{
    sg_destroy_buffer(gizmo_vtx_buf);
    sg_destroy_buffer(gizmo_idx_buf);
}
