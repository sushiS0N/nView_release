#include "sokol_gfx.h"
#include "shader.h"
#include "gizmo.h"

#include "string.h"

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

// Gizmo functions
Gizmo::Gizmo()
{
    create_axis_buffer();
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

void Gizmo::render_axis_indicator(const HMM_Mat4 &mvp)
{
    /* 
    HMM_Mat4 screenOffset = screen_offset();
    HMM_Mat4 rotationOnly = extract_rotation(mvp);
    HMM_Mat4 ortho = HMM_Orthographic_RH_NO(-1.5f, 1.5f, -1.5f, 1.5f, -1.5f, 1.5f);
    HMM_Mat4 model = HMM_M4D(1.0f);

    HMM_Mat4 mvp_gizmo = HMM_MulM4(screenOffset, HMM_MulM4(ortho,rotationOnly));
   */
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
