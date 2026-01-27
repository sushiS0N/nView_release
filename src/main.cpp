#define HANDMADE_MATH_IMPLEMENTATION
#include "HandmadeMath.h"

#define SOKOL_IMPL
//#define SOKOL_D3D11
#define SOKOL_GLCORE

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_log.h"

#include "stdio.h"
#include <vector>

#include "shader.h"
#include "bezier_patch.h"
#include "camera.h"
#include "gizmo.h"
#include "bspline.h"

//Geometry
float cp[48] = {
    // Row 0 (X = 0)
    0.0f, 5.0f, 0.0f,    0.0f, 0.0f, 2.0f,    0.0f, 0.0f, 4.0f,    0.0f, 0.0f, 6.0f,
    // Row 1 (X = 2)  
    2.0f, 0.0f, 0.0f,    2.0f, 0.0f, 2.0f,    2.0f, 0.0f, 4.0f,    2.0f, 0.0f, 6.0f,
    // Row 2 (X = 4)
    4.0f, 0.0f, 0.0f,    4.0f, 0.0f, 2.0f,    4.0f, 5.0f, 4.0f,    4.0f, 0.0f, 6.0f,
    // Row 3 (X = 6)
    6.0f, 5.0f, 0.0f,    6.0f, 0.0f, 2.0f,    6.0f, 0.0f, 4.0f,    6.0f, 0.0f, 6.0f
};
BezierPatch *patch = new BezierPatch(cp, 10);

std::vector<float> bsp={
    0.0f, 0.0f, 0.0f, // P0
    3.0f, 0.0f, 5.0f, // P2
    6.0f, 0.0f, 1.0f, // P3 
    8.0f, 0.0f, -12.0f, // P4
    11.0f, 0.0f, 5.0f, // P5
};

std::vector<float> knots = {0.0f,0.0f,0.0f,0.0f,0.5f,1.0f,1.0f,1.0f,1.0f};
Bspline *bspline = new Bspline(bsp, knots, 3, 1000);

// Camera
Camera *camera = new Camera(HMM_V3(0, 0, 0), 25.0f, 45.0f, 30.0f);

// Utils
Gizmo *gizmo;

static struct
{
    sg_pipeline pip_triangles;
    sg_pipeline pip_vertices;
    sg_pipeline pip_lines;
    sg_pipeline pip_curves;
    sg_pipeline pip_pts;
    sg_bindings bind;
    sg_pass_action pass_action;
} state;

void frame()
{
    sg_pass pass = {};
    pass.action = state.pass_action;
    pass.swapchain = sglue_swapchain();

    // Calculate MVP matix
    float fov_rad = 60.0f* (HMM_PI / 180.0f);
    HMM_Mat4 proj = HMM_Perspective_RH_NO(fov_rad, 640.0f / 480.0f, 0.01f, 100.0f);
     // HMM_Mat4 view = HMM_LookAt_RH(HMM_V3(0, 0.5f, 1.5f), HMM_V3(0, 0, 0), HMM_V3(0, 1, 0));
    HMM_Mat4 view = camera->get_view_matrix();
    HMM_Mat4 model = HMM_M4D(1.0f);
    HMM_Mat4 mvp = HMM_MulM4(proj, HMM_MulM4(view, model));

    HMM_Vec3 cam_pos = camera->calculate_position();

    sg_begin_pass(pass);

    // Draw axis indicator
    sg_apply_pipeline(state.pip_lines);
    gizmo->render_axis_indicator(mvp);

    // Display bspline
    sg_apply_pipeline(state.pip_curves);
    bspline->render_spline(mvp);

    // Draw bspline control polygon
    sg_apply_pipeline(state.pip_curves);
    bspline->render_control_points(mvp);

    // Draw bspline control points
    sg_apply_pipeline(state.pip_pts);
    bspline->render_control_points(mvp);

    /*
    // Draw bezier patch
    sg_apply_pipeline(state.pip_pts);
    sg_apply_pipeline(state.pip_triangles);
    sg_apply_uniforms(0, SG_RANGE_REF(mvp));
    patch->render();
    */

    // Draw Bspline
    //sg_apply_pipeline(state.pip_lines);
    //sg_apply_pipeline(state.pip_pts);    
    
    sg_end_pass();
    sg_commit();
}

void event(const sapp_event *ev)
{
    switch (ev->type)
    {
    case SAPP_EVENTTYPE_KEY_DOWN:
        if (ev->key_code == SAPP_KEYCODE_ESCAPE)
        {
            sapp_quit();
        }
        break;
    }

    camera->handle_events(ev);
}

void init()
{
    // setup environment and logging function
    sg_desc desc = {};
    desc.environment = sglue_environment();
    desc.logger.func = slog_func;
    sg_setup(desc);

    gizmo = new Gizmo();
    patch->generate_mesh();
    bspline->generate_bspline();

    // Create shader
    sg_shader shd = sg_make_shader(shd_shader_desc(sg_query_backend()));

    // Points pipeline
    sg_pipeline_desc pip_pts_desc = {};
    pip_pts_desc.shader = shd;
    pip_pts_desc.layout.attrs[ATTR_shd_pos].format = SG_VERTEXFORMAT_FLOAT3;
    pip_pts_desc.layout.attrs[ATTR_shd_color0].format = SG_VERTEXFORMAT_FLOAT4;
    pip_pts_desc.primitive_type = SG_PRIMITIVETYPE_POINTS;
    pip_pts_desc.label = "points_pipeline";
    state.pip_pts = sg_make_pipeline(pip_pts_desc);

    // Single lines pipeline
    sg_pipeline_desc pip_desc_lines = pip_pts_desc;
    pip_desc_lines.shader = shd;
    pip_desc_lines.primitive_type = SG_PRIMITIVETYPE_LINES;
    pip_desc_lines.label = "lines_pipeline";
    state.pip_lines = sg_make_pipeline(pip_desc_lines);

    // Cruve pipeline
    sg_pipeline_desc pip_desc_curves = pip_pts_desc;
    pip_desc_curves.shader = shd;
    pip_pts_desc.layout.attrs[ATTR_shd_color0].format = SG_VERTEXFORMAT_FLOAT4;
    pip_desc_curves.primitive_type = SG_PRIMITIVETYPE_LINE_STRIP;
    pip_desc_curves.label = "lines_pipeline";
    state.pip_curves = sg_make_pipeline(pip_desc_curves);

    // Mesh pipeline
    sg_pipeline_desc pip_desc_tri = {};
    pip_desc_tri.shader = shd;
    pip_desc_tri.layout.attrs[ATTR_shd_pos].format = SG_VERTEXFORMAT_FLOAT3;
    pip_desc_tri.layout.attrs[ATTR_shd_color0].format = SG_VERTEXFORMAT_FLOAT4;
    pip_desc_tri.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
    pip_desc_tri.index_type = SG_INDEXTYPE_UINT16;
    pip_desc_tri.label = "triangle_pipeline";
    state.pip_triangles = sg_make_pipeline(pip_desc_tri);

    // Vertex pipeline
    sg_pipeline_desc pip_desc_verts = pip_desc_tri;
    pip_desc_verts.primitive_type = SG_PRIMITIVETYPE_POINTS;
    pip_desc_verts.label = "points_pipeline";
    state.pip_vertices = sg_make_pipeline(pip_desc_verts);

    state.pass_action.colors[0].load_action = SG_LOADACTION_CLEAR;
    state.pass_action.colors[0].clear_value = {0.1f, 0.1f, 0.1f, 1.0f};
}

void cleanup()
{
    sg_shutdown();
}

sapp_desc sokol_main(int argc, char *argv[])
{
    sapp_desc desc = {};

    desc.width = 640;
    desc.height = 480;
    desc.init_cb = init;
    desc.frame_cb = frame;
    desc.cleanup_cb = cleanup;
    desc.event_cb = event;
    desc.window_title = "NURBS Viewer";

    return desc;
}
