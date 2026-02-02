#define HANDMADE_MATH_IMPLEMENTATION
#include "HandmadeMath.h"

#define SOKOL_IMPL
#define SOKOL_GLCORE

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_log.h"
#include "sokol_debugtext.h"

#include "stdio.h"
#include <vector>

#include "shader.h"
#include "bezier_patch.h"
#include "camera.h"
#include "gizmo.h"
#include "nurbs.h"

// Resolution macros
#define RESOLUTION_X 640.0f
#define RESOLUTION_Y 480.0f 

// GEOMETRY
// NURBS - surface
std::vector<float> srf_cp = {
    // Row 0 (j=0)
    0.0f, 0.0f, 0.0f,    2.0f, 0.0f, 1.0f,    4.0f, 0.0f, 1.0f,    6.0f, 0.0f, 0.0f,
    // Row 1 (j=1)
    0.0f, 2.0f, 1.0f,    2.0f, 2.0f, 3.0f,    4.0f, 2.0f, 3.0f,    6.0f, 2.0f, 1.0f,
    // Row 2 (j=2)
    0.0f, 4.0f, 1.0f,    2.0f, 4.0f, 3.0f,    4.0f, 4.0f, 3.0f,    6.0f, 4.0f, 1.0f,
    // Row 3 (j=3)
    0.0f, 6.0f, 0.0f,    2.0f, 6.0f, 1.0f,    4.0f, 6.0f, 1.0f,    6.0f, 6.0f, 0.0f
};

std::vector<float> u_knots = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
std::vector<float> v_knots = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f};

std::vector<float> srf_weights = {
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f
};

NURBS_surface* surface = new NURBS_surface(
    srf_cp,
    u_knots,
    v_knots,
    3,        // degree
    4,        // u_num_pts
    4,        // v_num_pts
    20,       // resolution
    srf_weights
);

// Bezier patch
float bez_cp[48] = {
    // Row 0 (X = 0)
    0.0f, 5.0f, 0.0f,    0.0f, 0.0f, 2.0f,    0.0f, 0.0f, 4.0f,    0.0f, 0.0f, 6.0f,
    // Row 1 (X = 2)  
    2.0f, 0.0f, 0.0f,    2.0f, 0.0f, 2.0f,    2.0f, 0.0f, 4.0f,    2.0f, 0.0f, 6.0f,
    // Row 2 (X = 4)
    4.0f, 0.0f, 0.0f,    4.0f, 0.0f, 2.0f,    4.0f, 5.0f, 4.0f,    4.0f, 0.0f, 6.0f,
    // Row 3 (X = 6)
    6.0f, 5.0f, 0.0f,    6.0f, 0.0f, 2.0f,    6.0f, 0.0f, 4.0f,    6.0f, 0.0f, 6.0f
};
BezierPatch *bez_patch = new BezierPatch(bez_cp, 10);

// NURBS - spline
std::vector<float> bsp={
    0.0f, 0.0f, 0.0f, // P0
    6.0f, 0.0f, 0.0f, // P1 
    8.0f, 0.0f, 6.0f, // P2
    0.0f, 0.0f, 4.0f, // P3
    2.0f, 0.0f, 2.0f  // P4
};
std::vector<float> bsp_knots = {0.0f,0.0f,0.0f,0.0f,0.5f,1.0f,1.0f,1.0f,1.0f};
std::vector<float> bsp_weights = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

NURBS_spline *bspline = new NURBS_spline(bsp, bsp_knots, 3, 1000, bsp_weights);

// Camera
Camera *camera = new Camera(HMM_V3(0, 0, 0), 25.0f, 45.0f, 30.0f);

// Utils
Gizmo *gizmo;

// Sokol struct
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

// Mouse interaction struct and utility functions
static struct
{
    bool dragging = false;
    int selected_cp_index = -1;
    HMM_Vec3 drag_plane_normal;
    float mouse_x, mouse_y, ndc_x, ndc_y;
} interaction_struct;

HMM_Vec3 screen_to_ray(float mouse_x, float mouse_y);
int find_closest_cp(HMM_Vec3 ray_origin, HMM_Vec3 ray_dir);
void update_cp(int index, HMM_Vec3 new_pos);


// Loggin functions
void print_mouse_pos(const sapp_event *ev)
{
    // Screen coordinates
    float mouse_x = ev->mouse_x;
    float mouse_y = ev->mouse_y;

    // Convert to NDC
    float ndc_x = (2.0f * mouse_x / sapp_widthf()) - 1.0f;
    float ndc_y = 1.0f - (2.0f * mouse_y / sapp_heightf());

    interaction_struct.mouse_x = mouse_x;
    interaction_struct.mouse_y = mouse_x;   
    interaction_struct.ndc_x = ndc_x;   
    interaction_struct.ndc_y = ndc_x;   
}

static void print_status_text(float disp_w, float disp_h)
{
    sdtx_canvas(disp_w *0.6f, disp_h * 0.6f);
    sdtx_origin(0.5f, 0.5f);
    sdtx_color3f(0.0f,0.7f,0.0f);
    sdtx_printf("Last mouse coordinate:\n");
};

void frame()
{
    sg_pass pass = {};
    pass.action = state.pass_action;
    pass.swapchain = sglue_swapchain();

    // Debugger text
    const float w = sapp_widthf();
    const float h = sapp_heightf();
    print_status_text(sapp_widthf(), sapp_heightf());
    sdtx_printf("Mouse x: %.2f, Mouse: y: %.2f\n", interaction_struct.mouse_x, interaction_struct.mouse_y);
    sdtx_printf("NDC_x: %.2f, NDC_y: y: %.2f", interaction_struct.ndc_x, interaction_struct.ndc_y);

    // Calculate MVP matix
    float fov_rad = 60.0f* (HMM_PI / 180.0f);
    HMM_Mat4 proj = HMM_Perspective_RH_NO(fov_rad, RESOLUTION_X / RESOLUTION_Y, 0.01f, 100.0f);
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

    // sg_apply_pipeline(state.pip_triangles);
    // surface->render_surface(mvp);
    
    sdtx_draw();
    sg_end_pass();
    sg_commit();
}

void event(const sapp_event *ev)
{
    // Window actions
    switch (ev->type)
    {
    case SAPP_EVENTTYPE_KEY_DOWN:
        if (ev->key_code == SAPP_KEYCODE_ESCAPE)
        {
            sapp_quit();
        } break;
    }

    // Camera handler in camera.cpp
    camera->handle_events(ev);

    // Mouse picking
    switch(ev->type)
    {
        case SAPP_EVENTTYPE_MOUSE_DOWN:
        if(ev->mouse_button == SAPP_MOUSEBUTTON_LEFT)
        {
            print_mouse_pos(ev);
        } break;

    }
}

void init()
{
    // setup environment and logging function
    sg_desc desc = {};
    desc.environment = sglue_environment();
    desc.logger.func = slog_func;
    sg_setup(desc);

    // Setup sokol-debugtext
    sdtx_desc_t sdtx_desc = {};
    sdtx_desc.fonts[0] = sdtx_font_z1013();
    sdtx_desc.logger.func = slog_func;
    sdtx_setup(sdtx_desc);  

    // Initialize geometry
    gizmo = new Gizmo();
    //patch->generate_mesh();
    bspline->generate_bspline();
    //surface->generate_mesh();

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

    desc.width = RESOLUTION_X;
    desc.height = RESOLUTION_Y;
    desc.init_cb = init;
    desc.frame_cb = frame;
    desc.cleanup_cb = cleanup;
    desc.event_cb = event;
    desc.window_title = "NURBS Viewer";

    return desc;
}
