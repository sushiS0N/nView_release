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
#include <cmath>

#include "shader.h"
#include "bezier_patch.h"
#include "camera.h"
#include "gizmo.h"
#include "nurbs.h"
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// + ADD PAN + live weight edit
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Resolution macros        
#define RESOLUTION_X 1280.0f 
#define RESOLUTION_Y 960.0f 
#define FOV 60.0f * (HMM_PI / 180.0f)

// GEOMETRY
// NURBS - surface

std::vector<float> srf_cp = {
    // Row 0 (j=0)
    0.0f, 0.0f, 0.0f,    2.0f, 1.0f, 0.0f,    4.0f, 2.0f, 0.0f,    6.0f, 0.0f, 0.0f,
    // Row 1 (j=1)
    0.0f, 5.0f, 2.0f,    2.0f, 1.0f, 2.0f,    4.0f, 0.0f, 2.0f,    6.0f, 0.0f, 2.0f,
    // Row 2 (j=2)
    0.0f, 5.0f, 4.0f,    2.0f, 1.0f, 4.0f,    4.0f, 0.0f, 4.0f,    6.0f, 0.0f, 4.0f,
    // Row 3 (j=3)
    0.0f, 0.0f, 6.0f,    2.0f, 1.0f, 6.0f,    4.0f, 2.0f, 6.0f,    6.0f, 0.0f, 6.0f,
};

std::vector<float> u_knots = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
std::vector<float> v_knots = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f};

std::vector<float> srf_weights = {
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f
};

NURBS_surface* surface;

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
    2.0f, 0.0f, 2.0f,  // P4
    3.0f, 0.0f, 4.0f  // P5
};
NURBS_spline *bspline;

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
    sg_pipeline pip_idx_lines;
    sg_pipeline pip_curves;
    sg_pipeline pip_pts;
    sg_bindings bind;
    sg_pass_action pass_action;

    // View matrices
    HMM_Mat4 current_mvp;
    HMM_Mat4 current_proj;
    HMM_Mat4 current_view;

    // Buffer update flag
    bool buf_update_flag;
} state;

// Interaction enum
enum InteractionMode
{
    MODE_VIEW,
    MODE_EDIT_CURVE,
    MODE_CREATE_CURVE,
    MODE_EDIT_SURFACE
};

// Interaction struct
static struct
{
    bool dragging = false;
    float mouse_x, mouse_y, ndc_x, ndc_y;
    HMM_Vec3 mouse_ray;

    int selected_cp_index = -1;

    // Draw curve
    bool add_pts = false;

    InteractionMode mode = MODE_EDIT_SURFACE;

} interaction;

int closest_cp(std::vector<float> &points, HMM_Mat4 mvp)
{
    int pt_idx = 0;
    float min_dist = sapp_width() * 2;
    int closest_idx = -1;

    for (int i = 0; i < points.size()/3; i++)
    {
        pt_idx = i *3;
        HMM_Vec3 world_pt = HMM_V3(points[pt_idx], points[pt_idx + 1], points[pt_idx + 2]);
        HMM_Vec4 world_pos = HMM_V4(world_pt.X, world_pt.Y, world_pt.Z, 1.0f);
        HMM_Vec4 clip_pos = HMM_MulM4V4(mvp, world_pos);

        // Project to 3D
        HMM_Vec3 ndc = HMM_V3(clip_pos.X / clip_pos.W, clip_pos.Y / clip_pos.W, clip_pos.Z / clip_pos.W);

        // NDC to screen
        float screen_x = (ndc.X + 1.0f) * sapp_width() / 2.0f;
        float screen_y = (1.0f - ndc.Y) * sapp_height() / 2.0f;

        float pixel_dist = std::sqrt((screen_x - interaction.mouse_x)*(screen_x - interaction.mouse_x) + (screen_y-interaction.mouse_y)*(screen_y-interaction.mouse_y));

        if (pixel_dist < min_dist && pixel_dist < 10)
        {
            min_dist = pixel_dist;
            closest_idx = i;
        }
    }
    return closest_idx;
}

float dist_pt_ln(HMM_Vec3 point, HMM_Vec3 origin, HMM_Vec3 dir)
{
    printf("  Testing CP at (%.2f, %.2f, %.2f) against ray from (%.2f, %.2f, %.2f) dir (%.2f, %.2f, %.2f)\n",
           point.X, point.Y, point.Z, origin.X, origin.Y, origin.Z, dir.X, dir.Y, dir.Z);
        
    HMM_Vec3 a = HMM_Cross(HMM_SubV3(point, origin), dir);
    float dist = std::sqrt(HMM_Dot(a, a) / HMM_Dot(dir, dir));
    printf("    Distance: %.2f\n", dist);
    return dist;
}

HMM_Vec3 line_plane_int(HMM_Vec3 cam_pos, HMM_Vec3 dir)
{
    HMM_Vec3 int_pt = HMM_V3(0.0f, 0.0f, 0.0f);
    HMM_Vec3 normal = HMM_V3(0.0f, 1.0f, 0.0f); // XZ plane

    float denom = HMM_DotV3(normal, dir);
    if (std::abs(denom) > 1e-6)
    {
        float t = -cam_pos.Y / denom; // plane_origin.y = 0
        if (t >= 0)
        {
            int_pt = HMM_AddV3(cam_pos, HMM_MulV3F(dir, t));
        }
    }
    return int_pt;
}

HMM_Vec3 unproject_point(float ndc_x, float ndc_y, float ndc_z, HMM_Mat4 proj, HMM_Mat4 view)
{
    HMM_Vec4 clip_pos = HMM_V4(ndc_x,ndc_y,ndc_z, 1.0f);

    // Inverse perspective projection
    HMM_Mat4 inv_proj = HMM_InvGeneralM4(proj);
    HMM_Vec4 view_pos = HMM_MulM4V4(inv_proj, clip_pos);

    // Perspective divide
    view_pos = HMM_DivV4F(view_pos, view_pos.W);

    // World space
    HMM_Mat4 inv_view = HMM_InvGeneralM4(view);
    HMM_Vec4 world_pos = HMM_MulM4V4(inv_view, view_pos);

    return HMM_V3(world_pos.X,world_pos.Y,world_pos.Z);
}

HMM_Vec3 screen_to_ray(float ndc_x, float ndc_y, HMM_Mat4 proj, HMM_Mat4 view)
{
    HMM_Vec3 near_pt = unproject_point(ndc_x,ndc_y, -1.0f, proj, view);
    HMM_Vec3 far_pt = unproject_point(ndc_x,ndc_y, 1.0f, proj, view);

    HMM_Vec3 ray_dir = HMM_SubV3(far_pt, near_pt);

    return HMM_NormV3(ray_dir);
}

void move_crv_pt(int cp_idx, HMM_Vec3 ray_dir, HMM_Vec3 camera_pos)
{
    // Intersect mouse_ray with XZ plane
    HMM_Vec3 pos_XZ = line_plane_int(camera_pos, ray_dir);
    bspline->control_points[cp_idx*3] = pos_XZ.X;
    bspline->control_points[cp_idx*3+1] = pos_XZ.Y;
    bspline->control_points[cp_idx*3+2] = pos_XZ.Z;
    
    // Update control point and curve
    bspline->update_cp(cp_idx, pos_XZ);
}

void move_srf_pt(int cp_idx, HMM_Vec3 ray_dir, HMM_Vec3 camera_pos)
{
    // Intersect mouse_ray with XZ plane
    HMM_Vec3 pos_XZ = line_plane_int(camera_pos, ray_dir);
    surface->control_points[cp_idx*3] = pos_XZ.X;
    surface->control_points[cp_idx*3+1] = pos_XZ.Y;
    //bspline->control_points[cp_idx*3+2] = pos_XZ.Z; // no need for Z yet
    
    // Update control point and curve
    surface->update_srf_cp(cp_idx, pos_XZ);
}

// Loggin functions
void update_mouse_pos(const sapp_event *ev)
{
    // Screen coordinates
    float mouse_x = ev->mouse_x;
    float mouse_y = ev->mouse_y;

    // Convert to NDC
    float ndc_x = (2.0f * mouse_x / sapp_widthf()) - 1.0f;
    float ndc_y = 1.0f - (2.0f * mouse_y / sapp_heightf());

    // Calcualte ray
    interaction.mouse_ray = screen_to_ray(ndc_x, ndc_y, state.current_proj, state.current_view);

    // Stoe mouse screen and ndc coordinates
    interaction.mouse_x = mouse_x;
    interaction.mouse_y = mouse_y;   
    interaction.ndc_x = ndc_x;   
    interaction.ndc_y = ndc_y; 
}

static void print_status_text(float disp_w, float disp_h)
{
    sdtx_canvas(disp_w *.5f , disp_h *.5f);
    sdtx_origin(0.5f, 0.5f);
    sdtx_color3f(0.0f,0.7f,0.0f);

    sdtx_printf("Window x: %.2f, y: %.2f\n", interaction.mouse_x, interaction.mouse_y);
    sdtx_printf("NDC x: %.2f, y: %.2f\n", interaction.ndc_x, interaction.ndc_y);

    // Interaction mode
    switch (interaction.mode)
    {
    case MODE_VIEW:
        sdtx_printf("Mode: view \n");
        break;
    case MODE_EDIT_CURVE:
        sdtx_printf("Mode: edit curve \n");
        sdtx_printf("Toggle:\n");
        sdtx_printf("  Display CP influence - i\n");
        sdtx_printf("  Display knots - k\n");
        sdtx_printf("  Add points - c\n");
        break;
    case MODE_EDIT_SURFACE:
        sdtx_printf("Mode: edit surface \n");
        break;
    }
};

void frame()
{
    sg_pass pass = {};
    pass.action = state.pass_action;
    pass.swapchain = sglue_swapchain();

    // Calculate MVP matix
    HMM_Mat4 proj = HMM_Perspective_RH_NO(FOV, sapp_width() / sapp_height(), 0.01f, 100.0f);
    HMM_Mat4 view = camera->get_view_matrix();
    HMM_Mat4 model = HMM_M4D(1.0f);
    HMM_Mat4 mvp = HMM_MulM4(proj, HMM_MulM4(view, model));

    // Save state
    state.current_mvp = mvp;
    state.current_proj = proj;
    state.current_view = view;

    // HMM_Vec3 cam_pos = camera->calculate_position();
    //     printf("Camera pos: %.2f, %.2f, %.2f\n", cam_pos.X, cam_pos.Y, cam_pos.Z);

    // Debugger text
    print_status_text(sapp_widthf(), sapp_heightf());

    if(state.buf_update_flag)
    {
        bspline->update_buffer();
        surface->update_buffer();
        state.buf_update_flag = false;
    }

    sg_begin_pass(pass);

    // Draw axis indicator
    sg_apply_pipeline(state.pip_lines);
    gizmo->render_axis_indicator(mvp);

    switch (interaction.mode)
    {
    case MODE_EDIT_CURVE:
        // Display bspline
        sg_apply_pipeline(state.pip_curves);
        bspline->render_spline(mvp);

        // Draw control polygon
        sg_apply_pipeline(state.pip_curves);
        bspline->render_control_points(mvp);

        // Draw bspline control points
        sg_apply_pipeline(state.pip_pts);
        bspline->render_control_points(mvp);

        // Draw bspline markers
        if (bspline->show_knots)
        {
            sg_apply_pipeline(state.pip_pts);
            bspline->render_knots(mvp);
        }
        break;
    case MODE_EDIT_SURFACE:
        // Draw surface
        sg_apply_pipeline(state.pip_triangles);
        surface->render_surface(mvp);

        // Draw surface's control points
        sg_apply_pipeline(state.pip_pts);
        surface->render_control_points(mvp);

        // Draw surface's control polygon
        sg_apply_pipeline(state.pip_idx_lines);
        surface->render_control_polygon(mvp);
        break;
    }

    sdtx_draw();
    sg_end_pass();
    sg_commit();
}

void event(const sapp_event *ev)
{
    // Switch modes
    if (ev->type == SAPP_EVENTTYPE_KEY_DOWN)
    {
        if (ev->key_code == SAPP_KEYCODE_1)
        {
            interaction.mode = MODE_VIEW;
        }
        else if (ev->key_code == SAPP_KEYCODE_2)
        {
            interaction.mode = MODE_EDIT_CURVE;
        }
        else if (ev->key_code == SAPP_KEYCODE_3)
        {
            interaction.mode = MODE_EDIT_SURFACE;
        }
        else if (ev->key_code == SAPP_KEYCODE_ESCAPE)
        {
            sapp_quit();
        }
    }

    // General interaction
    switch (interaction.mode)
    {
    case MODE_VIEW:
        camera->handle_events(ev);
        break;
    case MODE_EDIT_CURVE:
        if (ev->type == SAPP_EVENTTYPE_KEY_DOWN)
        {
            if (ev->key_code == SAPP_KEYCODE_K)
            {
                if (bspline->show_knots)
                {
                    bspline->show_knots = false;
                }
                else
                {
                    state.buf_update_flag = true;
                    bspline->show_knots = true;
                }
            }
            else if (ev->key_code == SAPP_KEYCODE_I)
            {
                if (bspline->show_influence)
                    bspline->show_influence = false;
                else
                    bspline->show_influence = true;
            }
            else if (ev->key_code == SAPP_KEYCODE_C)
            {
                if (interaction.add_pts)
                    interaction.add_pts = false;
                else
                    interaction.add_pts = true;
            }
        }
        else if (ev->type == SAPP_EVENTTYPE_MOUSE_DOWN && ev->mouse_button == SAPP_MOUSEBUTTON_LEFT)
        {
            if (interaction.add_pts)
            {
                update_mouse_pos(ev);
                HMM_Vec3 new_pt = line_plane_int(camera->calculate_position(), interaction.mouse_ray);
                bspline->add_cp(new_pt);
                state.buf_update_flag = true;
            }
            else
            {
                interaction.dragging = true;

                update_mouse_pos(ev);
                interaction.selected_cp_index = closest_cp(bspline->control_points, state.current_mvp);
            }
        }
        else if (ev->type == SAPP_EVENTTYPE_MOUSE_UP && ev->mouse_button == SAPP_MOUSEBUTTON_LEFT)
        {
            interaction.dragging = false;
        }
        else if (ev->type == SAPP_EVENTTYPE_MOUSE_MOVE)
        {
            if (interaction.dragging)
            {
                state.buf_update_flag = true;
                update_mouse_pos(ev);
                if (interaction.selected_cp_index != -1)
                {
                    move_crv_pt(interaction.selected_cp_index, interaction.mouse_ray, camera->calculate_position());
                }
            }
        }
        camera->handle_events(ev);
        break;
    case MODE_EDIT_SURFACE:
        if (ev->type == SAPP_EVENTTYPE_MOUSE_DOWN && ev->mouse_button == SAPP_MOUSEBUTTON_LEFT)
        {
            interaction.dragging = true;

            update_mouse_pos(ev);
            interaction.selected_cp_index = closest_cp(surface->control_points, state.current_mvp);
        }
        else if (ev->type == SAPP_EVENTTYPE_MOUSE_UP && ev->mouse_button == SAPP_MOUSEBUTTON_LEFT)
        {
            interaction.dragging = false;
        }
        else if (ev->type == SAPP_EVENTTYPE_MOUSE_MOVE)
        {
            if (interaction.dragging)
            {
                state.buf_update_flag = true;
                update_mouse_pos(ev);
                if (interaction.selected_cp_index != -1)
                {
                    move_srf_pt(interaction.selected_cp_index, interaction.mouse_ray, camera->calculate_position());
                }
            }
        }
        camera->handle_events(ev);
        break;
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
    sdtx_desc.fonts[0] = sdtx_font_oric();
    sdtx_desc.logger.func = slog_func;
    sdtx_setup(sdtx_desc);  

    
    // Initialize geometry
    gizmo = new Gizmo();

    // NURBS spline
    bspline = new NURBS_spline(bsp, 3, 1000);
    bspline->generate(0);
    state.buf_update_flag = true;

    // NURBS surface
    surface = new NURBS_surface(srf_cp, u_knots, v_knots, 3, 4, 4, 20, srf_weights);
    surface->generate_mesh();
    state.buf_update_flag = true;

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

    // Indexed lines pipeline
    sg_pipeline_desc idx_ln_desc = pip_desc_lines;
    idx_ln_desc.index_type = SG_INDEXTYPE_UINT16;
    idx_ln_desc.label = "indexed_lines_pipeline";
    state.pip_idx_lines = sg_make_pipeline(idx_ln_desc);

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
