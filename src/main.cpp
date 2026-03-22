#define HANDMADE_MATH_IMPLEMENTATION
#include "HandmadeMath.h"

#include "imgui.h"

#define SOKOL_IMPL
#define SOKOL_GLCORE
#define SOKOL_FETCH_IMPL

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_log.h"
#include "sokol_debugtext.h"
#include "sokol_fetch.h"
#include "sokol_imgui.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "stdio.h"
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include <optional>

#include "shader.h"
#include "camera.h"
#include "gizmo.h"
#include "nurbs.h"
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// + ADD live weight edit
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Resolution macros        
#define RESOLUTION_X 2560.0f 
#define RESOLUTION_Y 1440.0f 
#define FOV 60.0f * (HMM_PI / 180.0f)
#define MAX_FILE_SIZE 1024*1024


// Image loading
static void response_callback(const sfetch_response_t*);
static uint8_t buffer[MAX_FILE_SIZE];

// GEOMETRY PARAMETERS
// NURBS - surface 
std::vector<float> srf_cp = {
    // Row 0 (j=0)
    0.0f, 0.0f, 0.0f,    2.0f, 1.0f, 0.0f,    4.0f, 2.0f, 0.0f,    8.0f, 0.0f, 0.0f,
    // Row 1 (j=1)
    -2.0f, 2.5f, 1.0f,    2.0f, 1.0f, 1.0f,    4.0f, 0.0f, 1.0f,    8.0f, 0.0f, 1.0f,
    // Row 2 (j=2)
    -2.0f, 2.5f, 2.0f,    2.0f, 1.0f, 2.0f,    4.0f, 0.0f, 2.0f,    8.0f, 0.0f, 2.0f,
    // Row 3 (j=3)
    0.0f, 0.0f, 3.0f,    2.0f, 1.0f, 3.0f,    4.0f, 2.0f, 3.0f,    8.0f, 0.0f, 3.0f,
};

std::vector<float> u_knots = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
std::vector<float> v_knots = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f};

std::vector<float> srf_weights = {
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f
};

// NURBS - Curve
std::vector<float> bsp={
    0.0f, 0.0f, 7.0f, // P0
    3.0f, 0.0f, 4.0f, // P1 
    6.0f, 0.0f, 12.0f, // P2
    8.0f, 0.0f, 7.0f, // P3
};

// Scene objects
// Camera
auto camera = std::make_unique<Camera>(HMM_V3(0, 0, 0), 30.0f, 45.0f, 30.0f, RESOLUTION_X, RESOLUTION_Y);
// Gizmo
std::unique_ptr<Gizmo> world_axis;
// Gumball
std::unique_ptr<Gizmo> gumball;

// Geometry objects
// Create objects
std::unique_ptr<NURBS_spline> bspline;
std::unique_ptr<NURBS_surface> surface;

// Sokol struct
static struct
{
    sg_pipeline pip_triangles;
    sg_pipeline pip_matcap;
    sg_pipeline pip_vertices;
    sg_pipeline pip_lines;
    sg_pipeline pip_idx_lines;
    sg_pipeline pip_curves;
    sg_pipeline pip_pts;
    sg_bindings bind;
    sg_pass_action pass_action;

    // View matrices
    HMM_Mat4 mvp;
    HMM_Mat4 proj;
    HMM_Mat4 view;

    // Buffer update flag
    bool buf_update_flag;
    bool matcap_loaded = false;

    // Image
    uint8_t file_buffer[256*1024];
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
    InteractionMode mode = MODE_VIEW;

    bool dragging = false;
    float mouse_x, mouse_y, ndc_x, ndc_y, m_norm_x, m_norm_y;
    HMM_Vec3 mouse_ray;

    int selected_cp_index = -1;
    HMM_Vec3 selected_cp_pos = HMM_V3(0.0f, 0.0f, 0.0f);

    // Curve
    bool insert_knot = false;
    float knot_value = 0.0f;

    bool show_bezier_aabb = false;
    bool add_pts = false;

    // Gumball
    float gumball_size = 0.04f;
    bool gumball_mode = false;
} interaction;

static ImFont* ui_font = nullptr;

// Utils
int closest_cp_idx(const std::vector<float>& points, HMM_Mat4 mvp)
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

        if (pixel_dist < min_dist && pixel_dist < 10.0f)
        {
            min_dist = pixel_dist;
            closest_idx = i;
        }
    }
    return closest_idx;
}

HMM_Vec3 closest_cp_pos(const std::vector<float> &points, int idx)
{
    HMM_Vec3 pos = HMM_V3(points[idx*3],points[idx*3+1],points[idx*3+2]);
    return pos;
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

void update_mouse_pos(const sapp_event *ev)
{
    // Screen coordinates
    float mouse_x = ev->mouse_x;
    float mouse_y = ev->mouse_y;

    // Convert to NDC
    float ndc_x = (2.0f * mouse_x / sapp_widthf()) - 1.0f;
    float ndc_y = 1.0f - (2.0f * mouse_y / sapp_heightf());

    // Calcualte ray
    interaction.mouse_ray = screen_to_ray(ndc_x, ndc_y, state.proj, state.view);

    // Store mouse screen and ndc coordinates
    interaction.mouse_x = mouse_x;
    interaction.mouse_y = mouse_y;
    interaction.ndc_x = ndc_x;   
    interaction.ndc_y = ndc_y; 

    // Normalized x and y
    interaction.m_norm_x = std::clamp(mouse_x / sapp_widthf(), 0.0f, 1.0f);
    interaction.m_norm_y = std::clamp(mouse_y / sapp_heightf(), 0.0f, 1.0f);
}

// Loggin functions
static void print_status_text(float disp_w, float disp_h)
{
    sdtx_canvas(disp_w *.5f , disp_h *.5f);
    sdtx_origin(0.5f, 0.5f);
    sdtx_color3f(0.0f,0.7f,0.0f);

    // sdtx_printf("Window x: %.2f, y: %.2f\n", interaction.mouse_x, interaction.mouse_y);
    // sdtx_printf("NDC x: %.2f, y: %.2f\n", interaction.ndc_x, interaction.ndc_y);

    // Interaction mode
    switch (interaction.mode)
    {
    case MODE_VIEW:
        sdtx_printf("Mode: view \n");
        break;
    case MODE_EDIT_CURVE:
        sdtx_printf("Mode: edit curve \n");
        sdtx_printf("Toggles:\n");
        sdtx_printf(" i - display CP influence: %s\n", bspline->show_influence ? "on" : "off");
        sdtx_printf(" k - display knots: %s\n",  bspline->show_knots ? "on" : "off");
        sdtx_printf(" c - add points:%s\n", interaction.add_pts ? "on" : "off");
        break;
    case MODE_EDIT_SURFACE:
        sdtx_printf("Mode: edit surface \n");
        sdtx_printf("RMB - orbit \n");
        sdtx_printf("LMB - drag points on XZ plane \n");
        sdtx_printf("F - reset camera \n");
        break;
    }

    sdtx_canvas(disp_w *.5f , disp_h *.5f);
    sdtx_origin(disp_w *.5f , disp_h *.5f);
    sdtx_printf("Mode: view \n");
}

// UI
static void render_ui()
{
    ImGui::SetNextWindowPos(ImVec2(10,10), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(270,400), ImGuiCond_FirstUseEver);
    ImGui::PushFont(ui_font);
    ImGui::Begin("Menu", nullptr, ImGuiWindowFlags_NoMove);
    ImGui::Text("Application average %.3f ms/frame \n (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    // Mode buttons
    ImVec2 button_size(ImGui::GetContentRegionAvail().x / 3.0f - 4.0f,0);
    if(ImGui::Button("View", button_size)) interaction.mode = MODE_VIEW;
    ImGui::SameLine();
    if(ImGui::Button("Curve", button_size)) interaction.mode = MODE_EDIT_CURVE;
    ImGui::SameLine();
    if(ImGui::Button("Surface", button_size)) interaction.mode = MODE_EDIT_SURFACE;

    ImGui::Separator();
    switch (interaction.mode)
    { 
    case MODE_VIEW:
        ImGui::Text("RMB - orbit\n");
        ImGui::Text("Scroll - zoom\n");
        ImGui::Text("Shift + RMB - pan\n");

        break;
    case MODE_EDIT_CURVE:
        if(ImGui::Checkbox("Gumball - G", &interaction.gumball_mode))
        if(ImGui::Checkbox("Display Knots - K", &bspline->show_knots))
        {
            state.buf_update_flag = true;
        }
        if(ImGui::Checkbox("Display CP Influence - I", &bspline->show_influence))
        {
            state.buf_update_flag = true;
        }
        ImGui::Checkbox("Add points - C", &interaction.add_pts);
        ImGui::Checkbox("Show bezier AABB - B", &interaction.show_bezier_aabb);
        ImGui::Checkbox("Show knots - K", &bspline->show_knots);
        ImGui::Checkbox("Insert knot - P", &interaction.insert_knot);
        if(ImGui::Button("Reset"))
        {
            bspline = std::make_unique<NURBS_spline>(bsp, 3, 1000);
            bspline->generate(0);
            state.buf_update_flag = true;

            gumball->reset();

            interaction.gumball_mode = false;
            interaction.insert_knot = false;
        }

        // Insert knots
        ImGui::Separator();
        ImGui::TextWrapped("%s", bspline->print_knots().c_str());
        if(ImGui::Button("Add knot"))
        {
            bspline->insert_knot(interaction.knot_value,1);
            state.buf_update_flag = true;
        }        
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::SliderFloat("[0.1,0.9]", &interaction.knot_value, 0.1f, 0.9f, "%.2f", 0);

        // Extract bezier
        if(ImGui::Button("Convert to bezier"))
        {
            bspline->convert_to_bezier();
            state.buf_update_flag = true;
        }   
        break;
    case MODE_EDIT_SURFACE:
        ImGui::Text("RMB + scroll - orbit and zoom");
        ImGui::Text("LMB - drag points");
        ImGui::Checkbox("Gumball - g", &interaction.gumball_mode);
        if(ImGui::Button("Reset"))
        {
            surface = std::make_unique<NURBS_surface>(srf_cp, u_knots, v_knots, 3, 4, 4, 20, srf_weights);
            surface->generate_mesh();

            if (state.matcap_loaded)
            {
                surface->mesh_bind.views[VIEW_tex] = state.bind.views[VIEW_tex];
                surface->mesh_bind.samplers[SMP_smp] = state.bind.samplers[SMP_smp];
            }
            gumball->reset();
            interaction.gumball_mode = false;

            state.buf_update_flag = true;
        }
        break;
    }    
    ImGui::PopFont();
    ImGui::End();
}

void frame()
{
    simgui_frame_desc_t frame_desc = {sapp_width(), sapp_height(), sapp_frame_duration(), 1.0};
    simgui_new_frame(frame_desc);
    render_ui();

    sg_pass pass = {};
    pass.action = state.pass_action;
    pass.swapchain = sglue_swapchain();

    sfetch_dowork();

    // Calculate MVP matix
    HMM_Mat4 proj = HMM_Perspective_RH_NO(FOV, sapp_widthf() / sapp_heightf(), 0.01f, 100.0f);
    HMM_Mat4 view = camera->get_view_matrix();
    HMM_Mat4 model = HMM_M4D(1.0f);
    HMM_Mat4 mvp = HMM_MulM4(proj, HMM_MulM4(view, model));

    // Save state
    state.mvp = mvp;
    state.proj = proj;
    state.view = view;

    // Debugger text
    //print_status_text(sapp_widthf(), sapp_heightf());

    if(state.buf_update_flag)
    {
        bspline->update_buffer();
        surface->update_buffer();
        state.buf_update_flag = false;
    }

    sg_begin_pass(pass);
    simgui_render();

    // Draw axis indicator
    sg_apply_pipeline(state.pip_triangles);
    world_axis->render_gizmo(mvp);

    switch (interaction.mode)
    {
    case MODE_VIEW:
        // Draw surface
        sg_apply_pipeline(state.matcap_loaded ? state.pip_matcap : state.pip_triangles);
        surface->render_surface(mvp, view, state.matcap_loaded);

        // Display bspline
        sg_apply_pipeline(state.pip_curves);
        bspline->render_spline(mvp);
        break;
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

        // Draw gumball
        if(gumball->show)
        {
            sg_apply_pipeline(state.pip_triangles);
            gumball->render_gizmo(gumball->set_gumball_mvp(camera->calculate_position(), state.view, state.proj, interaction.gumball_size));
        }
        // Draw bspline markers
        if (bspline->show_knots)
        {
            sg_apply_pipeline(state.pip_pts);
            bspline->render_knots(mvp);
        }

        if (bspline->show_pt_on_crv)
        {
            sg_apply_pipeline(state.pip_pts);
            bspline->render_pt_on_crv(mvp);
        }
        if (interaction.show_bezier_aabb)
        {
            bspline->show_aabb = true;
            sg_apply_pipeline(state.pip_lines);
            bspline->render_aabb(mvp);
        }
        break;

    case MODE_EDIT_SURFACE:
        // Draw surface
        sg_apply_pipeline(state.pip_matcap);
        surface->render_surface(mvp, view, state.matcap_loaded);

        // Draw surface's control points
        sg_apply_pipeline(state.pip_pts);
        surface->render_control_points(mvp);

        // Draw surface's control polygon
        sg_apply_pipeline(state.pip_idx_lines);
        surface->render_control_polygon(mvp);

        // Draw gumball
        if(gumball->show)
        {
            sg_apply_pipeline(state.pip_triangles);
            gumball->render_gizmo(gumball->set_gumball_mvp(camera->calculate_position(), state.view, state.proj, interaction.gumball_size));
        }
        break;
    }

    sdtx_draw();
    sg_end_pass();
    sg_commit();
    // 
}

void event(const sapp_event *ev)
{
    simgui_handle_event(ev);
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
            interaction.gumball_mode = false;
            gumball->reset();
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
                    state.buf_update_flag = true;
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
                {
                    bspline->show_influence = false;
                    bspline->generate();
                    state.buf_update_flag = true;
                }
                else
                {
                    bspline->show_influence = true;
                    bspline->generate();
                    state.buf_update_flag = true;
                }
            }
            else if (ev->key_code == SAPP_KEYCODE_C)
            {
                if (interaction.add_pts)
                    interaction.add_pts = false;
                else
                    interaction.add_pts = true;
            }
            else if (ev->key_code == SAPP_KEYCODE_G)
            {
                if (interaction.gumball_mode)
                {
                    interaction.gumball_mode = false;
                    gumball->reset();
                }
                else
                {
                    interaction.gumball_mode = true;
                    interaction.add_pts = false;
                }                    
            }
            else if (ev->key_code == SAPP_KEYCODE_P)
            {
                if (interaction.insert_knot)
                {
                    interaction.insert_knot = false;
                    bspline->show_pt_on_crv = false;
                }
                else
                {
                    // Disable gumball
                    interaction.gumball_mode = false;
                    gumball->reset();

                    // Disable point addition
                    interaction.add_pts = false;

                    interaction.insert_knot = true;
                    bspline->show_pt_on_crv = true;
                }
            }
            else if (ev->key_code == SAPP_KEYCODE_B)
            {
                if (interaction.show_bezier_aabb)
                {
                    interaction.show_bezier_aabb = false;
                    bspline->show_aabb = false;
                    state.buf_update_flag = true;
                }
                else
                {
                    interaction.show_bezier_aabb = true;
                    bspline->show_aabb = true;
                    state.buf_update_flag = true;
                }
            }
            else if (ev->key_code == SAPP_KEYCODE_MINUS)
            {
                interaction.gumball_size -= 0.02f;
                if(interaction.gumball_size < 0.04f) interaction.gumball_size = 0.04f;
            }
            else if (ev->key_code == SAPP_KEYCODE_EQUAL)
            {
                interaction.gumball_size += 0.02f;
                if(interaction.gumball_size > 0.1f) interaction.gumball_size = 0.1f;
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
            else if(interaction.insert_knot)
            {
                state.buf_update_flag = true;
                update_mouse_pos(ev);
                float scr_to_crv =  interaction.m_norm_x * bspline->length; 
                bspline->insert_knot(bspline->lookup(scr_to_crv),1);
            }
            else if(interaction.gumball_mode)
            {
                update_mouse_pos(ev);   
                int closest_idx = closest_cp_idx(bspline->control_points, state.mvp);
                if(closest_idx >= 0)
                {
                    interaction.selected_cp_index = closest_idx;
                    interaction.selected_cp_pos = closest_cp_pos(bspline->control_points, interaction.selected_cp_index);
                    gumball->origin = interaction.selected_cp_pos;
                }
                gumball->show = true;
                gumball->select_axis(state.mvp, sapp_widthf(), sapp_heightf(), interaction.mouse_x, interaction.mouse_y);
                if (gumball->active_axis != ActiveAxis::None)
                    interaction.dragging = true;
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
                gumball->drag_axis(ev->mouse_dx,ev->mouse_dy, sapp_widthf(), sapp_heightf());
                bspline->update_cp(interaction.selected_cp_index, gumball->origin);
            }
            if(interaction.insert_knot)
            {
                state.buf_update_flag = true;
                update_mouse_pos(ev);
                float scr_to_crv =  interaction.m_norm_x* bspline->length;
                bspline->slide_pt(scr_to_crv);
            }
        }
        camera->handle_events(ev);
        break;
    case MODE_EDIT_SURFACE:
        if (ev->type == SAPP_EVENTTYPE_KEY_DOWN)
        {
            if (ev->key_code == SAPP_KEYCODE_G)
            {
                if (interaction.gumball_mode)
                {
                    interaction.gumball_mode = false;
                   
                }
                else
                {
                    interaction.gumball_mode = true;
                    gumball->show;
                }  
            }
        }
        else if (ev->type == SAPP_EVENTTYPE_MOUSE_DOWN && ev->mouse_button == SAPP_MOUSEBUTTON_LEFT)
        {
            if(interaction.gumball_mode)
            {
                update_mouse_pos(ev);   
                int closest_idx = closest_cp_idx(surface->control_points, state.mvp);
                if(closest_idx >= 0)
                {
                    interaction.selected_cp_index = closest_idx;
                    interaction.selected_cp_pos = closest_cp_pos(surface->control_points, interaction.selected_cp_index);
                    gumball->origin = interaction.selected_cp_pos;
                }
                gumball->show = true;
                gumball->select_axis(state.mvp, sapp_widthf(), sapp_heightf(), interaction.mouse_x, interaction.mouse_y);
                if (gumball->active_axis != ActiveAxis::None)
                    interaction.dragging = true;
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
                gumball->drag_axis(ev->mouse_dx,ev->mouse_dy, sapp_widthf(), sapp_heightf());
                printf("Selected cpi:  %i\n", interaction.selected_cp_index);
                fflush(stdout);
                surface->update_srf_cp(interaction.selected_cp_index, gumball->origin);
            }
        }

        camera->handle_events(ev);
        break;
    }
}


void cleanup()
{
    // Cleanup geo and scene
    camera.reset();
    world_axis.reset();
    gumball.reset();
    bspline.reset();
    surface.reset();

    // Cleanup SOKOL
    simgui_shutdown();
    sfetch_shutdown();
    sg_shutdown();
}


static void response_callback(const sfetch_response_t* response)
{
    if(response->fetched)
    {
        int png_width, png_height, num_channels;
        const int desired_channels = 4;

        const void* data = response->data.ptr;
        size_t data_size = response->data.size;
        
        stbi_uc* pixels = stbi_load_from_memory(
            (const stbi_uc*)response->data.ptr,
            (int)response->data.size,
            &png_width, &png_height, &num_channels, desired_channels);

        if(pixels)
        {
            sg_image_desc img_desc = {};
            img_desc.width = png_width;
            img_desc.height = png_height;
            img_desc.pixel_format = SG_PIXELFORMAT_RGBA8;
            img_desc.data.mip_levels[0].ptr = pixels;
            img_desc.data.mip_levels[0].size = (size_t)(png_width*png_height*4);
            img_desc.label = "png_image";
            sg_image img = sg_make_image(img_desc);
            stbi_image_free(pixels);

            sg_view_desc view_desc = {};
            view_desc.texture.image = img;
            view_desc.label = "png_texture_view";
            sg_init_view(state.bind.views[VIEW_tex], view_desc);

            surface->mesh_bind.views[VIEW_tex] = state.bind.views[VIEW_tex];
            surface->mesh_bind.samplers[SMP_smp] = state.bind.samplers[SMP_smp];

            state.matcap_loaded = true;
        }
    }
    
    if(response->failed)
    {
        printf("Callback fired! fetched=%d, failed=%d\n", response->fetched, response->failed);
        fflush(stdout);
        switch(response->error_code)
        {
        case SFETCH_ERROR_FILE_NOT_FOUND:
            {
                printf("FileNotFound");
                fflush(stdout);
            }
                break;
        }
    }
}

// SOKOL initialize
void init()
{
    // Setup environment and logging function
    sg_desc desc = {};
    desc.environment = sglue_environment();
    desc.logger.func = slog_func;
    sg_setup(desc);

    // Setup sokol-debugtext
    sdtx_desc_t sdtx_desc = {};
    sdtx_desc.fonts[0] = sdtx_font_oric();
    sdtx_desc.logger.func = slog_func;
    sdtx_setup(sdtx_desc);  

    // Setup sokol fetch
    sfetch_desc_t sfetch = {};
    sfetch.max_requests = 1;
    sfetch.num_channels = 1;
    sfetch.num_lanes = 1;
    sfetch.logger.func = slog_func;
    sfetch_setup(sfetch);
    
    state.pass_action.colors[0].load_action = SG_LOADACTION_CLEAR;
    state.pass_action.colors[0].clear_value = {0.1f, 0.1f, 0.1f, 1.0f};

    state.bind.views[VIEW_tex] = sg_alloc_view();

    // Sampler object
    sg_sampler_desc sampler_desc = {};
    sampler_desc.min_filter = SG_FILTER_LINEAR;
    sampler_desc.mag_filter = SG_FILTER_LINEAR;
    sampler_desc.label = "png-sampler";
    state.bind.samplers[SMP_smp] = sg_make_sampler(sampler_desc);

    // Initialize geometry
    world_axis = std::make_unique<Gizmo>();
    gumball = std::make_unique<Gizmo>();

    // NURBS spline
    bspline = std::make_unique<NURBS_spline>(bsp, 3, 1000);
    bspline->generate(0);
    state.buf_update_flag = true;

    // NURBS surface
    surface = std::make_unique<NURBS_surface>(srf_cp, u_knots, v_knots, 3, 4, 4, 20, srf_weights);
    surface->generate_mesh();
    state.buf_update_flag = true;

    // Create shader
    sg_shader shd = sg_make_shader(shd_shader_desc(sg_query_backend()));
    sg_shader shd_mat = sg_make_shader(shd_matcap_shader_desc(sg_query_backend()));

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

    // Mesh matcap pipeline
    sg_pipeline_desc mat_desc = {};
    mat_desc.shader = shd_mat;
    mat_desc.layout.attrs[ATTR_shd_matcap_pos].format = SG_VERTEXFORMAT_FLOAT3;
    mat_desc.layout.attrs[ATTR_shd_matcap_color0].format = SG_VERTEXFORMAT_FLOAT4;
    mat_desc.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
    mat_desc.index_type = SG_INDEXTYPE_UINT16;
    //mat_desc.cull_mode = SG_CULLMODE_FRONT;
    mat_desc.label = "matcap_pipleline";
    state.pip_matcap = sg_make_pipeline(mat_desc);

    // Vertex pipeline
    sg_pipeline_desc pip_desc_verts = pip_desc_tri;
    pip_desc_verts.primitive_type = SG_PRIMITIVETYPE_POINTS;
    pip_desc_verts.label = "points_pipeline";
    state.pip_vertices = sg_make_pipeline(pip_desc_verts);

    // Load PNG
    char path_buf[512];
    sfetch_request_t request = {};
    request.path = "../../assets/matcap_2.png";
    request.callback = response_callback;
    request.buffer = SFETCH_RANGE(state.file_buffer);
    sfetch_send(request);

    // ImGUI setup
    simgui_desc_t simgui_desc = { };
    simgui_desc.logger.func = slog_func;
    simgui_setup(simgui_desc);

    // Font
    ImGuiIO& io = ImGui::GetIO();
    ImFontConfig font_cfg;
    font_cfg.OversampleH = 2;
    font_cfg.OversampleV = 2;
    font_cfg.PixelSnapH = true;
    ui_font = io.Fonts->AddFontFromFileTTF("../../assets/Ubuntu-Medium.ttf", 14.0f, &font_cfg);      

    // Overall Style
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 5.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.FramePadding = ImVec2(8,4);
    style.ItemSpacing = ImVec2(8, 6);
    //Buttons
    //style.Colors[ImGuiCol_Button] = ImVec4(0.8f,0.2f,0.2f,1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.1f,0.4f,0.6f,1.0f);
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.1f,0.1f,0.1f,1.0f);
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
    desc.enable_clipboard = true;
    desc.logger.func = slog_func;

    return desc;
}