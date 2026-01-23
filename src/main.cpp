#define HANDMADE_MATH_IMPLEMENTATION
#include "HandmadeMath.h"

#define SOKOL_IMPL
#define SOKOL_D3D11

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_log.h"

#include "shader.h"
#include "bezier_patch.h"
#include "camera.h"

float cp[48] = {
    -5.75f, -2.75f, 0.0f, -0.25f, -0.75f, 0.1f, 0.25f, -0.75f, 0.1f, 0.75f, -0.75f, 0.0f,
    -0.75f, -0.25f, 0.1f, -0.25f, -0.25f, 0.3f, 0.25f, -0.25f, 0.3f, 0.75f, -0.25f, 0.1f,
    -0.75f, 0.25f, 0.1f, -0.25f, 0.25f, 0.3f, 0.25f, 0.25f, 0.3f, 0.75f, 0.25f, 0.1f,
    -0.75f, 0.75f, 0.0f, -0.25f, 0.75f, 0.1f, 0.25f, 0.75f, 0.1f, 0.75f, 0.75f, 0.0f};

BezierPatch *patch = new BezierPatch(cp, 10);
Camera *camera = new Camera(HMM_V3(0, 0, 0), 2.0f, 45.0f, 30.0f);

static struct
{
    sg_pipeline pip;
    sg_bindings bind;
    sg_pass_action pass_action;
} state;

void frame()
{
    sg_pass pass = {};
    pass.action = state.pass_action;
    pass.swapchain = sglue_swapchain();

    sg_begin_pass(pass);
    sg_apply_pipeline(state.pip);

    // Calculate MVP matix
    HMM_Mat4 proj = HMM_Perspective_RH_ZO(60.0f, 640.0f / 480.0f, 0.01f, 10.0f);
    // HMM_Mat4 view = HMM_LookAt_RH(HMM_V3(0, 0.5f, 1.5f), HMM_V3(0, 0, 0), HMM_V3(0, 1, 0));
    HMM_Mat4 view = camera->get_view_matrix();
    HMM_Mat4 model = HMM_M4D(1.0f);
    HMM_Mat4 mvp = HMM_MulM4(proj, HMM_MulM4(view, model));

    // Pass to shader
    sg_apply_uniforms(0, SG_RANGE_REF(mvp));

    patch->render();
    sg_end_pass();
    sg_commit();
}

void event(const sapp_event *ev)
{
    if (ev->type == SAPP_EVENTTYPE_KEY_DOWN)
    {
        if (ev->key_code == SAPP_KEYCODE_ESCAPE)
        {
            sapp_quit();
        }
    }

    if (ev->type == SAPP_EVENTTYPE_MOUSE_SCROLL)
    {
        camera->zoom(ev->scroll_y * 0.5f);
    }

    if (ev->type == SAPP_EVENTTYPE_MOUSE_DOWN)
    {
        if (ev->mouse_button == SAPP_MOUSEBUTTON_LEFT)
        {
            sapp_lock_mouse(true);
        }
    }

    if (ev->type == SAPP_EVENTTYPE_MOUSE_UP)
    {
        if (ev->mouse_button == SAPP_MOUSEBUTTON_LEFT)
        {
            sapp_lock_mouse(false);
        }
    }

    if (ev->type == SAPP_EVENTTYPE_MOUSE_MOVE)
    {
        if (sapp_mouse_locked())
        {
            camera->orbit(ev->mouse_dx * 0.25f, ev->mouse_dy * 0.25f);
        }
    }
}

void init()
{
    // setup environment and logging function
    sg_desc desc = {};
    desc.environment = sglue_environment();
    desc.logger.func = slog_func;
    sg_setup(desc);

    patch->generate_mesh();

    // create shader
    sg_shader shd = sg_make_shader(shd_shader_desc(sg_query_backend()));

    // create pipeline
    sg_pipeline_desc pip_desc = {};
    pip_desc.shader = shd;
    pip_desc.layout.attrs[ATTR_shd_pos].format = SG_VERTEXFORMAT_FLOAT3;
    pip_desc.layout.attrs[ATTR_shd_color0].format = SG_VERTEXFORMAT_FLOAT4;
    pip_desc.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
    pip_desc.index_type = SG_INDEXTYPE_UINT16;
    pip_desc.label = "triangle_pipeline";

    state.pip = sg_make_pipeline(pip_desc);
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
