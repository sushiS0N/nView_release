#define SOKOL_IMPL
#define SOKOL_D3D11

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_log.h"

#include "shader.h"
#include "bezier_patch.h"

float cp[48] = {
    -0.75f, -0.75f, 0.0f, -0.25f, -0.75f, 0.1f, 0.25f, -0.75f, 0.1f, 0.75f, -0.75f, 0.0f,
    -0.75f, -0.25f, 0.1f, -0.25f, -0.25f, 0.3f, 0.25f, -0.25f, 0.3f, 0.75f, -0.25f, 0.1f,
    -0.75f, 0.25f, 0.1f, -0.25f, 0.25f, 0.3f, 0.25f, 0.25f, 0.3f, 0.75f, 0.25f, 0.1f,
    -0.75f, 0.75f, 0.0f, -0.25f, 0.75f, 0.1f, 0.25f, 0.75f, 0.1f, 0.75f, 0.75f, 0.0f};

BezierPatch *patch = new BezierPatch(cp, 10);

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
    state.pass_action.colors[0].clear_value = {0.2f, 0.2f, 0.2f, 1.0f};
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
