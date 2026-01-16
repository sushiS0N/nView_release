#define SOKOL_IMPL
#define SOKOL_D3D11
#include "sokol_app.h"
#include "sokol_log.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "shader.h"
#include <cstdio>
#include <cmath>
using namespace std;

#define NUM_POINTS 100

float vertices[NUM_POINTS * 7];

static struct
{
    sg_pipeline pip;
    sg_bindings bind;
    sg_pass_action pass_action;
} state;

// Interpolations
float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

float cubic_bernstein(float t, float p0, float p1, float p2, float p3)
{
    float invT = 1 - t;
    return (p0 * std::pow(invT, 3) +
            p1 * 3 * std::pow(invT, 2) * t +
            p2 * 3 * invT * std::pow(t, 2) +
            p3 * std::pow(t, 3));
}

void frame()
{
    // Control points where the middle one oscilates sin(time)
    float p0_x = -0.5f, p0_y = -0.5f;

    float time = (float)sapp_frame_count() * 0.05f;
    float p1_x = 0.0f, p1_y = 0.0f + sin(time) * 0.3f;

    float p2_x = 0.7f, p2_y = -0.8f;
    float p3_x = 0.9f, p3_y = 0.0f;

    // Bernstein evaluation
    for (int i = 0; i < NUM_POINTS; i++)
    {
        // normalize the t value
        float t = (float)i / (float)(NUM_POINTS - 1);

        // Bernstein points
        float x = cubic_bernstein(t, p0_x, p1_x, p2_x, p3_x);
        float y = cubic_bernstein(t, p0_y, p1_y, p2_y, p3_y);

        // Fill buffer
        int index = i * 7; // 7 floats per vertex
        vertices[index + 0] = x;
        vertices[index + 1] = y;
        vertices[index + 2] = 0.0f;

        // Color
        vertices[index + 3] = 1.0f - t;
        vertices[index + 4] = 0.0f;
        vertices[index + 5] = t * sin(time);
        vertices[index + 6] = 1.0f;
    }

    sg_update_buffer(state.bind.vertex_buffers[0], SG_RANGE(vertices));

    sg_pass pass = {};
    pass.action = state.pass_action;
    pass.swapchain = sglue_swapchain();

    sg_begin_pass(pass);
    sg_apply_pipeline(state.pip);
    sg_apply_bindings(state.bind);
    sg_draw(0, NUM_POINTS, 1);
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

    // float vertices[] = {
    //     0.0f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f, 1.0f,
    //     0.5f, -0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 1.0f,
    //     -0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 1.0f};
    // // static buffer - simple triangle
    // sg_buffer_desc buffer = {};
    // buffer.data = SG_RANGE(vertices);
    // buffer.label = "triangle_vertices";
    // state.bind.vertex_buffers[0] = sg_make_buffer(buffer);

    sg_buffer_desc buffer_desc = {};
    buffer_desc.size = sizeof(vertices);
    buffer_desc.usage.dynamic_update = true;
    buffer_desc.label = "dynamic-vertices";
    state.bind.vertex_buffers[0] = sg_make_buffer(buffer_desc);

    // create shader
    sg_shader shd = sg_make_shader(shd_shader_desc(sg_query_backend()));

    // create pipeline
    sg_pipeline_desc pip_desc = {};
    pip_desc.shader = shd;
    pip_desc.layout.attrs[ATTR_shd_pos].format = SG_VERTEXFORMAT_FLOAT3;
    pip_desc.layout.attrs[ATTR_shd_color0].format = SG_VERTEXFORMAT_FLOAT4;
    pip_desc.primitive_type = SG_PRIMITIVETYPE_LINE_STRIP;
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
