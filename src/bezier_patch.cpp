#include "sokol_gfx.h"

#include "bezier_patch.h"

#include <cmath>
#include <cstdio>
#include <vector>
using namespace std;

// Interpolation functions
float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

float cubic_bernstein(float t, float p0, float p1, float p2, float p3)
{
    float invT = 1 - t;
    return (p0 * std::pow(invT, 3) + p1 * 3 * std::pow(invT, 2) * t + p2 * 3 * invT * std::pow(t, 2) +
            p3 * std::pow(t, 3));
}

// Bezier patch class functions
BezierPatch::BezierPatch(float *cp, int res)
{
    memcpy(control_points, cp, sizeof(control_points));
    resolution = res;
}

void BezierPatch::evaluate(float u, float v, float *out_pos)
{
    float temp_x[4], temp_y[4], temp_z[4];

    // loop through the 4 rows of CP
    for (int i = 0; i < 4; i++)
    {
        // stride index for the row -> 4cp * 3(xyz) = 12
        int idx = i * 12;

        // Horizontal pass
        temp_x[i] = cubic_bernstein(u, control_points[idx + 0], control_points[idx + 3], control_points[idx + 6],
                                    control_points[idx + 9]);
        temp_y[i] = cubic_bernstein(u, control_points[idx + 1], control_points[idx + 4], control_points[idx + 7],
                                    control_points[idx + 10]);
        temp_z[i] = cubic_bernstein(u, control_points[idx + 2], control_points[idx + 5], control_points[idx + 8],
                                    control_points[idx + 11]);
    }

    // Vertical pass
    out_pos[0] = cubic_bernstein(v, temp_x[0], temp_x[1], temp_x[2], temp_x[3]);
    out_pos[1] = cubic_bernstein(v, temp_y[0], temp_y[1], temp_y[2], temp_y[3]);
    out_pos[2] = cubic_bernstein(v, temp_z[0], temp_z[1], temp_z[2], temp_z[3]);
}

void BezierPatch::generate_mesh()
{
    int num_verts = (resolution + 1) * (resolution + 1);
    std::vector<float> vertices(num_verts * 7);
    std::vector<uint16_t> indices(resolution * resolution * 6);

    int vtx_idx = 0;

    // Compute vertex positions
    for (int i = 0; i <= resolution; i++)
    {
        float u = (float)i / (float)resolution;
        for (int j = 0; j <= resolution; j++)
        {
            float v = (float)j / (float)resolution;
            float out_pos[3];
            evaluate(u, v, out_pos);

            vtx_idx = i * (resolution + 1) + j;
            int arr_idx = vtx_idx * 7;

            // store vertex position
            vertices[arr_idx] = out_pos[0];
            vertices[arr_idx + 1] = out_pos[1];
            vertices[arr_idx + 2] = out_pos[2];

            // store vertex color
            vertices[arr_idx + 3] = out_pos[2];
            vertices[arr_idx + 4] = out_pos[2];
            vertices[arr_idx + 5] = out_pos[2];
            vertices[arr_idx + 6] = 1.0f;
        }
    }

    int quad_idx = 0;
    // Compute triangle indices
    for (int i = 0; i <= resolution - 1; i++)
    {
        for (int j = 0; j <= resolution - 1; j++)
        {
            // Create triangle indices array
            // Tri 1
            indices[quad_idx * 6] = i * (resolution + 1) + j;
            indices[quad_idx * 6 + 1] = (i + 1) * (resolution + 1) + j;
            indices[quad_idx * 6 + 2] = (i + 1) * (resolution + 1) + (j + 1);
            // Tri 2
            indices[quad_idx * 6 + 3] = i * (resolution + 1) + j;
            indices[quad_idx * 6 + 4] = (i + 1) * (resolution + 1) + (j + 1);
            indices[quad_idx * 6 + 5] = i * (resolution + 1) + (j + 1);

            quad_idx++;
        }
    }

    // Create vertex buffer
    sg_buffer_desc vbuf_desc = {};
    vbuf_desc.data = sg_range{vertices.data(), vertices.size() * sizeof(float)}; // keep as alternate way of ceating the struct
    // vbuf_desc.usage.dynamic_update = true;
    vbuf_desc.label = "bezierPatch_dynamicVert";
    vertex_buffer = sg_make_buffer(vbuf_desc);

    // Create index buffer
    sg_buffer_desc ibuf_desc = {};
    ibuf_desc.usage.index_buffer = true;
    ibuf_desc.data.ptr = indices.data();
    ibuf_desc.data.size = indices.size() * sizeof(uint16_t);
    ibuf_desc.label = "bezierPatch_indices";
    index_buffer = sg_make_buffer(ibuf_desc);

    num_indices = indices.size();
}

void BezierPatch::render()
{
    sg_bindings bind = {};
    bind.vertex_buffers[0] = this->vertex_buffer;
    bind.index_buffer = this->index_buffer;
    sg_apply_bindings(bind);
    sg_draw(0, this->num_indices, 1);
}

BezierPatch::~BezierPatch()
{
    sg_destroy_buffer(vertex_buffer);
    sg_destroy_buffer(index_buffer);
    printf("Bezier patch deleted");
}