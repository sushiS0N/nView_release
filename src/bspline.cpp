#include "sokol_gfx.h"
#include "shader.h"

#include "stdio.h"
#include <vector>
#include <cassert>

#include "bspline.h"

std::vector<float> Bspline::colour_points(float r, float g, float b, float a)
{
    std::vector<float> colored_cp((n + 1) * 7, 0.0f);
    int point_idx = 0;
    int color_indx = 0;
    int idx = 0;
    for (size_t i = 0; i < control_points.size()/3; i++)
    {
        color_indx = i * 7;
        point_idx = i*3;
        idx = (n + 1) * 7;

        colored_cp[color_indx] = control_points[point_idx];
        colored_cp[color_indx + 1] = control_points[point_idx+1];
        colored_cp[color_indx + 2] = control_points[point_idx+2];

        // Colors
        colored_cp[color_indx + 3] = r;
        colored_cp[color_indx + 4] = g;
        colored_cp[color_indx + 5] = b;
        colored_cp[color_indx + 6] = a;
    }
    return colored_cp;
}

Bspline::Bspline(std::vector<float> cp, std::vector<float> knots, int degree, int num_pts, std::vector<float> weights_in)
{
    control_points = std::move(cp);
    knot_vector = std::move(knots);

    if(weights_in.empty())
    {
        this->weights.resize(control_points.size()/3, 1.0f);
    }
    else this->weights = std::move(weights_in);

    // Validate knot vector is non-decreasing
    for (size_t i = 1; i < knot_vector.size(); i++)
    {
        assert(knot_vector[i] >= knot_vector[i - 1] && "Knot vector must be non-decreasing");
    }

    // Create the 4D homogeneous control points array
    weighted_points.resize(control_points.size()/3*4, 0.0f);
    int wp_idx, cp_idx = 0;
    for(int i = 0; i<weights.size(); i++)
    {
        wp_idx = i*4;
        cp_idx = i*3;
        weighted_points[wp_idx] = control_points[cp_idx]*this->weights[i];
        weighted_points[wp_idx+1] = control_points[cp_idx+1]*this->weights[i];
        weighted_points[wp_idx+2] = control_points[cp_idx+2]*this->weights[i];
        weighted_points[wp_idx+3] = this->weights[i];
    }

    n = control_points.size() / 3 - 1;
    p = degree;
    m = knot_vector.size() - 1;
    this->num_pts = num_pts;
}


int Bspline::find_span(float u)
{
    int knot_index = n + 1;
    if (u == this->knot_vector[n + 1])
        return n;
    int low = p;
    int high = n + 1;

    int mid = (low + high) / 2;

    while (u < this->knot_vector[mid] || u >= this->knot_vector[mid + 1])
    {
        if (u < this->knot_vector[mid])
            high = mid;
        else
            low = mid;
        mid = (low + high) / 2;
    }

    return mid;
}

void Bspline::compute_basis_funs(int i, float u)
{
    basis_funs.resize(p + 1, 0.0f);
    basis_funs[0] = 1.0;
    std::vector<float> right(p + 1, 0.0f), left(p + 1, 0.0f);
    for (int j = 1; j <= p; j++)
    {
        left[j] = u - knot_vector[i + 1 - j];
        right[j] = knot_vector[i + j] - u;
        float saved = 0.0f;
        float temp = 0.0f;

        for (int r = 0; r < j; r++)
        {
            temp = basis_funs[r] / (right[r + 1] + left[j - r]);

            basis_funs[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        basis_funs[j] = saved;
    }
}

void Bspline::curve_point(float u, float *out_pos)
{
    int span = find_span(u);
    compute_basis_funs(span, u);
    float cw[4] = {0.0f,0.0f,0.0f,0.0f};


    printf("\n=== u=%f, span=%d ===\n", u, span);
    printf("Basis functions: ");
    for(int i = 0; i <= p; i++) {
        printf("N[%d]=%f ", i, basis_funs[i]);
    }
    printf("\n");

    out_pos[0] = 0.0f;
    out_pos[1] = 0.0f;
    out_pos[2] = 0.0f;

    for (int i = 0; i <= p; i++)
    {
        int cp_idx = (span - p + i) * 4;
        printf("i=%d, cp_idx=%d, wp=[%f,%f,%f,%f]\n", 
               i, cp_idx, 
               weighted_points[cp_idx], 
               weighted_points[cp_idx+1],
               weighted_points[cp_idx+2], 
               weighted_points[cp_idx+3]);
        cw[0] += basis_funs[i] * weighted_points[cp_idx];
        cw[1] += basis_funs[i] * weighted_points[cp_idx + 1];
        cw[2] += basis_funs[i] * weighted_points[cp_idx + 2];
        cw[3] += basis_funs[i] * weighted_points[cp_idx + 3];
    }    
    printf("cw=[%f,%f,%f,%f]\n", cw[0], cw[1], cw[2], cw[3]);
    out_pos[0] = cw[0]/cw[3];
    out_pos[1] = cw[1]/cw[3];
    out_pos[2] = cw[2]/cw[3];
}

void Bspline::generate_bspline()
{
    std::vector<float> crv_pts;
    crv_pts.resize((num_pts + 1) * 7, 0.0f);

    int idx = 0;
    for (int i = 0; i <= num_pts; i++)
    {
        idx = i * 7;

        float u = (float)i / (float)num_pts;
        float out_pos[3];
        curve_point(u, out_pos);

        crv_pts[idx] = out_pos[0];
        crv_pts[idx + 1] = out_pos[1];
        crv_pts[idx + 2] = out_pos[2];

        // Point color
        crv_pts[idx + 3] = 1.0f;
        crv_pts[idx + 4] = 1.0f;
        crv_pts[idx + 5] = 1.0f;
        crv_pts[idx + 6] = 1.0f;
    }

    // Create curve points buffer
    sg_buffer_desc crv_buf_desc = {};
    crv_buf_desc.data = sg_range{crv_pts.data(), crv_pts.size() * sizeof(float)};
    crv_buf_desc.label = "bspline_buffer";
    crv_vtx_buf = sg_make_buffer(crv_buf_desc);

    crv_bind = {};
    crv_bind.vertex_buffers[0] = this->crv_vtx_buf;

    // Color the points
    std::vector<float> color_cp = std::move(colour_points(1.0f, 0.0f, 1.0f, 1.0f));

    // Create control point buffer
    sg_buffer_desc crv_pt_buf_desc = {};
    crv_pt_buf_desc.data = sg_range{color_cp.data(), color_cp.size() * sizeof(float)};
    crv_pt_buf_desc.label = "bspline_cp_buffer";
    control_pts_buf = sg_make_buffer(crv_pt_buf_desc);

    cp_bind = {};
    cp_bind.vertex_buffers[0] = this->control_pts_buf;
}

void Bspline::render_spline(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(crv_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 10.0f;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, this->num_pts + 1, 1);
}

void Bspline::render_control_points(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(cp_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 10.0f;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, n + 1, 1);
}
