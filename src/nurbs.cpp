#include "sokol_gfx.h"
#include "shader.h"

#include "stdio.h"
#include <vector>
#include <cassert>

#include "nurbs.h"

///// Utility functions /////
std::vector<float> colour_points(std::vector<float> control_points, float r, float g, float b, float a)
{
    int cp_size = control_points.size()/3;
    std::vector<float> colored_cp(cp_size * 7, 0.0f);
    int point_idx = 0;
    int color_indx = 0;
    int idx = 0;
    for (size_t i = 0; i < control_points.size()/3; i++)
    {
        color_indx = i * 7;
        point_idx = i*3;
        idx = cp_size * 7;

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


int find_span(float u, int n, int p, std::vector<float> knot_vector)
{
    int knot_index = n + 1;
    if (u == knot_vector[n + 1])
        return n;
    int low = p;
    int high = n + 1;

    int mid = (low + high) / 2;

    while (u < knot_vector[mid] || u >= knot_vector[mid + 1])
    {
        if (u < knot_vector[mid])
            high = mid;
        else
            low = mid;
        mid = (low + high) / 2;
    }

    return mid;
}

void compute_basis_funs(float u, int i, int p, std::vector<float> &basis_funs, std::vector<float> &knot_vector)
{
    basis_funs.resize(p + 1, 0.0f); // how is this passed and processed in memory good moemtn to learn!
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

//////////////////////////////////
///// NURBS Spline functions /////

NURBS_spline::NURBS_spline(std::vector<float> cp, std::vector<float> knots, int degree, int resolution, std::vector<float> weights_in)
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

    num_pts = resolution;
}

void NURBS_spline::curve_point(float u, float *out_pos)
{
    int span = find_span(u, this->n, this->p, this->knot_vector);
    compute_basis_funs(u, span, p, this->basis_funs, this->knot_vector);
    float cw[4] = {0.0f,0.0f,0.0f,0.0f};

    out_pos[0] = 0.0f;
    out_pos[1] = 0.0f;
    out_pos[2] = 0.0f;

    for (int i = 0; i <= p; i++)
    {
        int cp_idx = (span - p + i) * 4;
    
        cw[0] += basis_funs[i] * weighted_points[cp_idx];
        cw[1] += basis_funs[i] * weighted_points[cp_idx + 1];
        cw[2] += basis_funs[i] * weighted_points[cp_idx + 2];
        cw[3] += basis_funs[i] * weighted_points[cp_idx + 3];
    }    
}

void NURBS_spline::generate_bspline()
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
    std::vector<float> color_cp = std::move(colour_points(control_points,1.0f, 0.0f, 1.0f, 1.0f));

    // Create control point buffer
    sg_buffer_desc crv_pt_buf_desc = {};
    crv_pt_buf_desc.data = sg_range{color_cp.data(), color_cp.size() * sizeof(float)};
    crv_pt_buf_desc.label = "bspline_cp_buffer";
    control_pts_buf = sg_make_buffer(crv_pt_buf_desc);

    cp_bind = {};
    cp_bind.vertex_buffers[0] = this->control_pts_buf;
}

void NURBS_spline::render_spline(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(crv_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 10.0f;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, this->num_pts + 1, 1);
}

void NURBS_spline::render_control_points(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(cp_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 10.0f;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, n + 1, 1);
}


//////////////////////////////////
///// NURBS Surface functions /////

NURBS_surface::NURBS_surface(std::vector<float> cp, std::vector<float> u_knot_vector, std::vector<float> v_knot_vector, 
                            int degree, int u_num, int v_num, int res, std::vector<float> weights_in)
{
    control_points = std::move(cp);
    u_knots = std::move(u_knot_vector);
    v_knots = std::move(v_knot_vector);


    if(weights_in.empty())
    {
        this->weights.resize(control_points.size()/3, 1.0f);
    }
    else this->weights = std::move(weights_in);

    // Validate knot vector is non-decreasing
    for (size_t i = 1; i < u_knots.size(); i++)
    {
        assert(u_knots[i] >= u_knots[i - 1] && "Knot vector must be non-decreasing");
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

    u_num_pts = u_num;
    v_num_pts = v_num;
    num_pts = u_num_pts * v_num_pts;
    resolution = res;

    n = u_num_pts - 1; // last index in u direction
    m = v_num_pts - 1; // last index in v direction

    p = degree; // degree in u direction
    q = degree; // degree in v direction
}

void NURBS_surface::surface_point(float u, float v, float *out_pos)
{
    int uspan = find_span(u, n, p, u_knots);
    compute_basis_funs(u, uspan, p, u_basis_funs, u_knots);

    int vspan = find_span(v, m, q, v_knots);
    compute_basis_funs(v, vspan, q, v_basis_funs, v_knots);

    float sw[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    out_pos[0] = 0.0f;
    out_pos[1] = 0.0f;
    out_pos[2] = 0.0f;

    std::vector<float> temp((q+1) * 4, 0.0f);

    for (int l = 0; l <= q; l++)
    {
        temp[l] = 0.0f;

        for (int k = 0; k <= p; k++)
        {
            int i_idx = uspan - p + k;
            int j_idx = vspan - q + l;
            int srf_idx = (i_idx * v_num_pts + j_idx) * 4;
            int t_idx = l * 4;

            temp[t_idx] += u_basis_funs[k] * weighted_points[srf_idx];
            temp[t_idx + 1] += u_basis_funs[k] * weighted_points[srf_idx + 1];
            temp[t_idx + 2] += u_basis_funs[k] * weighted_points[srf_idx + 2];
            temp[t_idx + 3] += u_basis_funs[k] * weighted_points[srf_idx + 3];
        }
    }

    for (int l = 0; l <= q; l++)
    {
        int t_idx = l * 4;

        sw[0] += v_basis_funs[l] * temp[t_idx];
        sw[1] += v_basis_funs[l] * temp[t_idx + 1];
        sw[2] += v_basis_funs[l] * temp[t_idx + 2];
        sw[3] += v_basis_funs[l] * temp[t_idx + 3];
    }

    out_pos[0] = sw[0] / sw[3];
    out_pos[1] = sw[1] / sw[3];
    out_pos[2] = sw[2] / sw[3];
}

void NURBS_surface::generate_mesh()
{
    int num_verts = (resolution+1)*(resolution+1);
    std::vector<float> vertices(num_verts * 7);
    std::vector<uint16_t> indices(resolution*resolution*6);

    int vtx_idx = 0;
    // float min_z = 0.0f;
    // float max_z = 0.3f;
    for (int i = 0; i <= resolution; i++)
    {
        float u = (float)i / (float)resolution;
        for (int j = 0; j <= resolution; j++)
        {
            vtx_idx = i * (resolution + 1) + j;
            int arr_idx = vtx_idx * 7;
            float v = (float)j / (float)resolution;
            float out_pos[3];
            surface_point(u, v, out_pos);

            vertices[arr_idx] = out_pos[0];
            vertices[arr_idx + 1] = out_pos[1];
            vertices[arr_idx + 2] = out_pos[2];

            // Point color
            vertices[arr_idx + 3] = 1.0f;
            vertices[arr_idx + 4] = 0.8f;
            vertices[arr_idx + 5] = 1.0f;
            vertices[arr_idx + 6] = 1.0f;
        }
    }

    int quad_idx = 0;
    for (int i = 0; i < resolution; i++)
    {
        for (int j = 0; j < resolution; j++)
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
    vbuf_desc.label = "NURBS_surface_vertices";
    mesh_vtx_buf = sg_make_buffer(vbuf_desc);

    // Create index buffer
    sg_buffer_desc ibuf_desc = {};
    ibuf_desc.usage.index_buffer = true;
    ibuf_desc.data.ptr = indices.data();
    ibuf_desc.data.size = indices.size() * sizeof(uint16_t);
    ibuf_desc.label = "NURBS_surface_indices";
    mesh_idx_buf = sg_make_buffer(ibuf_desc);

    num_indices = indices.size();

    mesh_bind = {};
    mesh_bind.vertex_buffers[0] = mesh_vtx_buf;
    mesh_bind.index_buffer = mesh_idx_buf;
/*
    // Color the points
    std::vector<float> color_cp = std::move(colour_points(control_points,1.0f, 0.0f, 1.0f, 1.0f));

    // Create control point buffer
    sg_buffer_desc crv_pt_buf_desc = {};
    crv_pt_buf_desc.data = sg_range{color_cp.data(), color_cp.size() * sizeof(float)};
    crv_pt_buf_desc.label = "bspline_cp_buffer";
    control_pts_buf = sg_make_buffer(crv_pt_buf_desc);

    cp_bind = {};
    cp_bind.vertex_buffers[0] = this->control_pts_buf;
    */
}

void NURBS_surface::render_surface(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(mesh_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 10.0f;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, num_indices, 1);
}

/*
void NURBS_spline::render_control_points(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(cp_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 10.0f;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, n + 1, 1);
}
*/