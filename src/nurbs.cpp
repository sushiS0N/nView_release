#include "sokol_gfx.h"
#define SOKOL_COLOR_IMPL
#include "sokol_color.h"
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



////////////////////////
///// NURBS Spline /////
void NURBS_spline::calc_knots()
{
    int len_kv = n + p + 2;
    float val_step = 1.0f / (len_kv - 2.0f * p - 1.0f);

    knot_vector.resize(len_kv, 0.0f);

    for (int i = p + 1; i < len_kv; i++)
    {
        if (i >= len_kv - (p + 1))
        {
            knot_vector[i] = 1.0f;
        }
        else
        {
            knot_vector[i] = knot_vector[i - 1] + val_step;
        }
    }
}

void NURBS_spline::calc_weighted_pts()
{
    weighted_points.resize(control_points.size() / 3 * 4, 0.0f);
    int wp_idx, cp_idx = 0;
    for (int i = 0; i < weights.size(); i++)
    {
        wp_idx = i * 4;
        cp_idx = i * 3;
        weighted_points[wp_idx] = control_points[cp_idx] * this->weights[i];
        weighted_points[wp_idx + 1] = control_points[cp_idx + 1] * this->weights[i];
        weighted_points[wp_idx + 2] = control_points[cp_idx + 2] * this->weights[i];
        weighted_points[wp_idx + 3] = this->weights[i];
    }
}

void NURBS_spline::create_buffers()
{
    // Create empty dynamic buffers
    sg_buffer_desc crv_buf_desc = {};
    crv_buf_desc.size = (num_pts+1) * 7 * sizeof(float);
    crv_buf_desc.usage.dynamic_update = true;    
    crv_buf_desc.label = "bspline_buffer";
    crv_vtx_buf = sg_make_buffer(crv_buf_desc);

    crv_bind = {};
    crv_bind.vertex_buffers[0] = this->crv_vtx_buf;

    // Create control point buffer
    sg_buffer_desc crv_pt_buf_desc = {};
    crv_pt_buf_desc.size = control_points.size()/3 * 7 * sizeof(float);
    crv_pt_buf_desc.usage.dynamic_update = true;
    crv_pt_buf_desc.label = "bspline_cp_buffer";
    control_pts_buf = sg_make_buffer(crv_pt_buf_desc);

    cp_bind = {};
    cp_bind.vertex_buffers[0] = this->control_pts_buf;

    // Create knot buffer
    sg_buffer_desc knot_buf_desc = {};
    knot_buf_desc.size = knot_vector.size() * 7 * sizeof(float);
    knot_buf_desc.usage.dynamic_update = true;
    knot_buf_desc.label = "bspline_knot_buffer";
    knots_buf = sg_make_buffer(knot_buf_desc);

    knots_bind = {};
    knots_bind.vertex_buffers[0] = this->knots_buf;

    knots_markers.resize((knot_vector.size()-6) * 7, 0.0f);
}

NURBS_spline::NURBS_spline(std::vector<float> cp, int degree, int resolution, std::vector<float> knots, std::vector<float> weights_in)
{
    // Control points
    control_points = std::move(cp);
    
    // Parameters
    n = control_points.size() / 3 - 1;
    p = degree;
    num_pts = resolution;
    show_influence = false;
    show_knots = false;

    // Knot vector
    if (knots.empty())
    {
        calc_knots();
    }
    else
    {
        // Validate knot vector is non-decreasing
        for (size_t i = 1; i < knots.size(); i++)
        {
            assert(knots[i] >= knots[i - 1] && "Knot vector must be non-decreasing");
        }

        knot_vector = std::move(knots);
    }

    // Weights
    if(weights_in.empty())
    {
        this->weights.resize(control_points.size()/3, 1.0f);
    }
    else this->weights = std::move(weights_in);

    // Create the 4D homogeneous control points array
    calc_weighted_pts();

    create_buffers();    
}

void NURBS_spline::curve_point(float u, float *out_pos)
{
    int span = find_span(u, n, p, knot_vector);
    compute_basis_funs(u, span, p, basis_funs, knot_vector);
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
    out_pos[0] = cw[0]/cw[3];
    out_pos[1] = cw[1]/cw[3];
    out_pos[2] = cw[2]/cw[3];
}


void NURBS_spline::update_cp(int index, HMM_Vec3 new_pos)
{
    int cp_idx = index * 3;
    int wp_idx = index * 4;

    // Update control point
    control_points[cp_idx] = new_pos.X;
    control_points[cp_idx+1] = new_pos.Y;
    control_points[cp_idx+2] = new_pos.Z;

    // Update weights
    weighted_points[wp_idx] = control_points[cp_idx]*weights[index];
    weighted_points[wp_idx+1] = control_points[cp_idx+1]*weights[index];
    weighted_points[wp_idx+2] = control_points[cp_idx+2]*weights[index];
    weighted_points[wp_idx+3] = weights[index];

    // Regenerate curve geo
    generate(index);
}

void NURBS_spline::add_cp(HMM_Vec3 new_pos)
{
    // Increase n and add new point
    this->n += 1;
    control_points.push_back(new_pos.X);
    control_points.push_back(new_pos.Y);
    control_points.push_back(new_pos.Z);

    // Calculate knots, weights and weighted points
    calc_knots();
    weights.push_back(1.0f);
    calc_weighted_pts();

    // Rebuild buffers
    create_buffers();

    // Generate curve
    generate();
}

void NURBS_spline::generate(int selected_idx)
{
    crv_pts.resize((num_pts + 1) * 7, 0.0f);
    int idx = 0;
    for (int i = 0; i <= num_pts; i++)
    {
        idx = i * 7;
        float influence = 0.0f;

        float u = (float)i / (float)num_pts;
        float out_pos[3];
        curve_point(u, out_pos);

        crv_pts[idx] = out_pos[0];
        crv_pts[idx + 1] = out_pos[1];
        crv_pts[idx + 2] = out_pos[2];

        int span = find_span(u, n, p, knot_vector);
        compute_basis_funs(u, span, p, basis_funs, knot_vector);    

        if(selected_idx!=-1 && selected_idx >= span-p && selected_idx <= span && show_influence)
        {
            int local_index = selected_idx - (span-p);
            //influence = basis_funs[local_index];
            influence = 1.0f;
        }

        sg_color clr = sg_color_lerp(sg_white_smoke, sg_red, influence);        
        // Point color
        crv_pts[idx + 3] = clr.r;
        crv_pts[idx + 4] = clr.g;
        crv_pts[idx + 5] = clr.b;
        crv_pts[idx + 6] = clr.a;
    }

    for(int i = p; i<=knot_vector.size()-1; i++)
    {
        float u = knot_vector[i];

        if(i>p && knot_vector[i]==knot_vector[i-1]) continue;

        float pos[3];
        curve_point(u, pos);

        idx = (i-p)*7;
        knots_markers[idx] = pos[0];
        knots_markers[idx+1] = pos[1];
        knots_markers[idx+2] = pos[2];

        knots_markers[idx+3] = sg_magenta.r;
        knots_markers[idx+4] = sg_magenta.g;
        knots_markers[idx+5] = sg_magenta.b;
        knots_markers[idx+6] = sg_magenta.a;
    }

    color_cp = colour_points(control_points, 0.5f,0.2f,0.9f,1.0f);
}

// Render functions
void NURBS_spline::update_buffer()
{
    sg_update_buffer(crv_vtx_buf, sg_range{crv_pts.data(), crv_pts.size() * sizeof(float)});
    sg_update_buffer(control_pts_buf, sg_range{color_cp.data(), color_cp.size() * sizeof(float)});
    if (show_knots)
    {
        sg_update_buffer(knots_buf, sg_range{knots_markers.data(), knots_markers.size() * sizeof(float)});
    }
}

void NURBS_spline::render_spline(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(crv_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 1.0f;
    params.draw_mode = 0;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, this->num_pts + 1, 1);
}

void NURBS_spline::render_control_points(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(cp_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 6.0f;
    params.draw_mode = 0;


    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, n + 1, 1);
}

void NURBS_spline::render_knots(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(knots_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 8.0f;
    params.draw_mode = 0;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, knots_markers.size()/7, 1);
}

//////////////////////////////////
///// NURBS Surface functions /////
void NURBS_surface::create_buffers()
{
    // Create vertex buffer
    sg_buffer_desc vbuf_desc = {};
    vbuf_desc.size = mesh_verts.size() * sizeof(float); 
    vbuf_desc.usage.dynamic_update = true;   
    vbuf_desc.label = "NURBS_surface_vertices";
    mesh_vtx_buf = sg_make_buffer(vbuf_desc);

    // Create index buffer
    std::vector<uint16_t> indices(num_indices, 0);
    int quad_idx = 0;
    for (int i = 0; i < resolution; i++)
    {
        for (int j = 0; j < resolution; j++)
        {
            // Create triangle indices array ccw winding
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

    sg_buffer_desc ibuf_desc = {};
    ibuf_desc.usage.index_buffer = true;
    ibuf_desc.data.ptr = indices.data();
    ibuf_desc.data.size = indices.size() * sizeof(uint16_t);
    ibuf_desc.label = "NURBS_surface_indices";
    mesh_idx_buf = sg_make_buffer(ibuf_desc);

    mesh_bind = {};
    mesh_bind.vertex_buffers[0] = mesh_vtx_buf;
    mesh_bind.index_buffer = mesh_idx_buf;

    // Control point buffer
    sg_buffer_desc cp_buf = {};
    cp_buf.size = control_points.size() / 3 * 7 * sizeof(float);
    cp_buf.usage.dynamic_update = true;   
    cp_buf.label = "NURBS_control_points";
    control_pts_buf = sg_make_buffer(cp_buf);

    scp_bind = {};
    scp_bind.vertex_buffers[0] = control_pts_buf;
}


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

    num_indices = resolution*resolution*6;

    int num_verts = (resolution+1)*(resolution+1);
    mesh_verts.resize(num_verts * 7, 0.0f);

    create_buffers();
    color_cp = std::move(colour_points(control_points, 1.0f, 0.0f, 0.0f, 1.0f));
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
    int vtx_idx = 0;
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

            mesh_verts[arr_idx] = out_pos[0];
            mesh_verts[arr_idx + 1] = out_pos[1];
            mesh_verts[arr_idx + 2] = out_pos[2];

            float h = 0.01f;            
            float u_minus = (u - h >= 0.0f) ? (u - h) : u;
            float u_plus  = (u + h <= 1.0f) ? (u + h) : u;
            float v_minus = (v - h >= 0.0f) ? (v - h) : v;
            float v_plus  = (v + h <= 1.0f) ? (v + h) : v;

            float temp[3];
            surface_point(u_minus, v, temp);
            HMM_Vec3 S_u_minus = HMM_V3(temp[0], temp[1], temp[2]);

            surface_point(u_plus, v, temp);
            HMM_Vec3 S_u_plus = HMM_V3(temp[0], temp[1], temp[2]);

            surface_point(u, v_minus, temp);
            HMM_Vec3 S_v_minus = HMM_V3(temp[0], temp[1], temp[2]);

            surface_point(u, v_plus, temp);
            HMM_Vec3 S_v_plus = HMM_V3(temp[0], temp[1], temp[2]);

            HMM_Vec3 S_u = HMM_DivV3F(HMM_SubV3(S_u_plus, S_u_minus), 2.0f * h);
            HMM_Vec3 S_v = HMM_DivV3F(HMM_SubV3(S_v_plus, S_v_minus), 2.0f * h);
            HMM_Vec3 normal = HMM_NormV3(HMM_Cross(S_u, S_v));

            float light = HMM_Dot(normal, HMM_V3(0.5f, 1.0f, 0.5f));
            light *= .7;

            // Light color
            mesh_verts[arr_idx + 3] = light;
            mesh_verts[arr_idx + 4] = light;
            mesh_verts[arr_idx + 5] = light;

            // // Normal color
            // mesh_verts[arr_idx + 3] = normal.X;
            // mesh_verts[arr_idx + 4] = normal.Y;
            // mesh_verts[arr_idx + 5] = normal.Z;

            mesh_verts[arr_idx + 6] = 1.0f;
        }
    }
}

void NURBS_surface::update_srf_cp(int index, HMM_Vec3 new_pos)
{
    int cp_index = index * 3;
    int wp_index = index * 4;

    // Update control point
    control_points[cp_index] = new_pos.X;
    // keep Y
    control_points[cp_index + 2] = new_pos.Z;

    weighted_points[wp_index] = control_points[cp_index]*weights[index];
    // keep Y
    weighted_points[wp_index+2] = control_points[cp_index+2]*weights[index];
    // keep same weight 

    color_cp = std::move(colour_points(control_points, 1.0f, 0.0f,0.0f,1.0f));

    generate_mesh();
}

void NURBS_surface::update_buffer()
{
    sg_update_buffer(mesh_vtx_buf, sg_range{mesh_verts.data(), mesh_verts.size() * sizeof(float)});
    sg_update_buffer(control_pts_buf, sg_range{color_cp.data(), color_cp.size() * sizeof(float)});

}


void NURBS_surface::render_surface(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(mesh_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 10.0f;
    params.draw_mode = 0;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, num_indices, 1);
}

void NURBS_surface::render_control_points(const HMM_Mat4 &mvp)
{
    sg_apply_bindings(scp_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 10.0f;
    params.draw_mode = 0;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, control_points.size()/3, 1);
}