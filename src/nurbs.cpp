#include "sokol_gfx.h"
#define SOKOL_COLOR_IMPL
#include "sokol_color.h"
#include "shader.h"

#include "stdio.h"
#include <vector>
#include <cassert>
#include <cmath>
#include <set>
#include <string>
#include <limits>

#include "nurbs.h"

///// Utility functions /////
std::vector<float> colour_points(const std::vector<float> &control_points, float r, float g, float b, float a)
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


int find_span(float u, int n, int p, const std::vector<float> &knot_vector)
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

void compute_basis_funs(float u, int i, int p, std::vector<float> &basis_funs, const std::vector<float> &knot_vector)
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

int find_multiplicity(float u, const std::vector<float> &knot_vector)
{
    int s = 0;

    for(int i=0; i<knot_vector.size(); i++)
        if(u == knot_vector[i]) s++;

    return s;
}

std::vector<float> project_3D(const std::vector<float> &pts_4D)
{
    std::vector<float> pts_3D(pts_4D.size()/4*3, 0.0f);
    for(int i = 0; i < pts_4D.size()/4; i++)
    {
        int idx = i*4;
        int ci = i*3;
        pts_3D[ci] = pts_4D[idx] / pts_4D[idx+3];
        pts_3D[ci+1] = pts_4D[idx+1] / pts_4D[idx+3];
        pts_3D[ci+2] = pts_4D[idx+2] / pts_4D[idx+3];
    }
    return pts_3D;
}

std::vector<float> extract_weights(const std::vector<float> &pts_4D)
{
    std::vector<float> weights(pts_4D.size()/4, 0.0f);
    //printf("Weights:");
    for(int i = 0; i < pts_4D.size()/4; i++)
    {
        int widx = i * 4 + 3;
        weights[i] = pts_4D[widx];
        //printf("%.2f, ",weights[i]);
    }
    //printf("\n");
    fflush(stdout);
    return weights;
}

std::vector<std::array<float,6>> compute_AABB(const std::vector<std::vector<float>>& Qw)   // Qw[seg][4D_cp * (p+1)]
{
    std::vector<std::array<float,6>> aabb;
    // Compute AABB bbox
    for(int i = 0; i<Qw.size(); i++)
    {
        std::vector<float> temp3d = std::move(project_3D(Qw[i]));
        float minX,minY, minZ, maxX, maxY, maxZ;
        minX = minY = minZ = std::numeric_limits<float>::max();
        maxX = maxY = maxZ = std::numeric_limits<float>::lowest();
    
        for(int i = 0; i<temp3d.size()/3; i++)
        {
            int idx = i*3;
            // x
            minX = temp3d[idx] < minX ? temp3d[idx] : minX;
            maxX = temp3d[idx] > maxX ? temp3d[idx] : maxX;
            // y
            minY = temp3d[idx+1] < minY ? temp3d[idx+1] : minY;
            maxY = temp3d[idx+1] > maxY ? temp3d[idx+1] : maxY;
            // z
            minZ = temp3d[idx+2] < minZ ? temp3d[idx+2] : minZ;
            maxZ = temp3d[idx+2] > maxZ ? temp3d[idx+2] : maxZ;
        }

        aabb.push_back({minX, minY, minZ, maxX, maxY, maxZ});
    }
    return aabb;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///// NURBS Spline functions //////////////////////////////////////////////////////////////////////
std::string NURBS_spline::print_knots()
{
    std::string knot_str = "{";
    for(float knot : knot_vector)
        knot_str += std::to_string(knot).substr(0,4) + ", ";
    knot_str += "}";
    return knot_str;
}

float NURBS_spline::lookup(float dist)
{   
    auto it = std::lower_bound(arc_lengths.begin(), arc_lengths.end(), dist);
    int i = it - arc_lengths.begin();

    if(dist>=arc_lengths.back()) return 1.0f;
    if(dist<=arc_lengths.front()) return 0.0f;

    if(i>arc_lengths.size()-2) i = arc_lengths.size()-2;

    if(arc_lengths[i] == dist)
    {
        return i/num_pts;
    }
    else
    {
        float d1 = arc_lengths[i];
        float d2 = arc_lengths[i+1];
        float t = (dist - d1) / (d2 - d1); 
        
        float u1 = (float)i / num_pts;
        float u2 = (float)(i+1) / num_pts;
        return u1 + (u2-u1)*t;
    }
}

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

    // Create point on curve buffer
    sg_buffer_desc pcrv_desc = {};
    pcrv_desc.size = sizeof(pt_crv);
    pcrv_desc.usage.dynamic_update = true;
    pcrv_desc.label = "bspline_point_on_crv_buffer";
    pt_on_crv_buf = sg_make_buffer(pcrv_desc);

    pt_on_crv_bind = {};
    pt_on_crv_bind.vertex_buffers[0] = this->pt_on_crv_buf;
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
    show_aabb = false;

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
    // Rebuild buffers and curve geometry
    regenerate_curve();
}

void NURBS_spline::slide_pt(float mval)
{  
    float u = lookup(mval);
    float outpos[3];
    curve_point(u, outpos);
    pt_crv[0] = outpos[0];
    pt_crv[1] = outpos[1];
    pt_crv[2] = outpos[2];

    // Set color
    sg_color clr = sg_alice_blue;
    pt_crv[3] = clr.r;
    pt_crv[4] = clr.g;
    pt_crv[5] = clr.b;
    pt_crv[6] = clr.a;
}

void NURBS_spline::insert_knot(float u, int r, bool rebuild_bufs)
{
    // Find the knot span
    int k = find_span(u, n, p, knot_vector);
    int s = find_multiplicity(u, knot_vector);

    printf("Span: %i, Multiplicity: %i \n",k,s);
    fflush(stdout);

    assert((r+s)<=p && "The multiplicity of the knot must not exceed the dregree r + s <= p");  // maybe introduce global multiplicity?

    int mp = n + p + 1;
    int nq = n + r;

    std::vector<float> Qw(weighted_points.size() + r * 4, 0.0f);    // new 4D CPs
    std::vector<float> Rw((p + 1) * 4, 0.0f);                       // temporary array for calculating new 4D CPs
    std::vector<float> UQ(knot_vector.size() + r, 0.0f);            // new knot vector

    // Fill new vector
    for (int i = 0; i <= k; i++) UQ[i] = knot_vector[i];
    for (int i = 1; i <= r; i++) UQ[k + i] = u;
    for (int i = k + 1; i <= mp; i++) UQ[i + r] = knot_vector[i];

    // Copy unaltered points
    for (int i = 0; i <= k - p; i++)
    {
        int idx = i * 4;
        Qw[idx] = weighted_points[idx];
        Qw[idx + 1] = weighted_points[idx + 1];
        Qw[idx + 2] = weighted_points[idx + 2];
        Qw[idx + 3] = weighted_points[idx + 3];
    }
    for (int i = k - s; i <= n; i++)
    {
        int idx = i * 4;
        int ir = (i+r)*4;
        Qw[ir] = weighted_points[idx];
        Qw[ir + 1] = weighted_points[idx + 1];
        Qw[ir + 2] = weighted_points[idx + 2];
        Qw[ir + 3] = weighted_points[idx + 3];
    }
    for (int i = 0; i <= p - s; i++)
    {
        int idx = i * 4;
        Rw[idx] = weighted_points[(k - p + i)*4];
        Rw[idx + 1] = weighted_points[(k - p + i)*4 + 1];
        Rw[idx + 2] = weighted_points[(k - p + i)*4 + 2];
        Rw[idx + 3] = weighted_points[(k - p + i)*4 + 3];
    }

    int L;
    // Insert the knot r times
    for (int j = 1; j <= r; j++)
    {
        L = k - p + j;
        for (int i = 0; i <= p - j - s; i++)
        {
            int idx = i * 4;
            // Calculate alpha values
            float alpha = (u - knot_vector[L + i]) / (knot_vector[i + k + 1] - knot_vector[L + i]);

            // Blend with alpha
            Rw[idx] = alpha * Rw[idx + 4] + (1.0f - alpha) * Rw[idx]; // [i+1] = [i+4] 4D flat array!
            Rw[idx + 1] = alpha * Rw[idx + 1 + 4] + (1.0f - alpha) * Rw[idx + 1];
            Rw[idx + 2] = alpha * Rw[idx + 2 + 4] + (1.0f - alpha) * Rw[idx + 2];
            Rw[idx + 3] = alpha * Rw[idx + 3 + 4] + (1.0f - alpha) * Rw[idx + 3];
        }

        // Copy first and last
        Qw[L * 4] = Rw[0];
        Qw[L * 4 + 1] = Rw[1];
        Qw[L * 4 + 2] = Rw[2];
        Qw[L * 4 + 3] = Rw[3];

        int krjs = (k + r - j - s) * 4;
        int pjs = (p - j - s) * 4;
        Qw[krjs] = Rw[pjs];
        Qw[krjs + 1] = Rw[pjs + 1];
        Qw[krjs + 2] = Rw[pjs + 2];
        Qw[krjs + 3] = Rw[pjs + 3];
    }

    // Load remaining CPs
        for (int i = L + 1; i < k - s; i++)
        {
            int idx = i * 4;
            int iL = (i - L) * 4;
            Qw[idx] = Rw[iL];
            Qw[idx + 1] = Rw[iL + 1];
            Qw[idx + 2] = Rw[iL + 2];
            Qw[idx + 3] = Rw[iL + 3];
        }

    // Update the curve member variables
    n = nq;
    weights = std::move(extract_weights(Qw));
    weighted_points = std::move(Qw);
    control_points = std::move(project_3D(weighted_points));
    knot_vector = std::move(UQ);

    if (rebuild_bufs) regenerate_curve();
}

void NURBS_spline::regenerate_curve()
{
    // Rebuild buffers
    sg_destroy_buffer(crv_vtx_buf);
    sg_destroy_buffer(control_pts_buf);
    sg_destroy_buffer(knots_buf);
    sg_destroy_buffer(pt_on_crv_buf);
    sg_destroy_buffer(aabb_buf);
    create_buffers();

    // Regenerate curve
    generate();
}

void NURBS_spline::convert_to_bezier()
{
    if (control_points.size() / 3 == (p + 1))
        return;

    std::set unique_knots(knot_vector.begin(), knot_vector.end());
    for (auto it : unique_knots)
    {
        int s = find_multiplicity(it, knot_vector);
        if (s >= p)
            continue;
        int r = p - s;
        insert_knot(it, r, false);
    }
    regenerate_curve();
}

void NURBS_spline::extract_bezier_segments()
{
    int m = n + p + 1;
    int a = p;
    int b = p + 1;
    int nb = 0;

    // Find individual spans to size Qw and alphas
    int unique_spans = 1;
    for (int i = p + 1; i < knot_vector.size() - p; i++)
    {
        if (knot_vector[i] != knot_vector[i + 1])
            unique_spans++;
    }

    std::vector<std::vector<float>> Qw(unique_spans, std::vector<float>((p + 1) * 4));
    std::vector<float> alphas(unique_spans * p);

    // Copy first 4 points
    for (int i = 0; i <= p; i++)
    {
        int idx = i * 4;
        Qw[nb][idx] = weighted_points[idx];
        Qw[nb][idx + 1] = weighted_points[idx + 1];
        Qw[nb][idx + 2] = weighted_points[idx + 2];
        Qw[nb][idx + 3] = weighted_points[idx + 3];
    }

    while (b < m)
    {
        int i = b;
        while (b < m && knot_vector[b + 1] == knot_vector[b]) b++;  // Get multiplicity
        int mult = b - i + 1;
        if (mult < p)
        {
            float numer = knot_vector[b] - knot_vector[a];          // Numerator of alpha
            for (int j = p; j > mult; j--)
                alphas[j - mult - 1] = numer / (knot_vector[a + j] - knot_vector[a]);
            int r = p - mult;
            for (int j = 1; j <= r; j++)
            {
                int save = r - j;
                int s = mult + j;                                   // This many new points
                for (int k = p; k >= s; k--)                        // Inser new points
                {
                    float k_idx = k * 4;
                    float alpha = alphas[k - s];
                    Qw[nb][k_idx] = alpha * Qw[nb][k_idx] + (1.0f - alpha) * Qw[nb][(k - 1) * 4];
                    Qw[nb][k_idx + 1] = alpha * Qw[nb][k_idx + 1] + (1.0f - alpha) * Qw[nb][(k - 1) * 4+1];
                    Qw[nb][k_idx + 2] = alpha * Qw[nb][k_idx + 2] + (1.0f - alpha) * Qw[nb][(k - 1) * 4+2];
                    Qw[nb][k_idx + 3] = alpha * Qw[nb][k_idx + 3] + (1.0f - alpha) * Qw[nb][(k - 1) * 4+3];
                }
                if (b < m)
                {
                    Qw[nb + 1][save*4] = Qw[nb][p*4];               // The last point of the bezier curve
                    Qw[nb + 1][save*4+1] = Qw[nb][p*4+1];
                    Qw[nb + 1][save*4+2] = Qw[nb][p*4+2];
                    Qw[nb + 1][save*4+3] = Qw[nb][p*4+3];
                }
            }
        }    
        nb = nb + 1;
        if (b < m)
        {
            for (int i = p - mult; i <= p; i++)
            {
                int idx = i * 4;
                int bpi_idx = (b - p + i) * 4;
                Qw[nb][idx] = weighted_points[bpi_idx];
                Qw[nb][idx + 1] = weighted_points[bpi_idx + 1];
                Qw[nb][idx + 2] = weighted_points[bpi_idx + 2];
                Qw[nb][idx + 3] = weighted_points[bpi_idx + 3];
            }

            a = b;
            b = b + 1;
        }
    }
    bezier_aabb = std::move(compute_AABB(Qw));
    bezier_segments = std::move(Qw);
    create_aabb_boxes();
}

void NURBS_spline::create_aabb_boxes()
{
    std::vector<float> box_pts;
    box_pts.reserve(bezier_aabb.size()*24*7);
    for(int i = 0; i<bezier_aabb.size(); i++)
    {
        auto push_vertex = [&box_pts](float x, float y, float z, sg_color col){
            box_pts.push_back(x);
            box_pts.push_back(y);
            box_pts.push_back(z);
            box_pts.push_back(col.r);
            box_pts.push_back(col.g);
            box_pts.push_back(col.b);
            box_pts.push_back(col.a);
        };
        
        auto& b = bezier_aabb[i];
        float minX=b[0], minY=b[1], minZ=b[2];
        float maxX=b[3], maxY=b[4], maxZ=b[5];

        HMM_Vec3 corners[8] = {
            // Base of the box
            {minX, minY, minZ}, {minX, minY, maxZ}, {maxX, minY, maxZ}, {maxX, minY, minZ},
            // Top of the box
            {minX, maxY, minZ}, {minX, maxY, maxZ}, {maxX, maxY, maxZ}, {maxX, maxY, minZ}
        };

        int edges[12][2] = {
            {0,1}, {1,2}, {2,3}, {3,0},
            {0,4}, {1,5}, {2,6}, {3,7},
            {4,5}, {5,6}, {6,7}, {7,4},
        };

        for(auto& e : edges)
        {
            push_vertex(corners[e[0]].X, corners[e[0]].Y, corners[e[0]].Z, sg_chartreuse);
            push_vertex(corners[e[1]].X, corners[e[1]].Y, corners[e[1]].Z, sg_chartreuse);
        }
    }
    aabb_pts = std::move(box_pts);
    create_aabb_buf();
}

void NURBS_spline::create_aabb_buf()
{
    if(aabb_buf.id != 0) sg_destroy_buffer(aabb_buf);

    sg_buffer_desc aabb_desc = {};
    aabb_desc.size = aabb_pts.size() * sizeof(float);
    aabb_desc.usage.dynamic_update = true;
    aabb_desc.label = "bspline_aabb_buffer";
    aabb_buf = sg_make_buffer(aabb_desc);

    aabb_bind = {};
    aabb_bind.vertex_buffers[0] = this->aabb_buf;
}
void NURBS_spline::generate(int selected_idx)
{
    crv_pts.resize((num_pts + 1) * 7, 0.0f);
    arc_lengths.resize((num_pts + 1), 0.0f);
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

        // Compute arc lengths
        int previdx = (i - 1) * 7;
        float distance = 0.0f;
        if (i == 0)
        {
            arc_lengths[i] = 0.0f;
        }
        else
        {
            distance = std::sqrt(std::pow((out_pos[0] - crv_pts[previdx]), 2) +
                                 std::pow((out_pos[1] - crv_pts[previdx + 1]), 2) +
                                 std::pow((out_pos[2] - crv_pts[previdx + 2]), 2));

            arc_lengths[i] = distance + arc_lengths[i-1];
        }

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

    // Updated curve length
    length = arc_lengths[num_pts];

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
    extract_bezier_segments();
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
    if (show_pt_on_crv)
    {
        sg_update_buffer(pt_on_crv_buf, SG_RANGE(pt_crv));
    }
    if (show_aabb)
    {
        sg_update_buffer(aabb_buf, sg_range{aabb_pts.data(), aabb_pts.size() * sizeof(float)});
    }
}

void NURBS_spline::render_spline(const HMM_Mat4 &mvp) const
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

void NURBS_spline::render_control_points(const HMM_Mat4 &mvp) const
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

void NURBS_spline::render_knots(const HMM_Mat4 &mvp) const
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

void NURBS_spline::render_pt_on_crv(const HMM_Mat4 &mvp) const
{
    sg_apply_bindings(pt_on_crv_bind);
    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 8.0f;
    params.draw_mode = 0;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, 1, 1);
}

void NURBS_spline::render_aabb(const HMM_Mat4 &mvp) const
{
    sg_apply_bindings(aabb_bind);
    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 8.0f;
    params.draw_mode = 0;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, aabb_pts.size()/7, 1);
}

NURBS_spline::~NURBS_spline()
{
    sg_destroy_buffer(crv_vtx_buf);
    sg_destroy_buffer(control_pts_buf);
    sg_destroy_buffer(knots_buf);
    sg_destroy_buffer(pt_on_crv_buf);
    sg_destroy_buffer(aabb_buf);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///// NURBS Surface functions //////////////////////////////////////////////////////////////////////
void NURBS_surface::create_buffers()
{
    // Create vertex buffer
    sg_buffer_desc vbuf_desc = {};
    vbuf_desc.size = mesh_verts.size() * sizeof(float); 
    vbuf_desc.usage.dynamic_update = true;   
    vbuf_desc.label = "NURBS_surface_vertices";
    mesh_vtx_buf = sg_make_buffer(vbuf_desc);

    // Create index buffer
    indices.resize(num_indices, 0);
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

    // Control polygon index buffer
    int num_seg = 2*(u_num_pts-1)*(v_num_pts-1) + (u_num_pts-1) + (v_num_pts-1);
    ctrl_indices.resize(0, 0);

    for (int i = 0; i < u_num_pts; i++)
    {
        for (int j = 1; j < v_num_pts; j++)
        {
            ctrl_indices.push_back(i*v_num_pts+j-1);
            ctrl_indices.push_back(i*v_num_pts+j);
        }
    }

    for (int j = 0; j < v_num_pts; j++)
    {
        for (int i = 1; i < u_num_pts; i++)
        {
            ctrl_indices.push_back(j + u_num_pts*i-u_num_pts);
            ctrl_indices.push_back(j + u_num_pts*i);
        }
    }

    sg_buffer_desc ctrl_ibuf_desc = {};
    ctrl_ibuf_desc.usage.index_buffer = true;
    ctrl_ibuf_desc.data.ptr = ctrl_indices.data();
    ctrl_ibuf_desc.data.size = ctrl_indices.size() * sizeof(uint16_t);
    ctrl_ibuf_desc.label = "NURBS_ctrl_polygon_indices";
    ctrl_poly_idx_buf = sg_make_buffer(ctrl_ibuf_desc);

    ctrl_poly_bind = {};
    ctrl_poly_bind.vertex_buffers[0] = control_pts_buf;
    ctrl_poly_bind.index_buffer = ctrl_poly_idx_buf;
}


NURBS_surface::NURBS_surface(std::vector<float> cp, std::vector<float> u_knot_vector, std::vector<float> v_knot_vector, 
                            int degree, int u_num, int v_num, int res, std::vector<float> weights_in)
{
    assert(u_knot_vector.size() == (u_num - 1 + degree + 2) && "Incorrect u knot size");
    assert(v_knot_vector.size() == (v_num - 1 + degree + 2) && "Incorrect v knot size");

    u_num_pts = u_num;
    v_num_pts = v_num;
    num_pts = u_num_pts * v_num_pts;
    resolution = res;

    n = u_num_pts - 1;  // last index in u direction
    m = v_num_pts - 1;  // last index in v direction

    p = degree;         // degree in u direction
    q = degree;         // degree in v direction

    num_indices = resolution*resolution*6;

    int num_verts = (resolution+1)*(resolution+1);
    mesh_verts.resize(num_verts * 7, 0.0f);

    // Control points
    assert(cp.size()>0 && "Control points are empty");
    control_points = std::move(cp);

    // Validate knot vectors(non-decreasing)
    for (size_t i = 1; i < u_knot_vector.size(); i++)
    {
        assert(u_knot_vector[i] >= u_knot_vector[i - 1] && "u knot vector must be non-decreasing");
    }
    for (size_t i = 1; i < v_knots.size(); i++)
    {
        assert(v_knot_vector[i] >= v_knot_vector[i - 1] && "v knot vector must be non-decreasing");
    }

    u_knots = std::move(u_knot_vector);
    v_knots = std::move(v_knot_vector);

    // Weights check and generate if empty
    if(weights_in.empty())
    {
        this->weights.resize(control_points.size()/3, 1.0f);
    }
    else this->weights = std::move(weights_in);

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

    create_buffers();
    color_cp = std::move(colour_points(control_points, 0.5f,0.2f,0.9f,1.0f));
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
        }
    }
    generate_normals();
}

void NURBS_surface::generate_normals()
{
    std::vector<HMM_Vec3> normals(mesh_verts.size()/7, HMM_V3(0,0,0));
    int cp1_idx = 0;
    int cp2_idx = 0;
    int cp3_idx = 0;
    int n_idx = 0;

    for (int i = 0; i < indices.size(); i+=3)
    {
        cp1_idx = indices[i] * 7;
        cp2_idx = indices[i+1] * 7;
        cp3_idx = indices[i+2] * 7;

        HMM_Vec3 v1 = HMM_V3(mesh_verts[cp2_idx] - mesh_verts[cp1_idx], mesh_verts[cp2_idx+1] - mesh_verts[cp1_idx+1], mesh_verts[cp2_idx+2] - mesh_verts[cp1_idx+2]);
        HMM_Vec3 v2 = HMM_V3(mesh_verts[cp3_idx] - mesh_verts[cp1_idx], mesh_verts[cp3_idx+1] - mesh_verts[cp1_idx+1], mesh_verts[cp3_idx+2] - mesh_verts[cp1_idx+2]);
        HMM_Vec3 normal = HMM_Norm(HMM_Cross(v1,v2));
        
        normals[indices[i]] = HMM_Add(normals[indices[i]], normal);
        normals[indices[i+1]] = HMM_Add(normals[indices[i+1]], normal);
        normals[indices[i+2]] = HMM_Add(normals[indices[i+2]], normal);
    }

    int cp_idx = 0;
    for (int i = 0; i < normals.size(); i++)
    {
        HMM_Vec3 temp = HMM_NormV3(normals[i]);
        HMM_Vec3 light_dir = HMM_NormV3(HMM_V3(0.0f, 1.0f, 5.0f));
        float light = HMM_Dot(temp, light_dir);

        sg_color color = sg_color_lerp(sg_dark_turquoise, sg_light_coral, (light + 1.0f) * 0.5f);

        cp_idx = i * 7;        
        mesh_verts[cp_idx + 3] = normals[i].X;
        mesh_verts[cp_idx + 4] = normals[i].Y;
        mesh_verts[cp_idx + 5] = normals[i].Z;
        // mesh_verts[cp_idx + 3] = color.r;
        // mesh_verts[cp_idx + 4] = color.g;
        // mesh_verts[cp_idx + 5] = color.b;
        mesh_verts[cp_idx + 6] = 1.0f;
    }
}

void NURBS_surface::update_srf_cp(int index, HMM_Vec3 new_pos)
{
    int cp_index = index * 7;
    int wp_index = index * 4;

    // Update control point
    control_points[index*3] = new_pos.X;
    // keep Y for now
    control_points[index*3+2] = new_pos.Z;

    // Update colour points
    color_cp[cp_index] = new_pos.X;
    // keep Y
    color_cp[cp_index + 2] = new_pos.Z;

    // Update weights
    weighted_points[wp_index] = color_cp[cp_index]*weights[index];
    // keep Y
    weighted_points[wp_index+2] = color_cp[cp_index+2]*weights[index];
    // keep same weight 

    generate_mesh();
}

void NURBS_surface::update_buffer()
{
    sg_update_buffer(mesh_vtx_buf, sg_range{mesh_verts.data(), mesh_verts.size() * sizeof(float)});
    sg_update_buffer(control_pts_buf, sg_range{color_cp.data(), color_cp.size() * sizeof(float)});
}

void NURBS_surface::render_surface(const HMM_Mat4 &mvp, const HMM_Mat4 &view, bool matcap_loaded) const
{
    sg_apply_bindings(mesh_bind);

    if (matcap_loaded)
    {
        // Matcap shader
        vs_mat_params_t params = {};
        memcpy(params.mvp, &mvp, sizeof(float) * 16);
        memcpy(params.view, &view, sizeof(float) * 16);
        sg_apply_uniforms(0, SG_RANGE_REF(params));
    }
    else
    {
        vs_params_t params = {};
        memcpy(params.mvp, &mvp, sizeof(float) * 16);
        params.draw_mode = 0;
        params.point_size = 0.0f;
        sg_apply_uniforms(0, SG_RANGE_REF(params));
    }

    sg_draw(0, num_indices, 1);
}

void NURBS_surface::render_control_points(const HMM_Mat4 &mvp) const
{
    sg_apply_bindings(scp_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 6.0f;
    params.draw_mode = 0;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, control_points.size()/3, 1);
}

void NURBS_surface::render_control_polygon(const HMM_Mat4 &mvp) const
{
    sg_apply_bindings(ctrl_poly_bind);

    // Struct for shader
    vs_params_t params = {};
    memcpy(params.mvp, &mvp, sizeof(float)*16);
    params.point_size = 6.0f;
    params.draw_mode = 0;

    sg_apply_uniforms(0, SG_RANGE_REF(params));
    sg_draw(0, ctrl_indices.size(), 1);
}

NURBS_surface::~NURBS_surface()
{
    sg_destroy_buffer(control_pts_buf);
    sg_destroy_buffer(mesh_vtx_buf);
    sg_destroy_buffer(mesh_idx_buf);
    sg_destroy_buffer(ctrl_poly_idx_buf);
}