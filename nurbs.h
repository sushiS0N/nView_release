#pragma once
#include "HandmadeMath.h"
#include "stdio.h"
#include <vector>
#include <string>
#include <array>

#include "render_utils.h"


///// NURBS Spline class /////
class NURBS_spline
{
    public:
        std::vector<float> control_points;
        bool show_influence, show_knots, show_pt_on_crv, show_aabb;
        float length;
        GpuBuffer crv_vtx_buf_, cp_buf_, knot_buf_, pt_on_crv_buf_, aabb_buf_;
        
        NURBS_spline(const std::vector<float>& cp, int degree, int num_pts, const std::vector<float>& knots={}, const std::vector<float>& weights_in={});
    
        // Geometry
        void update_cp(int index, HMM_Vec3 new_pos);
        void generate(int selected_idx = -1);
        void add_cp(HMM_Vec3 pt);
        void regenerate_curve();

        // Knots
        void insert_knot(float u, int r, bool rebuild_bufs = true);
        void convert_to_bezier();           // transforms the curve into bezier segments based on knot value
        void extract_bezier_segments();     // return bezier segments [seg][CPs]
        float lookup(float dist);
        void slide_pt(float mval);
        std::string print_knots();

        // Rendering
        void update_buffer();

    private:
        // Geometry 
        std::vector<float> knot_vector, knots_markers, weights, weighted_points, basis_funs;
        std::vector<float> color_cp, crv_pts;
        std::vector<float> arc_lengths;
        std::vector<float> aabb_pts;
        std::vector<float> pt_crv = std::vector<float>(7,0.0f);
        
        std::vector<std::vector<float>> bezier_segments;    //[seg][4D_pts * (p+1)] 
        std::vector<std::array<float,6>> bezier_aabb;   //[seg][minX,minY,minZ,maxX,maxY,maxZ] 
        int n, p, num_pts;

        // Helpers
        void curve_point(float u, float *crv_pt);   
        void calc_weighted_pts();
        void calc_knots();
        void create_aabb_boxes();
        void create_buffers();
};


///// NURBS Surface class /////
class NURBS_surface
{
    public:
        std::vector<float> control_points;
        sg_bindings mesh_bind;
        GpuBuffer mesh_vtx_buf_, control_pts_buf_, control_poly_buf_;

        NURBS_surface(std::vector<float> cp, std::vector<float> u_knot_vector, std::vector<float> v_knot_vector, 
                    int degree, int u_num, int v_num, int resolution, std::vector<float> weights_in = {});

        void generate_mesh();
        void update_srf_cp(int index, HMM_Vec3 new_pos);
        void update_buffer();

        void render_surface(const HMM_Mat4 &mvp, const HMM_Mat4 &view, bool matcap_loaded) const;
        void render_control_points(const HMM_Mat4 &mvp) const;
        void render_control_polygon(const HMM_Mat4 &mvp) const;

    private:
        std::vector<float> color_cp, mesh_verts, ctrl_poly, u_knots, v_knots, weights, weighted_points, u_basis_funs, v_basis_funs;
        std::vector<uint16_t> indices, ctrl_indices;
        
        int n, m, p, q, u_num_pts, v_num_pts, num_pts, num_indices, resolution; 
        
        void create_buffers();
        void surface_point(float u, float v, float *crv_pt);   
        void generate_normals();
};
