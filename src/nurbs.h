#include "HandmadeMath.h"
#include "stdio.h"
#include <vector>


///// NURBS Spline class /////
class NURBS_spline
{
    public:
        std::vector<float> control_points;
        bool show_influence, show_knots;
        
        NURBS_spline(std::vector<float> cp, int degree, int num_pts, std::vector<float> knots={}, std::vector<float> weights_in={});

        // Geometry
        void update_cp(int index, HMM_Vec3 new_pos);
        void generate(int selected_idx = -1);
        void add_cp(HMM_Vec3 pt);

        // Rendering
        void update_buffer();
        void render_spline(const HMM_Mat4 &mvp);
        void render_control_points(const HMM_Mat4 &mvp);
        void render_knots(const HMM_Mat4 &mvp);


    private:
        sg_buffer crv_vtx_buf, control_pts_buf, knots_buf;
        sg_bindings crv_bind, cp_bind, knots_bind;
        std::vector<float> knot_vector, knots_markers, weights, weighted_points, basis_funs, color_cp, crv_pts;
        int n, p, num_pts;

        void curve_point(float u, float *crv_pt);   
        void calc_weighted_pts();
        void calc_knots();
        void create_buffers();
};


///// NURBS Surface class /////
class NURBS_surface
{
    public:
        std::vector<float> control_points;

        NURBS_surface(std::vector<float> cp, std::vector<float> u_knot_vector, std::vector<float> v_knot_vector, 
                    int degree, int u_num, int v_num, int resolution, std::vector<float> weights_in = {});

        void generate_mesh();
        void update_srf_cp(int index, HMM_Vec3 new_pos);
        void update_buffer();
        void render_surface(const HMM_Mat4 &mvp);
        void render_control_points(const HMM_Mat4 &mvp);
        void render_control_polygon(const HMM_Mat4 &mvp);


    private:
        sg_buffer control_pts_buf, mesh_vtx_buf, mesh_idx_buf;
        sg_bindings scp_bind, mesh_bind;
        std::vector<float> color_cp, mesh_verts, u_knots, v_knots, weights, weighted_points, u_basis_funs, v_basis_funs;
        int n, m, p, q, u_num_pts, v_num_pts, num_pts, num_indices, resolution; 
        
        void create_buffers();
        void surface_point(float u, float v, float *crv_pt);   
};
