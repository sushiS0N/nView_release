#include "HandmadeMath.h"
#include "stdio.h"
#include <vector>


///// NURBS Spline class /////
class NURBS_spline
{
    public:
        std::vector<float> control_points;
        NURBS_spline(std::vector<float> cp, std::vector<float> knots, int degree, int num_pts, std::vector<float> weights_in={});

        void update_cp(int index, HMM_Vec3 new_pos);
        void generate();
        void update_buffer();
        void render_spline(const HMM_Mat4 &mvp);
        void render_control_points(const HMM_Mat4 &mvp);

    private:
        sg_buffer crv_vtx_buf, control_pts_buf;
        sg_bindings crv_bind, cp_bind;
        std::vector<float> knot_vector, weights, weighted_points, basis_funs, color_cp, crv_pts;
        int n, p, num_pts;

        void curve_point(float u, float *crv_pt);   
};


///// NURBS Surface class /////
class NURBS_surface
{
    public:
        NURBS_surface(std::vector<float> cp, std::vector<float> u_knot_vector, std::vector<float> v_knot_vector, 
                    int degree, int u_num, int v_num, int resolution, std::vector<float> weights_in = {});

        void generate_mesh();
        void update_buffer();
        void render_surface(const HMM_Mat4 &mvp);
        void render_control_points(const HMM_Mat4 &mvp);
        void render_control_polygon(const HMM_Mat4 &mvp);


    private:
        sg_buffer control_pts_buf, mesh_vtx_buf, mesh_idx_buf;
        sg_bindings cp_bind, mesh_bind;
        std::vector<float> control_points, u_knots, v_knots, weights, weighted_points, u_basis_funs, v_basis_funs;
        int n, m, p, q, u_num_pts, v_num_pts, num_pts, num_indices, resolution; 
        
        void surface_point(float u, float v, float *crv_pt);   
};
