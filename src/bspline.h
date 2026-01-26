#include "HandmadeMath.h"
#include "stdio.h"
#include <vector>

class Bspline
{
    public:
        Bspline(std::vector<float> cp, std::vector<float> knots, int degree, int num_pts);
        void generate_bspline();
        void render_spline(const HMM_Mat4 &mvp);
        void render_control_points(const HMM_Mat4 &mvp);
        void render_control_polygon(const HMM_Mat4 &mvp);


    private:
        sg_buffer crv_vtx_buf, control_pts_buf;
        sg_bindings crv_bind, cp_bind;
        std::vector<float> control_points, knot_vector, basis_funs;
        int n, p, m, num_pts;

        int find_span(float u);
        void compute_basis_funs(int i, float u);
        void curve_point(float u, float *crv_pt);   
        std::vector<float> colour_points(float r, float g, float b, float a);     
};