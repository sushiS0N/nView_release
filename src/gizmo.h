#pragma once
#include "HandmadeMath.h"

class Gizmo
{
public:
    Gizmo();
    ~Gizmo();

    // Axis indicator
    void render_axis_indicator(const HMM_Mat4& mvp);
    void render_gumball(const HMM_Mat4& mvp, bool show);

private:
    // Axis indicator data
    sg_buffer axis_gizmo_buffer;
    sg_bindings axis_gizmo_bind;
    float axis_gizmo_verts[42] = 
    {
        0.0f, 0.0f, 0.0f, 1.0f, 1.0f,1.0f,1.0f, // origin
        3.0f, 0.0f, 0.0f, 1.0f, 0.0f,0.0f,1.0f, // X - red

        0.0f, 0.0f, 0.0f, 1.0f, 1.0f,1.0f,1.0f, // origin
        0.0f, 3.0f, 0.0f, 0.0f, 1.0f,0.0f,1.0f, // Y - green

        0.0f, 0.0f, 0.0f, 1.0f, 1.0f,1.0f,1.0f, // origin
        0.0f, 0.0f, 3.0f, 0.0f, 0.0f,1.0f,1.0f, // Z - blue   
    };
    
    void axis_gizmo();
    void create_axis_buffer();

    // Gumball indicator data


};