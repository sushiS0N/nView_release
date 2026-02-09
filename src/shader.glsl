@vs vs
layout(binding=0)uniform vs_params
{
    mat4 mvp;
    float point_size;
    int draw_mode;
};

in vec4 pos;
in vec4 color0;

out vec4 color;


void main()
{
    gl_Position = mvp * pos;
    if(draw_mode == 1)
    {
        vec3 normal = color0.xyz;
        color = vec4((normal + 1.0) * 0.5, 1.0);
    }
    else
    {
        color = color0;
    }
    gl_PointSize = point_size;
}
@end 

@fs fs
in vec4 color;
out vec4 frag_color;
void main()
{

    frag_color = color;    
}
@end

@program shd vs fs 