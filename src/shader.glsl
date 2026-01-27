@vs vs
layout(binding=0)uniform vs_params
{
    mat4 mvp;
    float point_size;
};

in vec4 pos;
in vec4 color0;
out vec4 color;

void main()
{
    gl_Position = mvp * pos;
    color = color0;
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