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

// MATCAP shader
@vs vs_matcap
layout(binding=0)uniform vs_mat_params
{
    mat4 mvp;
    mat4 view;
};

in vec4 pos;
in vec4 color0;

out vec2 matcap_uv;
out vec4 color;

void main()
{
    gl_Position = mvp * pos;

    vec3 view_normal = normalize(mat3(view) * color0.xyz);
    matcap_uv = view_normal.xy * 0.5 + 0.5;

}
@end 

@fs fs_matcap
layout(binding = 0) uniform texture2D tex;
layout(binding = 0) uniform sampler smp;

in vec2 matcap_uv;
in vec4 color;

out vec4 frag_color;

void main()
{
    frag_color = texture(sampler2D(tex, smp), matcap_uv);
}
@end

@program shd_matcap vs_matcap fs_matcap
