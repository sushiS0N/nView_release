#include "sokol_gfx.h"

#include "stdio.h"
#include <vector>
#include <array>
#include <cassert>
#include <cmath>
#include <set>
#include <string>
#include <limits>

#include "render_utils.h"

sg_pipeline make_pipeline(sg_shader shd, PipelineType pip_type, const char* label)
{
    sg_pipeline_desc desc = {};
    switch(pip_type)
    {
        case PipelineType::points:
            desc.primitive_type = SG_PRIMITIVETYPE_POINTS;
            break;
        case PipelineType::lines:
            desc.primitive_type = SG_PRIMITIVETYPE_LINES;
            break;
        case PipelineType::lines_indexed:
            desc.primitive_type = SG_PRIMITIVETYPE_LINES;
            desc.index_type = SG_INDEXTYPE_UINT16;
            break;
        case PipelineType::curves:
            desc.primitive_type = SG_PRIMITIVETYPE_LINE_STRIP;
            break;
        case PipelineType::triangles:
            desc.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
            desc.index_type = SG_INDEXTYPE_UINT16;
            break;
    }
    desc.shader = shd;
    desc.layout.attrs[ATTR_shd_pos].format = SG_VERTEXFORMAT_FLOAT3;
    desc.layout.attrs[ATTR_shd_color0].format = SG_VERTEXFORMAT_FLOAT4;
    desc.label = label;
    return sg_make_pipeline(desc);
}

sg_pipeline make_matcap_pipeline(sg_shader matcap_shd)
{
    sg_pipeline_desc desc = {};
    desc.shader = matcap_shd;
    desc.layout.attrs[ATTR_shd_matcap_pos].format = SG_VERTEXFORMAT_FLOAT3;
    desc.layout.attrs[ATTR_shd_matcap_color0].format = SG_VERTEXFORMAT_FLOAT4;
    desc.primitive_type = SG_PRIMITIVETYPE_TRIANGLES;
    desc.index_type = SG_INDEXTYPE_UINT16;
    //mat_desc.cull_mode = SG_CULLMODE_FRONT;
    desc.label = "matcap_pipeline";
    return sg_make_pipeline(desc);
}

namespace Renderer
{
    static std::array<sg_pipeline, PipelineType::COUNT> pipelines {};

    void init(sg_shader& general_shd, sg_shader& matcap_shd)
    {
        pipelines[PipelineType::points] = make_pipeline(general_shd, PipelineType::points, "points_pipeline");
        pipelines[PipelineType::lines] = make_pipeline(general_shd, PipelineType::lines, "lines_pipeline");
        pipelines[PipelineType::lines_indexed] = make_pipeline(general_shd, PipelineType::lines_indexed, "lines_indexed_pipeline");
        pipelines[PipelineType::curves] = make_pipeline(general_shd, PipelineType::curves, "curves_pipeline");
        pipelines[PipelineType::triangles] = make_pipeline(general_shd, PipelineType::triangles, "triangles_pipeline");
        pipelines[PipelineType::matcap] = make_matcap_pipeline(matcap_shd);
    }

    void draw(const GpuBuffer &buf, const HMM_Mat4 &mvp, PipelineType pipeline)
    {

        sg_apply_pipeline(pipelines[pipeline]);

        // Apply bindings
        if(pipeline == PipelineType::points)
        {
            sg_bindings temp = buf.bind_; // TODO: implement a DrawInfo struct and decouple buffer from draw call info
            temp.index_buffer = {};
            sg_apply_bindings(temp);
        }
        else
        {
            sg_apply_bindings(buf.bind_);
        }

        vs_params_t params = {};
        memcpy(params.mvp, &mvp, sizeof(float) * 16);
        params.point_size = buf.params_.point_size;
        params.draw_mode = buf.params_.draw_mode;
        sg_apply_uniforms(0, SG_RANGE_REF(params));

        sg_draw(0, buf.num_elements_, 1);
    }

    void draw_matcap(const GpuBuffer &buf, const HMM_Mat4 &mvp, const HMM_Mat4 &view, bool matcap_loaded)
    {
        if(!matcap_loaded) return;
        // Apply pipeline before drawing
        sg_apply_pipeline(pipelines[PipelineType::matcap]);

        printf("view state: %d\n", sg_query_view_state(buf.bind_.views[VIEW_tex]));
        printf("view id: %u\n", buf.bind_.views[VIEW_tex].id);
        fflush(stdout);
        // Apply bindings
        sg_apply_bindings(buf.bind_);

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
        sg_draw(0, buf.num_elements_, 1);
    }
};