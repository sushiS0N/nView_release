#pragma once

#include "HandmadeMath.h"
#include "stdio.h"
#include <vector>
#include <string>
#include <array>

#include "shader.h"


struct GpuBuffer
{
    sg_buffer buf_ = {};
    sg_bindings bind_ = {};
    vs_params_t params_ = {};
    int num_elements_ = 0;

    GpuBuffer() = default;
    GpuBuffer(int max_elements, int stride, bool dynamic, const char *label, int num_elems, float point_size, int draw_mode, std::vector<uint16_t> indices = {}) 
        : num_elements_{num_elems}
    {
        // Vertex buffer
        sg_buffer_desc desc = {};
        desc.size = max_elements * stride * sizeof(float);
        desc.usage.dynamic_update = true;
        desc.label = label;
        buf_ = sg_make_buffer(desc);
        bind_.vertex_buffers[0] = buf_;

        // Index buffer
        if (!indices.empty())
        {
            sg_buffer_desc idx_desc = {};
            idx_desc.usage.index_buffer = true;
            idx_desc.data = sg_range{indices.data(), indices.size() * sizeof(uint16_t)};
            idx_desc.label = "indices_buf";
            sg_buffer idx_buf = sg_make_buffer(idx_desc);
            bind_.index_buffer = idx_buf;
        }

        params_.point_size = point_size;
        params_.draw_mode = draw_mode;        
    }

    void update_buffer(const std::vector<float>& data)
    {
        sg_update_buffer(buf_, sg_range{data.data(), data.size() * sizeof(float)});
    }

    // Delete copy constructor
    GpuBuffer(const GpuBuffer&) = delete;
    GpuBuffer& operator=(const GpuBuffer&) = delete;

    // Move constructor and move assignment operator
    GpuBuffer(GpuBuffer&& other) noexcept
    {
        buf_ = std::exchange(other.buf_, {});
        bind_ = std::exchange(other.bind_, {});
        params_ = std::exchange(other.params_, {});
        num_elements_ = std::exchange(other.num_elements_, 0);
    }
    GpuBuffer& operator=(GpuBuffer&& other) noexcept
    {

        if (this != &other)
        {
            sg_destroy_buffer(this->buf_);
            this->bind_ = {};
            this->params_ = {};
            this->num_elements_ = 0;

            buf_ = std::exchange(other.buf_, {});
            bind_ = std::exchange(other.bind_, {});
            params_ = std::exchange(other.params_, {});
            num_elements_ = std::exchange(other.num_elements_, 0);

        }
        return *this;
    }

    ~GpuBuffer()
    {
        sg_destroy_buffer(buf_);
    }
};

enum PipelineType{ points, lines, lines_indexed, curves, triangles, matcap, COUNT};

namespace Renderer
{
    void init(sg_shader& general_shd, sg_shader& matcap_shd);
    void draw(const GpuBuffer &buf, const HMM_Mat4 &mvp, PipelineType pipeline);
    void draw_matcap(const GpuBuffer &buf, const HMM_Mat4 &mvp, const HMM_Mat4 &view, bool matcap_loaded);
}