#pragma once

/// BEZIER PATCH HEADER
class BezierPatch
{
public:
    BezierPatch(float *cp, int res);

    // Explicitly delete copy operations
    BezierPatch(const BezierPatch &) = delete;
    BezierPatch &operator=(const BezierPatch &) = delete;

    void evaluate(float u, float v, float *out_pos);
    void generate_mesh();
    void render();

    ~BezierPatch();

private:
    float control_points[48];
    int resolution;
    sg_buffer vertex_buffer;
    sg_buffer index_buffer;
    int num_indices;
};