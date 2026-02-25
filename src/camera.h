#pragma once
#include "HandmadeMath.h"

class Camera
{
public:
    float orbit_sens, pan_sens, zoom_sens, res_width, res, height;
    Camera(HMM_Vec3 target, float distance, float yaw, float pitch, float res_width, float res_height);

    HMM_Vec3 calculate_position() const;

    void orbit(float delta_yaw, float delta_pitch);
    void zoom(float delta_distance);
    void pan(float dx, float dy);
    void handle_events(const sapp_event *ev);
    void reset();

    HMM_Mat4 get_view_matrix() const;

private:
    HMM_Vec3 target;
    float distance;
    float yaw;
    float pitch;
    bool panning;
};
