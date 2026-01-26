#pragma once
#include "HandmadeMath.h"

class Camera
{
public:
    Camera(HMM_Vec3 target, float distance, float yaw, float pitch);

    HMM_Vec3 calculate_position() const;

    void orbit(float delta_yaw, float delta_pitch);
    void zoom(float delta_distance);
    void handle_events(const sapp_event *ev);
    void reset();

    HMM_Mat4 get_view_matrix() const;

private:
    HMM_Vec3 target;
    float distance;
    float yaw;
    float pitch;
};
