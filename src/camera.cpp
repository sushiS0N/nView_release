
#include "stdio.h"
#include "camera.h"

HMM_Vec3 Camera::calculate_position() const
{
    // Convert to radians
    float yaw_rad = yaw * (HMM_PI / 180.0f);
    float pitch_rad = pitch * (HMM_PI / 180.0f);
    printf("yaw_rad: %.2f, pitch_rad: %.2f, distance: %.2f\n", yaw_rad, pitch_rad, distance);
    float camX = distance * HMM_CosF(pitch_rad) * HMM_CosF(yaw_rad);
    float camY = distance * HMM_SinF(pitch_rad);
    float camZ = distance * HMM_CosF(pitch_rad) * HMM_SinF(yaw_rad);

    HMM_Vec3 pos = HMM_V3(camX, camY, camZ) + target;
    printf("Camera pos: (%.2f, %.2f, %.2f)\n", pos.X, pos.Y, pos.Z);
    return pos;
}

// Constructor - initialize camera state
Camera::Camera(HMM_Vec3 target, float distance, float yaw, float pitch)
{
    this->target = target;
    this->distance = distance;
    this->yaw = yaw;
    this->pitch = pitch;
}

// Orbit - change yaw and pitch
void Camera::orbit(float delta_yaw, float delta_pitch)
{
    yaw += delta_yaw;
    pitch += delta_pitch;
}

// Zoom - change distance
void Camera::zoom(float delta_distance)
{
    this->distance += delta_distance;

    // Clamp distance
    if (distance < 0.1f)
    {
        distance = 0.1f;
    };
    if (distance > 100.0f)
    {
        distance = 100.0f;
    };
}

HMM_Mat4 Camera::get_view_matrix() const
{
    HMM_Vec3 position = calculate_position();

    return HMM_LookAt_RH(position, target, HMM_V3(0.0f, 1.0f, 0.0f));
}

// Reset camera
void Camera::reset()
{
    target = HMM_V3(0, 0, 0);
    distance = 5.0f;
    yaw = 0.0f;
    pitch = 0.0f;
}