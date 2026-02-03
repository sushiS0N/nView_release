#include "sokol_app.h"
#include "stdio.h"
#include "camera.h"

HMM_Vec3 Camera::calculate_position() const
{
    // Convert to radians
    float yaw_rad = yaw * (HMM_PI / 180.0f);
    float pitch_rad = pitch * (HMM_PI / 180.0f);

    float camX = distance * HMM_CosF(pitch_rad) * HMM_SinF(yaw_rad);
    float camY = distance * HMM_SinF(pitch_rad);
    float camZ = distance * HMM_CosF(pitch_rad) * HMM_CosF(yaw_rad);

    HMM_Vec3 pos = HMM_V3(camX, camY, camZ) + target;
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

    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;

    if (yaw < 0.0f)
    {
        yaw += 360.0f;
    }
    if (yaw > 360.0f)
    {
        yaw -= 360.0f;
    }
}

// Zoom - change distance
void Camera::zoom(float delta_distance)
{
    this->distance += delta_distance;

    // Clamp distance
    if (distance < 0.01f)
    {
        distance = 0.01f;
    };
    if (distance > 1000.0f)
    {
        distance = 1000.0f;
    };
}

// Calculate View matrix
HMM_Mat4 Camera::get_view_matrix() const
{
    HMM_Vec3 position = calculate_position();

    // convetion mismatch with HMM flipping Y for OpenGL standrds
    return HMM_LookAt_RH(position, target, HMM_V3(0.0f, 1.0f, 0.0f));
}

// Reset camera
void Camera::reset()
{
    target = HMM_V3(0, 0, 0);
    distance = 20.0f;
    yaw = 45.0f;
    pitch = 30.0f;
}

void Camera::handle_events(const sapp_event *ev)
{
    switch (ev->type)
    {
    case SAPP_EVENTTYPE_MOUSE_SCROLL:
        zoom(ev->scroll_y * -0.5f);
        break;

    case SAPP_EVENTTYPE_MOUSE_MOVE:
        {
            orbit(ev->mouse_dx * -0.25f, ev->mouse_dy * 0.25f);
        }
        break;
    }
}
