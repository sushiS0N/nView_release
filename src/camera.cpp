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
Camera::Camera(HMM_Vec3 target, float distance, float yaw, float pitch, float res_width, float res_height)
{
    this->target = target;
    this->distance = distance;
    this->yaw = yaw;
    this->pitch = pitch;

    // sensitivity
    pan_sens = .02f;
    orbit_sens = 0.25f;
    zoom_sens = 0.5f;
}

void Camera::pan(float dx, float dy)
{
    HMM_Mat4 view = get_view_matrix();
    HMM_Vec3 cam_right = HMM_V3(view[0][0],view[1][0],view[2][0]);
    HMM_Vec3 cam_up = HMM_V3(view[0][1],view[1][1],view[2][1]);

    dx *= -pan_sens;
    dy *= pan_sens;
    HMM_Vec3 pan_delta = HMM_MulV3F(cam_right, dx) + HMM_MulV3F(cam_up, dy);
    target = HMM_AddV3(target, pan_delta);
}

// Orbit - change yaw and pitch
void Camera::orbit(float delta_yaw, float delta_pitch)
{
    yaw += delta_yaw * - orbit_sens;
    pitch += delta_pitch * orbit_sens;

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
    distance += delta_distance * -zoom_sens;

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
    distance = 30.0f;
    yaw = 45.0f;
    pitch = 30.0f;
}

void Camera::handle_events(const sapp_event *ev)
{
    
    switch (ev->type)
    {
    case SAPP_EVENTTYPE_KEY_DOWN:
        if (ev->key_code == SAPP_KEYCODE_F)
        {
            reset();
        }
        break;
    case SAPP_EVENTTYPE_KEY_UP:
        break;
    
    case SAPP_EVENTTYPE_MOUSE_DOWN:
        if(ev->mouse_button == SAPP_MOUSEBUTTON_RIGHT)
            sapp_lock_mouse(true);
        break;

    case SAPP_EVENTTYPE_MOUSE_UP:
        if(ev->mouse_button == SAPP_MOUSEBUTTON_RIGHT)
            sapp_lock_mouse(false);
        break;


    case SAPP_EVENTTYPE_MOUSE_SCROLL:
        zoom(ev->scroll_y);
        break;

    case SAPP_EVENTTYPE_MOUSE_MOVE:
        if (sapp_mouse_locked() && ev->modifiers & SAPP_MODIFIER_SHIFT)
        {
            pan(ev->mouse_dx, ev->mouse_dy);
        }
        else if (sapp_mouse_locked())
        {
            orbit(ev->mouse_dx, ev->mouse_dy);
        }
        break;
    }
}
