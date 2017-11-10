//-- includes -----
#include "Camera.h"
#include "MathUtility.h"
#include "Renderer.h"

#include "SDL_mouse.h"

#include <glm/gtc/matrix_transform.hpp>

//-- constants -----
static const float k_camera_mouse_zoom_scalar= 50.f;
static const float k_camera_mouse_rotate_scalar= 0.5f;
static const float k_camera_mouse_translate_scalar= 1.f;
static const float k_camera_min_zoom= 100.f;
static const float k_camera_y_offset = 1.f;

//-- public methods -----
void Camera::onMouseMotion(int deltaX, int deltaY)
{
    if (!m_isLocked)
    {
        if (m_isRotatingOrbitCamera)
        {
            const float deltaYaw= -(float)deltaX * k_camera_mouse_rotate_scalar;
            const float deltaPitch= (float)deltaY * k_camera_mouse_rotate_scalar;

            setCameraOrbitAngles(
                m_cameraOrbitYawDegrees+deltaYaw, 
                m_cameraOrbitPitchDegrees+deltaPitch, 
                m_cameraOrbitRadius);
        }
        else if (m_isTranslatingOrbitCamera)
        {
            const float deltaRight= -(float)deltaX * k_camera_mouse_translate_scalar;
            const float deltaUp= (float)deltaY * k_camera_mouse_translate_scalar;

            const glm::vec3 forward = glm::normalize(m_cameraTarget - m_cameraPosition);
            const glm::vec3 right= glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
            const glm::vec3 up= glm::cross(right, forward);
            const glm::vec3 new_target= m_cameraTarget + right*deltaRight + up*deltaUp;

            setCameraViewTarget(new_target);
        }
    }
}

void Camera::onMouseButtonDown(int buttonIndex)
{
    if (!m_isLocked)
    {
        if (buttonIndex == SDL_BUTTON_LEFT)
        {
            m_isRotatingOrbitCamera= true;
        }
        else if (buttonIndex == SDL_BUTTON_MIDDLE)
        {
            m_isTranslatingOrbitCamera= true;
        }
    }
}

void Camera::onMouseButtonUp(int buttonIndex)
{
    if (!m_isLocked)
    {
        if (buttonIndex == SDL_BUTTON_LEFT)
        {
            m_isRotatingOrbitCamera= false;
        }
        else if (buttonIndex == SDL_BUTTON_MIDDLE)
        {
            m_isTranslatingOrbitCamera= false;
        }
    }
}

void Camera::onMouseWheel(int scrollAmount)
{
    if (!m_isLocked)
    {
        const float deltaRadius= (float)scrollAmount * k_camera_mouse_zoom_scalar;

        setCameraOrbitAngles(
            m_cameraOrbitYawDegrees, 
            m_cameraOrbitPitchDegrees, 
            m_cameraOrbitRadius+deltaRadius);
    }
}

void Camera::setIsLocked(bool locked)
{
    if (locked)
    {
        m_isLocked= true;
        m_isRotatingOrbitCamera= false;
    }
    else
    {
        m_isLocked= false;
    }
}

void Camera::setCameraOrbitAngles(float yawDegrees, float pitchDegrees, float radius)
{
    m_cameraOrbitYawDegrees= wrap_degrees(yawDegrees);
    m_cameraOrbitPitchDegrees= clampf(pitchDegrees, 0.f, 60.f);
    m_cameraOrbitRadius= fmaxf(radius, k_camera_min_zoom);

    const float yawRadians= degrees_to_radians(m_cameraOrbitYawDegrees);
    const float pitchRadians= degrees_to_radians(m_cameraOrbitPitchDegrees);
    const float xzRadiusAtPitch= m_cameraOrbitRadius*cosf(pitchRadians);

    m_cameraPosition= 
        glm::vec3(
            xzRadiusAtPitch*sinf(yawRadians),
            m_cameraOrbitRadius*sinf(pitchRadians),
            xzRadiusAtPitch*cosf(yawRadians))
        + glm::vec3(0.f, k_camera_y_offset, 0.f);

    publishCameraViewMatrix();
}

void Camera::setCameraOrbitYaw(float yawDegrees)
{
	setCameraOrbitAngles(yawDegrees, m_cameraOrbitPitchDegrees, m_cameraOrbitRadius);
}

void Camera::setCameraOrbitPitch(float pitchDegrees)
{
	setCameraOrbitAngles(m_cameraOrbitYawDegrees, pitchDegrees, m_cameraOrbitRadius);
}

void Camera::setCameraOrbitRadius(float radius)
{
    setCameraOrbitAngles(m_cameraOrbitYawDegrees, m_cameraOrbitPitchDegrees, radius);
}

void Camera::setCameraViewTarget(const glm::vec3 &cameraTarget)
{
    m_cameraTarget= cameraTarget;
    publishCameraViewMatrix();
}

void Camera::resetOrientation()
{
    setCameraOrbitAngles(0.f, 0.f, m_cameraOrbitRadius);
}

void Camera::reset()
{
    m_cameraOrbitYawDegrees= 0.f;
    m_cameraOrbitPitchDegrees= 0.f;
    m_cameraOrbitRadius= k_camera_min_zoom;
    m_cameraTarget= glm::vec3(0.f, 0.f, 0.f);
    publishCameraViewMatrix();
}

void Camera::publishCameraViewMatrix()
{
    const float yawRadians= degrees_to_radians(m_cameraOrbitYawDegrees);
    const float pitchRadians= degrees_to_radians(m_cameraOrbitPitchDegrees);
    const float xzRadiusAtPitch= m_cameraOrbitRadius*cosf(pitchRadians);

    m_cameraPosition= 
        glm::vec3(
            m_cameraTarget.x + xzRadiusAtPitch*sinf(yawRadians),
            m_cameraTarget.y + m_cameraOrbitRadius*sinf(pitchRadians),
            m_cameraTarget.z + xzRadiusAtPitch*cosf(yawRadians))
        + glm::vec3(0.f, k_camera_y_offset, 0.f);

    m_renderer->setCameraViewMatrix(
        glm::lookAt(
            m_cameraPosition,
            m_cameraTarget, // Look at tracking origin
            glm::vec3(0, 1, 0)));    // Up is up.
}
