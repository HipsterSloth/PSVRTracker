#ifndef CAMERA_H
#define CAMERA_H

//-- includes -----
#include <glm/glm.hpp>

//-- constants -----
const float k_camera_default_forward_degrees = 270.f;

enum eCameraType
{
    _cameraNone,
    _cameraOrbit,
    _cameraFixed
};

//-- definitions -----
class Camera
{
public:
    Camera(class Renderer *renderer) 
        : m_renderer(renderer)
        , m_cameraOrbitYawDegrees(0.f)
        , m_cameraOrbitPitchDegrees(0.f)
        , m_cameraOrbitRadius(100.f)
        , m_cameraTarget(0.f, 0.f, 0.f)
        , m_cameraPosition(0.f, 0.f, 100.f)
        , m_isRotatingOrbitCamera(false)
        , m_isTranslatingOrbitCamera(false)
        , m_isLocked(false)
    { }

    void onMouseMotion(int deltaX, int deltaY);
    void onMouseButtonDown(int buttonIndex);
    void onMouseButtonUp(int buttonIndex);
    void onMouseWheel(int scrollAmount);

    void setIsLocked(bool locked);	
    void setCameraOrbitAngles(float yawDegrees, float pitchDegrees, float radius);
	void setCameraOrbitYaw(float yawDegrees);
	void setCameraOrbitPitch(float pitchDegrees);
    void setCameraOrbitRadius(float radius);
    void setCameraViewTarget(const glm::vec3 &cameraTarget);
    void resetOrientation();
    void reset();
    void publishCameraViewMatrix();

private:
    class Renderer *m_renderer;
    float m_cameraOrbitYawDegrees;
    float m_cameraOrbitPitchDegrees;
    float m_cameraOrbitRadius;
    glm::vec3 m_cameraTarget;
    glm::vec3 m_cameraPosition;
    bool m_isRotatingOrbitCamera;
    bool m_isTranslatingOrbitCamera;
    bool m_isLocked;
};

#endif //CAMERA_H