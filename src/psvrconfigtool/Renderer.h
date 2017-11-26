#ifndef RENDERER_H
#define RENDERER_H

//-- includes -----
#include <glm/glm.hpp>
#include "PSVRClient_CAPI.h"

//-- typedefs -----
typedef union SDL_Event SDL_Event;

//-- macros -----
#if defined(__clang__) || defined(__GNUC__)
#define RENDERER_PRINTFARGS(FMT) __attribute__((format(printf, FMT, (FMT+1))))
#else
#define RENDERER_PRINTFARGS(FMT)
#endif

//-- definitions -----
class Renderer 
{
public:
    Renderer();
    ~Renderer();

    bool init();
    void destroy();

    bool onSDLEvent(const SDL_Event *event);

    void renderBegin();
    void renderStageBegin();
    void renderStageEnd();
    void renderUIBegin();
    void renderUIEnd();
    void renderEnd();

    static bool getIsRenderingStage() 
    { return m_instance != nullptr && m_instance->m_isRenderingStage; }
    static bool getIsRenderingUI()
    { return m_instance != nullptr && m_instance->m_isRenderingUI; }
    static float getWindowWidth()
    { return static_cast<float>(m_instance->m_windowWidth); }
    static float getWindowHeight()
    { return static_cast<float>(m_instance->m_windowHeight); }
    static float getWindowAspectRatio()
    { return static_cast<float>(m_instance->m_windowWidth) / static_cast<float>(m_instance->m_windowHeight); }

    void setProjectionMatrix(const glm::mat4 &matrix)
    { m_projectionMatrix= matrix; }
    void setCameraViewMatrix(const glm::mat4 &matrix)
    { m_cameraViewMatrix= matrix; }

    static const glm::mat4 &getCurrentProjectionMatrix()
    { return m_instance->m_projectionMatrix; }
    static const glm::mat4 &getCurrentCameraViewMatrix()
    { return m_instance->m_cameraViewMatrix; }

private:
    bool m_sdlapi_initialized;
    
    struct SDL_Window *m_window;
    int m_windowWidth, m_windowHeight;

    void *m_glContext;

    glm::mat4 m_projectionMatrix;
    glm::mat4 m_cameraViewMatrix;

    bool m_isRenderingStage;
    bool m_isRenderingUI;

    // imgui state
    double m_Time;
    bool m_MousePressed[3];
    float m_MouseWheel;
    unsigned int m_FontTexture;

    static Renderer *m_instance;
};

//-- drawing methods -----
PSVRVector2f remapPointIntoSubWindow(
    const float screenWidth, const float screenHeight,
    const float windowLeft, const float windowTop,
    const float windowRight, const float windowBottom,
    const PSVRVector2f &in_point);
glm::vec3 remapPointIntoSubWindow(
    const float screenWidth, const float screenHeight,
    const float windowLeft, const float windowTop,
    const float windowRight, const float windowBottom,
    const glm::vec3 &in_point);
void drawArrow(const glm::mat4 &transform, const glm::vec3 &start, const glm::vec3 &end, const float headFraction, const glm::vec3 &color);
void drawTextAtWorldPosition(const glm::mat4 &transform, const glm::vec3 &position, const char *format, ...) RENDERER_PRINTFARGS(3);
void drawTextAtScreenPosition(const glm::vec3 &screenCoords, const char *format, ...) RENDERER_PRINTFARGS(2);
void drawZeroTerminatedTextAtScreenPosition(const glm::vec3 &screenCoords, const char *text);
void drawFullscreenTexture(const unsigned int texture_id);
void drawFullscreenStereoTexture(
    const unsigned int left_texture_id, const unsigned int right_texture_id,
    const float top_y= 0.5f, const float bottom_y= -0.5f);
void drawPointCloudProjection(
    const float trackerWidth, const float trackerHeight,
    const glm::vec3 &color,
    const PSVRVector2f *points, const int point_count, const float point_size);
void drawPointCloudProjectionInSubWindow(
    const float trackerWidth, const float trackerHeight,
    const float windowX0, const float windowY0,
    const float windowX1, const float windowY1,
    const glm::vec3 &color,
    const PSVRVector2f *points, const int point_count, const float point_size);
void drawTransformedAxes(const glm::mat4 &transform, float scale);
void drawTransformedAxes(const glm::mat4 &transform, float xScale, float yScale, float zScale);
void drawTransformedBox(const glm::mat4 &transform, const glm::vec3 &half_extents, const glm::vec3 &color);
void drawTransformedBox(const glm::mat4 &transform, const glm::vec3 &box_min, const glm::vec3 &box_max, const glm::vec3 &color);
void drawTransformedTexturedCube(const glm::mat4 &transform, int textureId, float scale);
void drawTransformedFrustum(const glm::mat4 &transform, const PSVRFrustum *frustum, const glm::vec3 &color);
void drawWireframeTriangles(const glm::mat4 &transform, const float *vertices, const int *indices, const int triangle_count, const glm::vec3 &color);
void drawWireframeTriangle(const glm::mat4 &transform, const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &color, const float line_width);
void drawPointCloud(const glm::mat4 &transform, const glm::vec3 &color, const float *points, const int point_count);
void drawEllipsoid(
    const glm::mat4 &transform, const glm::vec3 &color, 
    const glm::mat3 &basis, const glm::vec3 &center, const glm::vec3 &extents,
    const int subdiv= 64);
void drawLineStrip(const glm::mat4 &transform, const glm::vec3 &color, const float *points, const int point_count);
void drawLineList2d(
    const float trackerWidth, const float trackerHeight, 
    const glm::vec3 &color, 
    const float *point_data_2d, const int point_count);
void drawQuadList2d(
    const float trackerWidth, const float trackerHeight, 
    const glm::vec3 &color, 
    const float *point_data_2d, const int point_count); // points2d={x0,y0,x1,y1,x2,y2,x3,y3, x0,y0,...}
void drawQuadList2dInSubWindow(
    const float trackerWidth, const float trackerHeight, 
    const float windowX0, const float windowY0,
    const float windowX1, const float windowY1,
    const glm::vec3 &color, 
    const float *point_data_2d, const int point_count);  // points2d={x0,y0,x1,y1,x2,y2,x3,y3, x0,y0,...}
void drawOpenCVChessBoard(
    const float trackerWidth, const float trackerHeight,
    const float *points2d, const int point_count, 
    bool valid_points);
void drawOpenCVChessBoardInSubWindow(
    const float trackerWidth, const float trackerHeight, 
    const float windowX0, const float windowY0,
    const float windowX1, const float windowY1,
    const float *points2d, const int point_count, 
    bool validPoints);
void drawPoseArrayStrip(const PSVRPosef *poses, const int poseCount, const glm::vec3 &color);
void drawPS3EyeModel(const glm::mat4 &transform);
void drawPS4CameraModel(const glm::mat4 &transform);
void drawTrackerList(const PSVRClientTrackerInfo *trackerList, const int trackerCount);
void drawMorpheusModel(const glm::mat4 &transform, const glm::vec3 &color);
void drawVirtualHMDModel(const glm::mat4 &transform, const glm::vec3 &color);

//-- Utilities -----
// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)
void RGBtoHSV(float r, float g, float b, float &h, float &s, float &v);
void HSVtoRGB(float h, float s, float v, float &r, float &g, float &b);

#endif // RENDERER_H
