#ifndef APP_STAGE_TEST_TRACKER_H
#define APP_STAGE_TEST_TRACKER_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <vector>

//-- definitions -----
class AppStage_TrackerTest : public AppStage
{
public:
    AppStage_TrackerTest(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

    void request_tracker_start_stream();
    void request_tracker_stop_stream();

protected:
    void handle_tracker_start_stream_response();
    void open_shared_memory_stream();

    void handle_tracker_stop_stream_response();
    void close_shared_memory_stream();
    
private:
    enum eTrackerMenuState
    {
        inactive,
        idle,

        pendingTrackerStartStreamRequest,
        failedTrackerStartStreamRequest,

        pendingTrackerStopStreamRequest,
        failedTrackerStopStreamRequest,
    };

    eTrackerMenuState m_menuState;
    bool m_bStreamIsActive;
    PSVRTracker *m_tracker_view;
    PSVRVideoFrameSection m_current_section;
    int m_section_count;
    class TextureAsset *m_video_texture;
};

#endif // APP_STAGE_TEST_TRACKER_H