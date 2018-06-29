#ifndef APP_STAGE_MONO_CALIBRATION_H
#define APP_STAGE_MONO_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <vector>

//-- definitions -----
class AppStage_MonoCalibration : public AppStage
{
public:
    AppStage_MonoCalibration(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

    void request_tracker_start_stream();
    void request_tracker_stop_stream();
    void request_tracker_set_temp_gain(int gain);
    void request_tracker_set_temp_exposure(int exposure);
    void request_tracker_set_intrinsic(const PSVRMonoTrackerIntrinsics &new_mono_intrinsics);
    void request_tracker_reload_settings();
    void request_exit();

    inline void setBypassCalibrationFlag(bool flag) { m_bypassCalibrationFlag= flag; }

protected:
    void renderCameraSettingsUI();

    void handle_tracker_start_stream_response();
    void open_shared_memory_stream();

    void handle_tracker_stop_stream_response();
    void close_shared_memory_stream();
    
private:
    enum eMenuState
    {
        inactive,
		showWarning,
		enterBoardSettings,
        capture,
        processingCalibration,
        testCalibration,

        pendingTrackerStartStreamRequest,
        failedTrackerStartStreamRequest,
        failedTrackerOpenStreamRequest,

        pendingTrackerStopStreamRequest,
        failedTrackerStopStreamRequest,
    };

    // Menu state
    eMenuState m_menuState;

	// Board Settings
	float m_square_length_mm;

    // Tracker Settings state
    int m_trackerExposure;
    int m_trackerGain;
    bool m_bypassCalibrationFlag;

    bool m_bStreamIsActive;
    PSVRTracker *m_tracker_view;
    class OpenCVMonoState *m_opencv_mono_state;
};

#endif // APP_STAGE_MONO_CALIBRATION_H