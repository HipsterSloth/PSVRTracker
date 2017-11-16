#ifndef APP_STAGE_COLOR_CALIBRATION_H
#define APP_STAGE_COLOR_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <vector>
#include <string>

//-- definitions -----
class AppStage_ColorCalibration : public AppStage
{
public:
    AppStage_ColorCalibration(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

	inline void set_override_hmd_id(int hmd_id)
	{ m_overrideHmdId = hmd_id; }

	inline void set_override_tracking_color(PSVRTrackingColorType tracking_color) {
		m_masterTrackingColorType = tracking_color;
	}
	
protected:
    enum eMenuState
    {
        inactive,
        waitingForStreamStartResponse,

		pendingHmdStartRequest,
		failedHmdStartRequest,

        pendingTrackerStartStreamRequest,
        failedTrackerStartStreamRequest,
    };

    enum eVideoDisplayMode
    {
        mode_bgr,
        mode_hsv,
        mode_masked,

        MAX_VIDEO_DISPLAY_MODES
    };

    struct TrackerColorPreset
    {
        float hue_center;
        float hue_range;
        float saturation_center;
        float saturation_range;
        float value_center;
        float value_range;
    };

    void setState(eMenuState newState);

	void request_start_hmd_stream();
	void handle_start_hmd_response();

    void request_tracker_start_stream();
    void handle_tracker_start_stream_response();

	void request_tracker_set_frame_width(double value);
	void handle_tracker_set_frame_width_response();

	void request_tracker_set_frame_rate(double value);
	void handle_tracker_set_frame_rate_response();

    void request_tracker_set_exposure(double value);
    void handle_tracker_set_exposure_response();

    void request_tracker_set_gain(double value);
    void handle_tracker_set_gain_response();

    void request_tracker_set_color_preset(PSVRTrackingColorType color_type, TrackerColorPreset &color_preset);
    void handle_tracker_set_color_preset_response();

    void request_tracker_get_settings();
    void handle_tracker_get_settings_response();

    void allocate_video_buffers();
    void release_video_buffers();

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

    inline TrackerColorPreset getColorPreset()
    { return m_colorPresets[m_masterTrackingColorType]; }

private:
    // ClientPSVRAPI state
	int m_overrideHmdId;
	PSVRHeadMountedDisplay *m_hmdView;
	bool m_isHmdStreamActive;
	int m_lastHmdSeqNum;

    PSVRTracker *m_trackerView;

    // Menu state
    eMenuState m_menuState;
    class VideoBufferState *m_video_buffer_state;
    eVideoDisplayMode m_videoDisplayMode;

    // Tracker Settings state
	double m_trackerFrameWidth;
	double m_trackerFrameRate;
    double m_trackerExposure;
    double m_trackerGain;
    TrackerColorPreset m_colorPresets[PSVRTrackingColorType_MaxColorTypes];
	int tracker_count;
	int tracker_index;

    // Color Settings
    PSVRTrackingColorType m_masterTrackingColorType;

	// Setting Windows visability
	bool m_bShowWindows;
	bool m_bShowAlignment;
	bool m_bShowAlignmentColor;
	float m_AlignmentOffset;
};

#endif // APP_STAGE_COLOR_CALIBRATION_H