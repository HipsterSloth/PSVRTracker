#ifndef APP_STAGE_COLOR_CALIBRATION_H
#define APP_STAGE_COLOR_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <vector>

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

	inline void set_override_controller_id(int controller_id)
	{ m_overrideControllerId= controller_id; }

	inline void set_override_hmd_id(int hmd_id)
	{ m_overrideHmdId = hmd_id; }

	inline void set_override_tracking_color(PSVRTrackingColorType tracking_color) {
		m_masterTrackingColorType = tracking_color;
	}
	
	inline void set_autoConfig(bool colour, bool controller, bool tracker) {
		m_bAutoChangeColor = colour;
		m_bAutoChangeController = controller;
		m_bAutoChangeTracker = tracker;
	}

protected:
    enum eMenuState
    {
        inactive,
        waitingForStreamStartResponse,
        manualConfig,
		autoConfig,
		blank1,
		blank2,
		changeController,
		changeTracker,

        failedControllerStartRequest,

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

    void setState(eMenuState newState);

    void request_start_controller_streams();
	void request_start_hmd_stream();
    void request_set_controller_tracking_color(PSVRController *controllerView, PSVRTrackingColorType tracking_color);
    void request_tracker_start_stream();
	bool request_tracker_set_mode(const char *new_mode);
    void request_tracker_set_video_property(PSVRVideoPropertyType prop_type, int value);
    void request_tracker_set_color_filter(const PSVRTrackingColorType color_type, const PSVR_HSVColorRange &color_filter);
    void request_tracker_get_settings();

    void allocate_video_buffers();
    void release_video_buffers();

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

	void request_turn_on_all_tracking_bulbs(bool bEnabled);

	void request_change_controller(int step);
	void request_change_tracker(int step);
    inline PSVR_HSVColorRange getColorPreset()
    { return m_colorPresetTable.color_presets[m_masterTrackingColorType]; }
	int getVideoPropertyStepSize(PSVRVideoPropertyType prop_type) const;
	bool canIncVideoProperty(PSVRVideoPropertyType prop_type) const;
	bool canDecVideoProperty(PSVRVideoPropertyType prop_type) const;

private:
    // ClientPSVRAPI state
	int m_overrideControllerId;	
    PSVRController *m_masterControllerView;
	std::vector<PSVRController *> m_controllerViews;
	std::vector<PSVRTrackingColorType> m_controllerTrackingColorTypes;
    bool m_areAllControllerStreamsActive;
    int m_lastMasterControllerSeqNum;
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
	int m_trackerModeIndex;
	int m_videoProperties[PSVRVideoProperty_COUNT];
    PSVR_HSVColorRangeTable m_colorPresetTable;
	int tracker_count;
	int tracker_index;

    // Color Settings
	bool m_bTurnOnAllControllers;
    PSVRTrackingColorType m_masterTrackingColorType;

	// Auto Calibration options
	bool m_bAutoChangeController;
	bool m_bAutoChangeColor;
	bool m_bAutoChangeTracker;
	bool m_bAutoCalibrate;

	// Setting Windows visibility
	bool m_bShowWindows;
	bool m_bShowAlignment;
	bool m_bShowAlignmentColor;
	float m_AlignmentOffset;
};

#endif // APP_STAGE_COLOR_CALIBRATION_H