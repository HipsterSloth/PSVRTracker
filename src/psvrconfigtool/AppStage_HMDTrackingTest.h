#ifndef APP_STAGE_COMPUTE_TRACKER_POSES_H
#define APP_STAGE_COMPUTE_TRACKER_POSES_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <map>

//-- definitions -----
class AppStage_HMDTrackingTest : public AppStage
{
public:
	struct HMDState
	{
		int listIndex;
		PSVRTrackingColorType trackingColorType;
		PSVRHeadMountedDisplay *hmdView;
	};
	typedef std::map<int, HMDState> t_hmd_state_map;
	typedef std::map<int, HMDState>::iterator t_hmd_state_map_iterator;
	typedef std::pair<int, HMDState> t_id_hmd_state_pair;

    AppStage_HMDTrackingTest(class App *app);

    static void enterStageAndTestTracking(class App *app, PSVRHmdID requested_hmd_id=-1);

	inline void set_tracker_id(PSVRTrackerID reqeusted_tracker_id)
	{
		m_overrideTrackerId = reqeusted_tracker_id;
	}

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    enum eMenuState
    {
        inactive,

        pendingHmdListRequest,
        failedHmdListRequest,

        pendingHmdStartRequest,
        failedHmdStartRequest,

        pendingTrackerListRequest,
        failedTrackerListRequest,

        pendingTrackerStartRequest,
        failedTrackerStartRequest,

        verifyTrackers,

        testTracking,
		showTrackerVideo,
        calibrateStepFailed,
    };

    void setState(eMenuState newState);
    void onExitState(eMenuState newState);
    void onEnterState(eMenuState newState);

    void update_tracker_video();
    void render_tracker_video();
    PSVRTracker *get_render_tracker_view() const;
    PSVRHeadMountedDisplay *get_calibration_hmd_view() const;

    void request_hmd_list();
    void handle_hmd_list_response(const PSVRHmdList &hmd_list);

    void request_start_hmd_stream(PSVRHmdID HmdID, int listIndex, PSVRTrackingColorType trackingColorType);
    void handle_start_hmd_response();

    void request_tracker_list();
    void handle_tracker_list_response(const PSVRTrackerList &tracker_list);

    void request_tracker_start_stream(const PSVRClientTrackerInfo *TrackerInfo);
    void handle_tracker_start_stream_response(const PSVRClientTrackerInfo *TrackerInfo);

    void handle_all_devices_ready();
    bool does_tracker_see_any_hmd(const PSVRTracker *trackerView);

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

protected:
    eMenuState m_menuState;

    PSVRTracker *m_trackerView;
    class TextureAsset *m_textureAsset;
    int m_pendingTrackerStartCount;

	t_hmd_state_map m_hmdViews;
	int m_pendingHmdStartCount;

    PSVRTrackerID m_overrideTrackerId;
    PSVRHmdID m_overrideHmdId;

	// Alignment Marker viability
	bool m_bShowAlignment;
	float m_AlignmentOffset;
};

#endif // APP_STAGE_COMPUTE_TRACKER_POSES_H