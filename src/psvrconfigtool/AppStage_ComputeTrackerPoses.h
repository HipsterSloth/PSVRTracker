#ifndef APP_STAGE_COMPUTE_TRACKER_POSES_H
#define APP_STAGE_COMPUTE_TRACKER_POSES_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <map>

//-- definitions -----
class AppStage_ComputeTrackerPoses : public AppStage
{
public:
    struct TrackerState
    {
        int listIndex;
        PSVRTracker *trackerView;
        class TextureAsset *textureAsset;
    };
    typedef std::map<int, TrackerState> t_tracker_state_map;
    typedef std::map<int, TrackerState>::iterator t_tracker_state_map_iterator;
	typedef std::map<int, TrackerState>::const_iterator t_tracker_state_map_iterator_const;
    typedef std::pair<int, TrackerState> t_id_tracker_state_pair;

	struct ControllerState
	{
		int listIndex;
		PSVRTrackingColorType trackingColorType;
		PSVRController *controllerView;
	};
	typedef std::map<int, ControllerState> t_controller_state_map;
	typedef std::map<int, ControllerState>::iterator t_controller_state_map_iterator;
	typedef std::pair<int, ControllerState> t_id_controller_state_pair;

	struct HMDState
	{
		int listIndex;
		PSVRTrackingColorType trackingColorType;
		PSVRHeadMountedDisplay *hmdView;
	};
	typedef std::map<int, HMDState> t_hmd_state_map;
	typedef std::map<int, HMDState>::iterator t_hmd_state_map_iterator;
	typedef std::pair<int, HMDState> t_id_hmd_state_pair;

    AppStage_ComputeTrackerPoses(class App *app);
    ~AppStage_ComputeTrackerPoses();

    static void enterStageAndCalibrateTrackersWithController(class App *app, PSVRControllerID reqeusted_controller_id=-1);
    static void enterStageAndCalibrateTrackersWithHMD(class App *app, PSVRHmdID reqeusted_hmd_id=-1);
    static void enterStageAndTestTrackers(class App *app, PSVRControllerID reqeusted_controller_id=-1, PSVRHmdID requested_hmd_id=-1);

	inline void set_tracker_id(int reqeusted_tracker_id)
	{
		m_ShowTrackerVideoId = reqeusted_tracker_id;
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

        failedControllerListRequest,

        pendingControllerStartRequest,
        failedControllerStartRequest,

        failedHmdListRequest,

        pendingHmdStartRequest,
        failedHmdStartRequest,

        pendingTrackerListRequest,
        failedTrackerListRequest,

        pendingTrackerStartRequest,
        failedTrackerStartRequest,

        verifyTrackers,
		selectCalibrationMethod,
        calibrateWithMat,

        testTracking,
		showTrackerVideo,
        calibrateStepFailed,
    };

    void setState(eMenuState newState);
    void onExitState(eMenuState newState);
    void onEnterState(eMenuState newState);

    void update_tracker_video();
    void render_tracker_video();
    void go_next_tracker();
    void go_previous_tracker();
    int get_tracker_count() const;
    int get_render_tracker_index() const;
    PSVRTracker *get_render_tracker_view() const;
	PSVRController *get_calibration_controller_view() const;
    PSVRHeadMountedDisplay *get_calibration_hmd_view() const;

    void request_controller_list();
    void request_start_controller_stream(int ControllerID, int listIndex, PSVRTrackingColorType trackingColorType);
    void request_hmd_list();
    void request_start_hmd_stream(int HmdID, int listIndex, PSVRTrackingColorType trackingColorType);
    void request_tracker_list();
    void request_tracker_start_stream(const PSVRClientTrackerInfo *TrackerInfo, int listIndex);
    void request_set_tracker_pose(const PSVRPosef *pose, PSVRTracker *TrackerView);

    void handle_all_devices_ready();
    bool does_tracker_see_any_device(const PSVRTracker *trackerView);
	bool does_tracker_see_any_controller(const PSVRTracker *trackerView);
    bool does_tracker_see_any_hmd(const PSVRTracker *trackerView);

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

protected:
    eMenuState m_menuState;

    t_tracker_state_map m_trackerViews;
    int m_pendingTrackerStartCount;

	t_controller_state_map m_controllerViews;
	int m_pendingControllerStartCount;

	t_hmd_state_map m_hmdViews;
	int m_pendingHmdStartCount;

    int m_renderTrackerIndex;
    t_tracker_state_map_iterator m_renderTrackerIter;

    class AppSubStage_CalibrateWithHMD *m_pCalibrateWithHMD;
    friend class AppSubStage_CalibrateWithHMD;

    class AppSubStage_CalibrateWithMat *m_pCalibrateWithMat;
    friend class AppSubStage_CalibrateWithMat;

    bool m_bSkipCalibration;
    int m_ShowTrackerVideoId;
	PSVRControllerID m_overrideControllerId;
    PSVRHmdID m_overrideHmdId;

	// Alignment Marker visability
	bool m_bShowAlignment;
	float m_AlignmentOffset;
};

#endif // APP_STAGE_COMPUTE_TRACKER_POSES_H