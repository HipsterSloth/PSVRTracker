#ifndef APP_STAGE_COMPUTE_TRACKER_POSES_H
#define APP_STAGE_COMPUTE_TRACKER_POSES_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <vector>

//-- definitions -----
class AppStage_ComputeTrackerPoses : public AppStage
{
public:
    struct TrackerState
    {
        PSVRTracker *trackerView;
        class TextureAsset *textureAsset;
    };
    typedef std::vector<TrackerState> t_tracker_state_list;
    typedef std::vector<TrackerState>::iterator t_tracker_state_list_iterator;
	typedef std::vector<TrackerState>::const_iterator t_tracker_state_list_iterator_const;

	struct ControllerState
	{
		PSVRTrackingColorType trackingColorType;
		PSVRController *controllerView;
	};
	typedef std::vector<ControllerState> t_controller_state_list;
	typedef std::vector<ControllerState>::iterator t_controller_state_list_iterator;

	struct HMDState
	{
		PSVRTrackingColorType trackingColorType;
		PSVRHeadMountedDisplay *hmdView;
	};
	typedef std::vector<HMDState> t_hmd_state_list;
	typedef std::vector<HMDState>::iterator t_hmd_state_list_iterator;

    AppStage_ComputeTrackerPoses(class App *app);
    ~AppStage_ComputeTrackerPoses();

    static void enterStageAndCalibrateTrackersWithController(class App *app, PSVRControllerID reqeusted_controller_id=-1);
    static void enterStageAndCalibrateTrackersWithHMD(class App *app, PSVRHmdID reqeusted_hmd_id=-1);
    static void enterStageAndTestTrackers(class App *app, PSVRControllerID reqeusted_controller_id=-1, PSVRHmdID requested_hmd_id=-1);

	inline void set_tracker_id(int reqeusted_tracker_id)
	{
		m_ShowTrackerVideoId = reqeusted_tracker_id;
	}

	inline const TrackerState *getRenderTrackerState() const
	{ 
		return 
			(m_renderTrackerListIndex >= 0 && m_renderTrackerListIndex < m_trackerViews.size()) 
			? &m_trackerViews[m_renderTrackerListIndex]
			: nullptr;
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

        failedControllerStartRequest,
        failedHmdStartRequest,
        failedTrackerStartRequest,

        verifyTrackers,
		selectCalibrationMethod,

        calibrateWithMat,
		calibrateWithHMD,

        testTracking,
		showTrackerVideo,
        calibrateStepFailed,
    };

	enum eCalibrationMethod
	{
		calibrationMethod_INVALID= -1,

		calibrationMethod_Mat,
		calibrationMethod_HMD
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
    bool request_start_controller_stream(PSVRControllerID ControllerID, PSVRTrackingColorType trackingColorType);
    void request_hmd_list();
    bool request_start_hmd_stream(PSVRHmdID HmdID, PSVRTrackingColorType trackingColorType);
    void request_tracker_list();
    bool request_tracker_start_stream(const PSVRClientTrackerInfo *TrackerInfo);
    void request_set_tracker_pose(const PSVRPosef *pose, PSVRTracker *TrackerView);

    void handle_all_devices_ready();
    bool does_tracker_see_any_device(const PSVRTracker *trackerView);
	bool does_tracker_see_any_controller(const PSVRTracker *trackerView);
    bool does_tracker_see_any_hmd(const PSVRTracker *trackerView);

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

protected:
    eMenuState m_menuState;

    t_tracker_state_list m_trackerViews;
	t_controller_state_list m_controllerViews;
	t_hmd_state_list m_hmdViews;

	int m_renderTrackerListIndex;

    class AppSubStage_CalibrateWithHMD *m_pCalibrateWithHMD;
    friend class AppSubStage_CalibrateWithHMD;

    class AppSubStage_CalibrateWithMat *m_pCalibrateWithMat;
    friend class AppSubStage_CalibrateWithMat;

	eCalibrationMethod m_calibrationMethod;

    bool m_bSkipCalibration;
    int m_ShowTrackerVideoId;
	PSVRControllerID m_overrideControllerId;
    PSVRHmdID m_overrideHmdId;

	// Alignment Marker visibility
	bool m_bShowAlignment;
	float m_AlignmentOffset;
};

#endif // APP_STAGE_COMPUTE_TRACKER_POSES_H