#ifndef APP_STAGE_HMD_TRACKING_TEST_H
#define APP_STAGE_HMD_TRACKING_TEST_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <map>
#include <vector>

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

    struct TrackerState
    {
        PSVRTracker *trackerView;
        class TextureAsset *textureAsset[2];
        bool bIsStereoTracker;
    };
    typedef std::vector<TrackerState> t_tracker_state_list;

    AppStage_HMDTrackingTest(class App *app);

    static void enterStageAndTestTracking(class App *app, PSVRHmdID requested_hmd_id=-1);

    inline void set_tracker_id(int reqeusted_tracker_id)
    {
        m_ShowTrackerVideoId = reqeusted_tracker_id;
    }

    inline const TrackerState *getRenderTrackerState() const
    {
        return
            (m_renderTrackerListIndex >= 0 && m_renderTrackerListIndex < m_trackerList.size())
            ? &m_trackerList[m_renderTrackerListIndex]
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

        failedHmdListRequest,
        failedHmdStartRequest,
        failedTrackerListRequest,
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
    PSVRHeadMountedDisplay *get_calibration_hmd_view() const;

    bool setup_hmd();
    bool start_hmd_stream(PSVRHmdID HmdID, int listIndex, PSVRTrackingColorType trackingColorType);

    bool setup_trackers();
    bool start_tracker_stream(const PSVRClientTrackerInfo *TrackerInfo);

    bool does_tracker_see_any_hmd(const PSVRTracker *trackerView);

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

protected:
    eMenuState m_menuState;

    t_tracker_state_list m_trackerList;

	t_hmd_state_map m_hmdViews;
	int m_pendingHmdStartCount;

    int m_renderTrackerListIndex;
    int m_ShowTrackerVideoId;

    PSVRHmdID m_overrideHmdId;

	// Alignment Marker viability
	bool m_bShowAlignment;
	float m_AlignmentOffset;
};

#endif // APP_STAGE_HMD_TRACKING_TEST_H