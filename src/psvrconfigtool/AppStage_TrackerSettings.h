#ifndef APP_STAGE_TRACKER_SETTINGS_H
#define APP_STAGE_TRACKER_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <vector>

//-- definitions -----
class AppStage_TrackerSettings : public AppStage
{
public:
    AppStage_TrackerSettings(class App *app);

	void setSelectedControllerIndex(int index);
	void setSelectedTrackerIndex(int index);
	void setSelectedHmdIndex(int index);

	const PSVRClientControllerInfo *getSelectedController() const;
    const PSVRClientTrackerInfo *getSelectedTracker() const;
	const PSVRClientHMDInfo *getSelectedHmd() const;

	int getControllerCount() const;
	int getTrackerCount() const;
	int getHmdCount() const;

	int getSelectedControllerIndex() const;
	int getSelectedTrackerIndex() const;
	int getSelectedHmdIndex() const;

	const PSVRClientControllerInfo *getControllerInfo(int index) const;
	const PSVRClientTrackerInfo *getTrackerInfo(int index) const;
	const PSVRClientHMDInfo *getHmdInfo(int index) const;

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

	void gotoControllerColorCalib(bool value = false) { m_gotoControllerColorCalib = value; }
    void gotoHMDColorCalib(bool value = false) { m_gotoHMDColorCalib = value; }
	void gotoTestControllerTracking(bool value = false) { m_gotoTestControllerTracking = value; }
	void gotoTrackingControllerVideo(bool value = false) { m_gotoTrackingControllerVideo = value; }
	void gotoTestHMDTracking(bool value = false) { m_gotoTestHmdTracking = value; }
    void gotoTrackingHMDVideo(bool value = false) { m_gotoTrackingHmdVideo = value; }
	void gotoTrackingVideoALL(bool value = false) { m_gotoTrackingVideoALL = value; }

protected:
    virtual bool onClientAPIEvent(PSVREventType event_type) override;

    void request_tracker_list();
    void handle_tracker_list_response(const PSVRTrackerList &tracker_list);

    void request_controller_list();
    void handle_controller_list_response(const PSVRControllerList &controller_list);

	void request_hmd_list();
	void handle_hmd_list_response(const PSVRHmdList &hmd_list);

protected:
    enum eTrackerMenuState
    {
        inactive,
        idle,

        pendingTrackerListRequest,
        failedTrackerListRequest,
		pendingControllerListRequest,
		failedControllerListRequest,
		pendingHmdListRequest,
		failedHmdListRequest,
        pendingSearchForNewTrackersRequest,
    };
    eTrackerMenuState m_menuState;

    PSVRControllerList m_controllerInfos;
	PSVRTrackerList m_trackerInfos;
	PSVRHmdList m_hmdInfos;

    int m_selectedTrackerIndex;
	int m_selectedControllerIndex;
	int m_selectedHmdIndex;

	bool m_gotoControllerColorCalib;
    bool m_gotoHMDColorCalib;
	bool m_gotoTestControllerTracking;
	bool m_gotoTrackingControllerVideo;
	bool m_gotoTestHmdTracking;
    bool m_gotoTrackingHmdVideo;
	bool m_gotoTrackingVideoALL;
};

#endif // APP_STAGE_TRACKER_SETTINGS_H