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

    const PSVRClientTrackerInfo *getSelectedTrackerInfo() const;
	void set_selectedTrackerIndex(int index);

	int get_tracker_count() const;
	int get_tracker_Index() const;

	const PSVRClientHMDInfo *get_selected_hmd();

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    virtual bool onClientAPIEvent(PSVREventType event_type) override;

    void request_tracker_list();
    void handle_tracker_list_response(const PSVRTrackerList &tracker_list);

	void request_hmd_list();
	void handle_hmd_list_response(const PSVRHmdList &hmd_list);

protected:
    enum eTrackerMenuState
    {
        inactive,
        idle,

        pendingTrackerListRequest,
        failedTrackerListRequest,
		pendingHmdListRequest,
		failedHmdListRequest,
        pendingSearchForNewTrackersRequest,
    };
    eTrackerMenuState m_menuState;

    PSVRTrackerList m_trackerInfos;
	PSVRHmdList m_hmdInfos;

    int m_selectedTrackerIndex;
	int m_selectedHmdIndex;
};

#endif // APP_STAGE_TRACKER_SETTINGS_H