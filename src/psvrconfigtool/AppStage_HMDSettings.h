#ifndef APP_STAGE_HMD_SETTINGS_H
#define APP_STAGE_HMD_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include <vector>
#include <string>

//-- definitions -----
class AppStage_HMDSettings : public AppStage
{
public:
    struct HMDInfo
    {
        PSVRClientHMDInfo hmd_info;
		int PositionFilterIndex;
		int OrientationFilterIndex;
    };

    AppStage_HMDSettings(class App *app);

    inline const PSVRClientHMDInfo *getSelectedHmdInfo() const
    {
        return
            (m_selectedHmdIndex != -1)
            ? &m_hmdInfos[m_selectedHmdIndex].hmd_info
            : nullptr;
    }

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    virtual bool onClientAPIEvent(PSVREventType event_type) override;

    void request_hmd_list();
	void handle_hmd_list_response(const PSVRHmdList &hmd_list);

	void request_set_orientation_filter(const int hmd_id, const std::string &filter_name);
	void request_set_position_filter(const int hmd_id, const std::string &filter_name);
	void request_set_hmd_prediction(const int hmd_id, float prediction_time);
	void request_set_hmd_tracking_color_id(
		const int hmd_id,
		PSVRTrackingColorType tracking_color_type);


private:
    enum eHmdMenuState
    {
        inactive,
        idle,

        pendingHmdListRequest,
        failedHmdListRequest,
    };
    eHmdMenuState m_menuState;

    std::vector<HMDInfo> m_hmdInfos;

    int m_selectedHmdIndex;
};

#endif // APP_STAGE_HMD_SETTINGS_H
