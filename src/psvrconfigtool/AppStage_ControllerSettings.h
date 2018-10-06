#ifndef APP_STAGE_CONTROLLER_SETTINGS_H
#define APP_STAGE_CONTROLLER_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include "PSVRClient_CAPI.h"

#include <vector>
#include <string>

//-- definitions -----
class AppStage_ControllerSettings : public AppStage
{
public:
    struct ControllerInfo
    {
		PSVRClientControllerInfo controller;
		int PositionFilterIndex;
		std::string PositionFilterName;
		int OrientationFilterIndex;
		std::string OrientationFilterName;
		int GyroGainIndex;
		std::string GyroGainSetting;
    };


    AppStage_ControllerSettings(class App *app);

    inline const ControllerInfo *getSelectedControllerInfo() const
    { 
        return 
            (m_selectedControllerIndex != -1) 
            ? &m_controllerInfos[m_selectedControllerIndex] 
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

    void rebuild_controller_list();

private:
    enum eControllerMenuState
    {
        inactive,
        idle,
        failedControllerListRequest,
    };
    eControllerMenuState m_menuState;

    std::vector<ControllerInfo> m_controllerInfos;
    std::string m_hostSerial;
    int m_selectedControllerIndex;
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
