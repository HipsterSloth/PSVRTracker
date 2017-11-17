#ifndef APP_STAGE_HMD_ACCELEROMETER_CALIBRATION_H
#define APP_STAGE_HMD_ACCELEROMETER_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSVRClient_CAPI.h"

#include <deque>
#include <chrono>

//-- definitions -----
class AppStage_HMDAccelerometerTest : public AppStage
{
public:
	AppStage_HMDAccelerometerTest(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    void request_exit_to_app_stage(const char *app_stage_name);

private:
    enum eCalibrationMenuState
    {
        inactive,

        waitingForStreamStartResponse,
        failedStreamStart,
        test
    };
    eCalibrationMenuState m_menuState;

    PSVRHeadMountedDisplay *m_hmdView;
    bool m_isHMDStreamActive;
    int m_lastHMDSeqNum;

    PSVRVector3i m_lastRawAccelerometer;
    PSVRVector3f m_lastCalibratedAccelerometer;
};

#endif // APP_STAGE_HMD_ACCELEROMETER_CALIBRATION_H