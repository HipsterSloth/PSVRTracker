#ifndef APP_STAGE_HMD_GYROSCOPE_CALIBRATION_H
#define APP_STAGE_HMD_GYROSCOPE_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSVRClient_CAPI.h"

#include <deque>
#include <chrono>

//-- definitions -----
class AppStage_HMDGyroscopeTest : public AppStage
{
public:
    AppStage_HMDGyroscopeTest(class App *app);

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
        measureComplete,
        test
    };
    eCalibrationMenuState m_menuState;
    bool m_bBypassCalibration;

    PSVRHeadMountedDisplay *m_hmdView;
    bool m_isHMDStreamActive;
    int m_lastHMDSeqNum;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastSampleTime;
    bool m_bLastSampleTimeValid;

    PSVRVector3i m_lastRawGyroscope;
    PSVRVector3f m_lastCalibratedGyroscope;
    PSVRVector3f m_lastCalibratedAccelerometer;
};

#endif // APP_STAGE_HMD_GYROSCOPE_CALIBRATION_H