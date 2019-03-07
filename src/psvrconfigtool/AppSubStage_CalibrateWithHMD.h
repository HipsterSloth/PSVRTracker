#ifndef APP_STAGE_CALIBRATION_WITH_HMD_H
#define APP_STAGE_CALIBRATION_WITH_HMD_H

//-- includes -----
#include "ClientGeometry_CAPI.h"
#include "ClientConstants.h"
#include <chrono>
#include <vector>
#include <string.h>  // Required for memset in Xcode

//-- definitions -----
class AppSubStage_CalibrateWithHMD
{
public:
    enum eMenuState
    {
        invalid,

        initial,
        calibrationStepPlaceHMD,
        calibrationStepRecordHMD,
        calibrationStepComputeTrackerPoses,

        calibrateStepSuccess,
        calibrateStepFailed,
    };

    AppSubStage_CalibrateWithHMD(class AppStage_ComputeTrackerPoses *parentStage);
	virtual ~AppSubStage_CalibrateWithHMD();

    void enter();
    void exit();
    void update();
    void render();
    void renderUI();

	void drawCalibrationDebug(PSVRTrackerID tracker_id);

    inline eMenuState getMenuState() const
    {
        return m_menuState;
    }

protected:
    void setState(eMenuState newState);
    void onExitState(eMenuState newState);
    void onEnterState(eMenuState newState);

private:
    class AppStage_ComputeTrackerPoses *m_parentStage;
    eMenuState m_menuState;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_stableStartTime;
    bool m_bIsStable;
    bool m_bForceStable;

	std::vector<struct TrackerRelativeHMDPoseStatistics *> m_deviceTrackerPoseStats;

    int m_currentPoseStatsIndex;
    int m_sampleLocationIndex;
    bool m_bNeedMoreSamplesAtLocation;
};

#endif // APP_STAGE_CALIBRATION_WITH_HMD_H