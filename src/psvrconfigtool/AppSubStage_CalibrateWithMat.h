#ifndef APP_STAGE_CALIBRATE_WITH_MAT_H
#define APP_STAGE_CALIBRATE_WITH_MAT_H

//-- includes -----
#include "ClientGeometry_CAPI.h"
#include "ClientConstants.h"
#include <chrono>
#include <vector>
#include <string.h>  // Required for memset in Xcode

//-- definitions -----
class AppSubStage_CalibrateWithMat
{
public:
    enum eMenuState
    {
        invalid,

        initial,
        calibrationStepPlaceController,
        calibrationStepRecordController,
        calibrationStepComputeTrackerPoses,

        calibrateStepSuccess,
        calibrateStepFailed,
    };

    AppSubStage_CalibrateWithMat(class AppStage_ComputeTrackerPoses *parentStage);
	virtual ~AppSubStage_CalibrateWithMat();

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

	std::vector<struct TrackerRelativeControllerPoseStatistics *> m_deviceTrackerPoseStats;

    int m_currentPoseStatsIndex;
    int m_sampleLocationIndex;
    bool m_bNeedMoreSamplesAtLocation;
};

#endif // APP_STAGE_CALIBRATE_WITH_MAT_H