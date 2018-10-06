//-- includes -----
#include "App.h"
#include "AppStage_AccelerometerCalibration.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_ColorCalibration.h"
#include "AppStage_HMDAccelerometerTest.h"
#include "AppStage_HMDGyroscopeTest.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_HMDTrackingTest.h"
#include "AppStage_MainMenu.h"
#include "AppStage_MonoCalibration.h"
#include "AppStage_StereoCalibration.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_TrackerTest.h"

#ifdef _WIN32
#pragma comment (lib, "winmm.lib")     /* link with Windows MultiMedia lib */
#pragma comment (lib, "opengl32.lib")  /* link with Microsoft OpenGL lib */
#pragma comment (lib, "glu32.lib")     /* link with OpenGL Utility lib */

#pragma warning (disable:4244)	/* Disable bogus conversion warnings. */
#pragma warning (disable:4305)  /* VC++ 5.0 version of above warning. */
#endif

//-- entry point -----
extern "C" int main(int argc, char *argv[])
{
    App app;

    // Register all of the app stages
	app.registerAppStage<AppStage_AccelerometerCalibration>();
	app.registerAppStage<AppStage_ControllerSettings>();
	app.registerAppStage<AppStage_ColorCalibration>();
	app.registerAppStage<AppStage_HMDAccelerometerTest>();
	app.registerAppStage<AppStage_HMDGyroscopeTest>();
    app.registerAppStage<AppStage_HMDSettings>();
    app.registerAppStage<AppStage_HMDTrackingTest>();
    app.registerAppStage<AppStage_MainMenu>();
	app.registerAppStage<AppStage_MonoCalibration>();
    app.registerAppStage<AppStage_StereoCalibration>();
    app.registerAppStage<AppStage_TrackerTest>();
    app.registerAppStage<AppStage_TrackerSettings>();

    return app.exec(argc, argv, AppStage_MainMenu::APP_STAGE_NAME);
}
