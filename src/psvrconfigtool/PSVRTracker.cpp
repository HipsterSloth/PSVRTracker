//-- includes -----
#include "App.h"
#include "AppStage_ColorCalibration.h"
#include "AppStage_HMDAccelerometerCalibration.h"
#include "AppStage_HMDGyroscopeCalibration.h"
#include "AppStage_HMDModelCalibration.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "AppStage_StereoCalibration.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_TestTracker.h"

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
	app.registerAppStage<AppStage_ColorCalibration>();
	app.registerAppStage<AppStage_HMDAccelerometerCalibration>();
	app.registerAppStage<AppStage_HMDGyroscopeCalibration>();
	app.registerAppStage<AppStage_HMDModelCalibration>();
    app.registerAppStage<AppStage_HMDSettings>();
    app.registerAppStage<AppStage_MainMenu>();
    app.registerAppStage<AppStage_StereoCalibration>();
    app.registerAppStage<AppStage_TestTracker>();
    app.registerAppStage<AppStage_TrackerSettings>();

    return app.exec(argc, argv, AppStage_MainMenu::APP_STAGE_NAME);
}
