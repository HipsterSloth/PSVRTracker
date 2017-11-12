#ifndef APP_STAGE_HMD_MODEL_CALIBRATION_H
#define APP_STAGE_HMD_MODEL_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSVRClient_CAPI.h"

#include <deque>
#include <chrono>
#include <string>

//-- definitions -----
class AppStage_HMDModelCalibration : public AppStage
{
public:
	enum eMenuState
	{
		inactive,

		pendingHmdListRequest,
		failedHmdListRequest,

		pendingHmdShapeRequest,
		failedHmdShapeRequest,

		pendingHmdStartRequest,
		failedHmdStartRequest,

		pendingTrackerListRequest,
		failedTrackerListRequest,

		pendingTrackerStartRequest,
		failedTrackerStartRequest,

		verifyTrackers,
		calibrate,
		test
	};

	enum eTrackingTestRenderMode
	{
        renderMode2d,
        renderMode3d
	};

	AppStage_HMDModelCalibration(class App *app);
	virtual ~AppStage_HMDModelCalibration();

	static void enterStageAndCalibrate(App *app, int reqeusted_hmd_id);
	static void enterStageAndSkipCalibration(App *app, int reqeusted_hmd_id);

	virtual void enter() override;
	virtual void exit() override;
	virtual void update() override;
	virtual void render() override;

	virtual void renderUI() override;

	static const char *APP_STAGE_NAME;

	inline void setBypassCalibrationFlag(bool bFlag)
	{
		m_bBypassCalibration = bFlag;
	}

protected:
	void setState(eMenuState newState);
	void onExitState(eMenuState newState);
	void onEnterState(eMenuState newState);

	void update_tracker_video();
	void render_tracker_video(const float top_y, const float bottom_y);

	void request_hmd_list();
	static void handle_hmd_list_response(
		const PSVRResponseMessage *response,
		void *userdata);

	void request_hmd_tracking_shape(PSVRHmdID HmdID);
	static void handle_hmd_tracking_shape_response(
		const PSVRResponseMessage *response,
		void *userdata);

	void request_start_hmd_stream(PSVRHmdID HmdID);
	static void handle_start_hmd_response(
		const PSVRResponseMessage *response_message,
		void *userdata);

	void request_tracker_list();
	static void handle_tracker_list_response(
		const PSVRResponseMessage *response_message,
		void *userdata);
	bool setup_stereo_tracker(const PSVRTrackerList &tracker_list);

	void request_tracker_start_stream(PSVRTracker *tracker_view);
	static void handle_tracker_start_stream_response(
		const PSVRResponseMessage *response,
		void *userdata);

	void request_set_hmd_led_model_calibration();

	void handle_all_devices_ready();

	void release_devices();
	void request_exit_to_app_stage(const char *app_stage_name);

private:
	eMenuState m_menuState;
    eTrackingTestRenderMode m_testRenderMode;
	bool m_bBypassCalibration;

	struct StereoCameraState *m_stereoTrackerState;
	class HMDModelState *m_hmdModelState;

	PSVRHeadMountedDisplay *m_hmdView;
	int m_overrideHmdId;

	std::string m_failureDetails;
};

#endif // APP_STAGE_HMD_MODEL_CALIBRATION_H
