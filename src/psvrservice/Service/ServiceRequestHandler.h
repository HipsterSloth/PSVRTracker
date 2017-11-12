#ifndef SERVICE_REQUEST_HANDLER_H
#define SERVICE_REQUEST_HANDLER_H

// -- includes -----
#include "PSVRClient_CAPI.h"

// -- pre-declarations -----
class DeviceManager;

// -- definitions -----
struct ControllerStreamInfo
{
    bool include_position_data;
    bool include_physics_data;
    bool include_raw_sensor_data;
    bool include_calibrated_sensor_data;
    bool include_raw_tracker_data;
    bool led_override_active;
	bool disable_roi;
    int last_data_input_sequence_number;
    int selected_tracker_index;

    inline void Clear()
    {
        include_position_data = false;
        include_physics_data = false;
        include_raw_sensor_data = false;
        include_calibrated_sensor_data= false;
        include_raw_tracker_data = false;
        led_override_active = false;
		disable_roi = false;
		last_data_input_sequence_number = -1;
        selected_tracker_index = 0;
    }
};

struct TrackerStreamInfo
{
    bool streaming_video_data;
	bool has_temp_settings_override;

    inline void Clear()
    {
        streaming_video_data = false;
		has_temp_settings_override = false;
    }
};

struct HMDStreamInfo
{
	bool include_position_data;
	bool include_physics_data;
	bool include_raw_sensor_data;
	bool include_calibrated_sensor_data;
	bool include_raw_tracker_data;
	bool disable_roi;
    int selected_tracker_index;

    inline void Clear()
    {
		include_position_data = false;
		include_physics_data = false;
		include_raw_sensor_data = false;
		include_calibrated_sensor_data = false;
		include_raw_tracker_data = false;
		disable_roi = false;
        selected_tracker_index = 0;
    }
};

class ServerRequestHandler 
{
public:
    ServerRequestHandler(DeviceManager *deviceManager);
    virtual ~ServerRequestHandler();

    static ServerRequestHandler *get_instance() { return m_instance; }

    bool startup();
    void update();
    void shutdown();

    /// When publishing tracker data to all listening connections
    /// we need to provide a callback that will fill out a data frame given:
    /// * A \ref ServerTrackerView we want to publish to all listening connections
    /// * A \ref TrackerStreamInfo that describes what info the connection wants
    /// This callback will be called for each listening connection
    typedef void(*t_generate_tracker_data_frame_for_stream)(
        const class ServerTrackerView *tracker_view,
        const TrackerStreamInfo *stream_info,
        DeviceOutputDataFramePtr &data_frame);
    void publish_tracker_data_frame(
        class ServerTrackerView *tracker_view, t_generate_tracker_data_frame_for_stream callback);
        
    /// When publishing hmd data to all listening connections
    /// we need to provide a callback that will fill out a data frame given:
    /// * A \ref ServerHMDView we want to publish to all listening connections
    /// * A \ref HMDStreamInfo that describes what info the connection wants
    /// This callback will be called for each listening connection
    typedef void(*t_generate_hmd_data_frame_for_stream)(
        const class ServerHMDView *hmd_view,
        const HMDStreamInfo *stream_info,
        DeviceOutputDataFramePtr &data_frame);
    void publish_hmd_data_frame(
        class ServerHMDView *hmd_view, t_generate_hmd_data_frame_for_stream callback);

    // -- tracker requests -----
    PSVRResult get_tracker_list(PSVRTrackerList *out_tracker_list);
    PSVRResult start_tracker_data_stream(PSVRTrackerID tracker_id);
    PSVRResult stop_tracker_data_stream(PSVRTrackerID tracker_id);
	PSVRResult get_shared_video_frame_buffer(PSVRTrackerID tracker_id, SharedVideoFrameBuffer **out_shared_buffer);
    PSVRResult get_tracker_settings(
		PSVRTrackerID tracker_id, PSVRHmdID hmd_id, 
		PSVRClientTrackerSettings *out_settings)
    PSVRResult set_tracker_frame_width(
		const PSVRTrackerID tracker_id, const float desired_frame_width, const bool bSaveSetting,
		float *out_result_frame_width);
    PSVRResult set_tracker_frame_height(
		const PSVRTrackerID tracker_id, const float desired_frame_height, const bool bSaveSetting,
		float *out_result_frame_height);
    PSVRResult set_tracker_frame_rate(
		const PSVRTrackerID tracker_id, const float desired_frame_rate, const bool bSaveSetting,
		float *out_result_frame_rate);
    PSVRResult set_tracker_exposure(
		const PSVRTrackerID tracker_id, const float desired_exposure, const bool bSaveSetting,
		float *out_result_exposure);
    PSVRResult set_tracker_gain(
		const PSVRTrackerID tracker_id, const float desired_gain, const bool bSaveSetting,
		float *out_result_gain);
    PSVRResult set_tracker_option(
		const PSVRTrackerID tracker_id, const std::string &option_name, const int desired_option_index, 
		int *out_new_option_index);
    PSVRResult handle_request__set_tracker_color_preset(
        const PSVRTrackerID tracker_id, const PSVRHmdID hmd_id,
		const PSVRTrackingColorType color_type,
		const PSVR_HSVColorRange *desired_range, PSVR_HSVColorRange *out_result_range);
    PSVRResult set_tracker_pose(const PSVRTrackerID tracker_id, const PSVRPosef *pose);
    PSVRResult set_tracker_intrinsics(const PSVRTrackerID tracker_id, const PSVRTrackerIntrinsics *tracker_intrinsics);
    PSVRResult reload_tracker_settings(const PSVRTrackerID tracker_id);
    PSVRResult get_tracking_space_settings(PSVRTrackingSpace *out_tracking_space);
	
    // -- hmd requests -----
    PSVRResult get_hmd_list(PSVRHmdList *out_hmd_list);
	PSVRResult get_hmd_tracking_shape(PSVRHmdID hmd_id, PSVRTrackingShape *out_shape);
    PSVRResult start_hmd_data_stream(const PSVRHmdID hmd_id);
    PSVRResult stop_hmd_data_stream(const PSVRHmdID hmd_id);
    PSVRResult set_hmd_led_tracking_color(const PSVRHmdID hmd_id, const PSVRTrackingColorType new_color_id);
    PSVRResult set_hmd_accelerometer_calibration(
        const PSVRHmdID hmd_id, const PSVRVector3f &measured_g, const float raw_variance);
    PSVRResult set_hmd_gyroscope_calibration(
        const PSVRHmdID hmd_id, const PSVRVector3f &raw_gyro_bias, const float raw_variance, const float raw_drift);
    PSVRResult set_hmd_orientation_filter(const PSVRHmdID hmd_id, const std::string orientation_filter);
    PSVRResult set_hmd_position_filter(const PSVRHmdID hmd_id, const std::string position_filter);
    PSVRResult set_hmd_prediction_time(const PSVRHmdID hmd_id, const float hmd_prediction_time);
    PSVRResult set_hmd_data_stream_tracker_index(const PSVRTrackerID tracker_id, const PSVRHmdID hmd_id);
	
	// -- general requests -----
    PSVRResult get_service_version(char *out_version_string, size_t max_version_string);		

private:
    // private implementation - same lifetime as the ServerRequestHandler
    class ServerRequestHandlerImpl *m_implementation_ptr;

    // Singleton instance of the class
    // Assigned in startup, cleared in teardown
    static ServerRequestHandler *m_instance;
};

#endif  // SERVICE_REQUEST_HANDLER_H
