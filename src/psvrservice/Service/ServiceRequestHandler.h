#ifndef SERVICE_REQUEST_HANDLER_H
#define SERVICE_REQUEST_HANDLER_H

// -- includes -----
#include "PSVRClient_CAPI.h"
#include "PSVRServiceInterface.h"
#include <bitset>

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

struct PersistentRequestConnectionState
{
	std::bitset<PSVRSERVICE_MAX_CONTROLLER_COUNT> active_controller_streams;
    std::bitset<PSVRSERVICE_MAX_TRACKER_COUNT> active_tracker_streams;
    std::bitset<PSVRSERVICE_MAX_HMD_COUNT> active_hmd_streams;
	ControllerStreamInfo  active_controller_stream_info[PSVRSERVICE_MAX_CONTROLLER_COUNT];
    TrackerStreamInfo active_tracker_stream_info[PSVRSERVICE_MAX_TRACKER_COUNT];
    HMDStreamInfo active_hmd_stream_info[PSVRSERVICE_MAX_HMD_COUNT];

    PersistentRequestConnectionState()
        : active_controller_streams()
		, active_tracker_streams()
        , active_hmd_streams()
    {
        for (int index = 0; index < PSVRSERVICE_MAX_CONTROLLER_COUNT; ++index)
        {
            active_controller_stream_info[index].Clear();
        }

        for (int index = 0; index < PSVRSERVICE_MAX_TRACKER_COUNT; ++index)
        {
            active_tracker_stream_info[index].Clear();
        }
        
        for (int index = 0; index < PSVRSERVICE_MAX_HMD_COUNT; ++index)
        {
            active_hmd_stream_info[index].Clear();
        }
    }
};

class ServiceRequestHandler 
{
public:
    ServiceRequestHandler();
    virtual ~ServiceRequestHandler();

    static ServiceRequestHandler *get_instance() { return m_instance; }

    bool startup(
        class DeviceManager *deviceManager,
        class IDataFrameListener *data_frame_listener, 
        class INotificationListener *notification_listener);
    void shutdown();

    /// When publishing controller data to all listening connections
    /// we need to provide a callback that will fill out a data frame given:
    /// * A \ref ServerControllerView we want to publish to all listening connections
    /// * A \ref ControllerStreamInfo that describes what info the connection wants
    /// This callback will be called for each listening connection
    typedef void(*t_generate_controller_data_frame_for_stream)(
        const class ServerControllerView *controller_view,
        const ControllerStreamInfo *stream_info,
        DeviceOutputDataFrame &data_frame);
    void publish_controller_data_frame(
        class ServerControllerView *controller_view, t_generate_controller_data_frame_for_stream callback);

    /// When publishing tracker data to all listening connections
    /// we need to provide a callback that will fill out a data frame given:
    /// * A \ref ServerTrackerView we want to publish to all listening connections
    /// * A \ref TrackerStreamInfo that describes what info the connection wants
    /// This callback will be called for each listening connection
    typedef void(*t_generate_tracker_data_frame_for_stream)(
        const class ServerTrackerView *tracker_view,
        const TrackerStreamInfo *stream_info,
        DeviceOutputDataFrame &data_frame);
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
        DeviceOutputDataFrame &data_frame);
    void publish_hmd_data_frame(
        class ServerHMDView *hmd_view, t_generate_hmd_data_frame_for_stream callback);

    /// Send a event to the client
    void publish_notification(const PSVREventMessage &message);

	// -- controller requests -----
    PSVRResult get_controller_list(PSVRControllerList *out_controller_list);
    PSVRResult start_controller_data_stream(PSVRControllerID controller_id, unsigned int flags);
    PSVRResult stop_controller_data_stream(PSVRControllerID controller_id);
    PSVRResult set_led_tracking_color(PSVRControllerID controller_id, PSVRTrackingColorType tracking_color);
    PSVRResult reset_orientation(PSVRControllerID controller_id, const PSVRQuatf& q_pose);
    PSVRResult set_controller_data_stream_tracker_index(PSVRControllerID controller_id, PSVRTrackerID tracker_id);
	PSVRResult set_controller_hand(PSVRControllerID controller_id, PSVRControllerHand controller_hand);

    // -- tracker requests -----
    PSVRResult get_tracker_list(PSVRTrackerList *out_tracker_list);
    PSVRResult start_tracker_data_stream(PSVRTrackerID tracker_id);
    PSVRResult stop_tracker_data_stream(PSVRTrackerID tracker_id);
	PSVRResult get_shared_video_frame_buffer(PSVRTrackerID tracker_id, const SharedVideoFrameBuffer **out_shared_buffer);
    PSVRResult get_tracker_settings(
		PSVRTrackerID tracker_id, PSVRHmdID hmd_id, 
		PSVRClientTrackerSettings *out_settings);
	PSVRResult get_tracker_mode(const PSVRTrackerID tracker_id, std::string &out_mode);
    PSVRResult set_tracker_mode(const PSVRTrackerID tracker_id, const std::string &new_mode);
    PSVRResult set_tracker_video_property(
		const PSVRTrackerID tracker_id, const PSVRVideoPropertyType property_type, int desired_value, bool save_setting, 
		int *out_value);
    PSVRResult set_tracker_color_preset(
        const PSVRTrackerID tracker_id, const PSVRHmdID HmdID, 
        const PSVRTrackingColorType tracking_color_type,
        const PSVR_HSVColorRange &desired_color_filter, PSVR_HSVColorRange &out_color_filter);
    PSVRResult set_tracker_pose(const PSVRTrackerID tracker_id, const PSVRPosef *pose);
    PSVRResult set_tracker_intrinsics(const PSVRTrackerID tracker_id, const PSVRTrackerIntrinsics *tracker_intrinsics);
    PSVRResult get_tracking_space_settings(PSVRTrackingSpace *out_tracking_space);
    PSVRResult reload_tracker_settings(const PSVRTrackerID tracker_id);
	PSVRResult get_tracker_debug_flags(PSVRTrackerDebugFlags *out_flags) const;
	PSVRResult set_tracker_debug_flags(PSVRTrackerDebugFlags flags);
	
    // -- hmd requests -----
    ServerHMDView *get_hmd_view_or_null(PSVRHmdID hmd_id);
    PSVRResult get_hmd_list(PSVRHmdList *out_hmd_list);
	PSVRResult get_hmd_tracking_shape(PSVRHmdID hmd_id, PSVRTrackingShape *out_shape);
    PSVRResult start_hmd_data_stream(const PSVRHmdID hmd_id, unsigned int data_stream_flags);
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
    // Keeps track of all active stream state
    PersistentRequestConnectionState *m_peristentRequestState;

    class DeviceManager *m_deviceManager;
    class IDataFrameListener *m_dataFrameListener;
    class INotificationListener *m_notificationListener;

    // Singleton instance of the class
    // Assigned in startup, cleared in teardown
    static ServiceRequestHandler *m_instance;
};

#endif  // SERVICE_REQUEST_HANDLER_H
