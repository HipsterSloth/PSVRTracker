#ifndef SERVER_CONTROLLER_VIEW_H
#define SERVER_CONTROLLER_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"
#include "PSVRServiceInterface.h"

#include <cstring>
#include <mutex>
#include <atomic>

#include "readerwriterqueue.h" // lockfree queue

// -- pre-declarations -----
class TrackerManager;

struct PoseSensorPacket;
using t_controller_imu_sensor_queue= moodycamel::ReaderWriterQueue<PoseSensorPacket, 1024>;
using t_controller_optical_sensor_queue= moodycamel::ReaderWriterQueue<PoseSensorPacket, 16>;

template<typename t_object_type>
class AtomicObject;

struct ShapeTimestampedPose;

// -- declarations -----
struct ControllerOpticalPoseEstimation
{
    std::chrono::time_point<std::chrono::high_resolution_clock> last_update_timestamp;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_visible_timestamp;
    bool bValidTimestamps;

	PSVRVector3f tracker_relative_position_cm;
	PSVRVector3f world_relative_position_cm;
	PSVRTrackingProjection projection;
    PSVRTrackingShape tracker_relative_shape; // estimated shape from optical tracking
	PSVRTrackingShape world_relative_shape;
	bool bCurrentlyTracking;

	PSVRQuatf tracker_relative_orientation;
	PSVRQuatf world_relative_orientation;
	bool bOrientationValid;

    inline void clear()
    {
        last_update_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
        last_visible_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
        bValidTimestamps= false;

		tracker_relative_position_cm= *k_PSVR_float_vector3_zero;
		world_relative_position_cm= *k_PSVR_float_vector3_zero;
		bCurrentlyTracking = false;

		tracker_relative_orientation= *k_PSVR_quaternion_identity;
		world_relative_orientation= *k_PSVR_quaternion_identity;
		bOrientationValid = false;

        memset(&tracker_relative_shape, 0, sizeof(PSVRTrackingShape));
		memset(&world_relative_shape, 0, sizeof(PSVRTrackingShape));
		memset(&projection, 0, sizeof(PSVRTrackingProjection));
		projection.shape_type = PSVRShape_INVALID_PROJECTION;
    }
};

class ServerControllerView : public ServerDeviceView, public IControllerListener
{
public:
    ServerControllerView(const int device_id);
    virtual ~ServerControllerView();

    bool open(const class DeviceEnumerator *enumerator) override;
    void close() override;

	// Tell the pose filter that the controller is aligned with global forward 
	// with the given pose relative to it's identity pose.
	// Recenter the pose filter state accordingly.
	bool recenterOrientation(const PSVRQuatf& q_pose_relative_to_identity_pose);

    // Set the rumble value between 0.f-1.f on a channel
    bool setControllerRumble(float rumble_amount, PSVRControllerRumbleChannel channel);

	// Update Pose Filter using update packets from the tracker and IMU threads
	void updatePoseFilter();

	// Recreate and initialize the pose filter for the controller
	void resetPoseFilter();

    // Registers the address of the bluetooth adapter on the host PC with the controller
    bool setHostBluetoothAddress(const std::string &address);
    
    IDeviceInterface* getDevice() const override {return m_device;}
    inline class IPoseFilter * getPoseFilterMutable() { return m_pose_filter; }
    inline const class IPoseFilter * getPoseFilter() const { return m_pose_filter; }

    // Estimate the given pose if the controller at some point into the future
    PSVRPosef getFilteredPose(float time= 0.f) const;

    // Get the current physics from the filter position and orientation
    PSVRPhysicsData getFilteredPhysics() const;

    // Returns true if the device is connected via Bluetooth, false if by USB
    bool getIsBluetooth() const;

	// Returns true if the device uses PIN authentication during device pairing
	bool getUsesBluetoothAuthentication() const;

	// Returns true if the device can stream controller data over it's current connection type (Bluetooth/USB)
	bool getIsStreamable() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

	// Returns the vendor ID of the controller
	int getVendorID() const;

	// Returns the product ID of the controller
	int getProductID() const;

    // Returns the serial number for the controller
    std::string getSerial() const;

	// Returns the "controller_" + serial number for the controller
	std::string getConfigIdentifier() const;

    // Gets the host bluetooth address registered with the 
    std::string getAssignedHostBluetoothAddress() const;

    // Returns what type of controller this controller view represents
    CommonSensorState::eDeviceType getControllerDeviceType() const;

	// Get the last IMU sensor packet posted for this HMD
	const PoseSensorPacket *getLastIMUSensorPacket() const { return m_lastIMUSensorPacket; }

	// Get the last optical sensor packet posted for this HMD from the given tracker ID
    const PoseSensorPacket *getLastOpticalSensorPacket(int tracker_id) const;

    // Fetch the latest controller state from the HID processing thread
    const CommonControllerState * getControllerState() const;

	// Get the tracking is enabled on this controller
	inline bool getIsTrackingEnabled() const { return m_tracking_enabled.load(); }

    // Sets the bulb LED color to some new override color
    // If tracking was active this likely will affect controller tracking
    void setLEDOverride(unsigned char r, unsigned char g, unsigned char b);

    // Removes the over led color and restores the tracking color
    // of the controller is currently being tracked
    void clearLEDOverride();

    // Returns true 
    inline bool getIsLEDOverrideActive() const { return m_LED_override_active; }

    // Get the currently assigned tracking color ID for the controller
	PSVRTrackingColorType getTrackingColorID() const;

    // Set the assigned tracking color ID for the controller
    bool setTrackingColorID(PSVRTrackingColorType colorID);

    // Increment the position tracking listener count
    // Starts position tracking this controller if the count was zero
    void startTracking();

    // Decrements the position tracking listener count
    // Stop tracking this controller if this count becomes zero
    void stopTracking();

    // Get the tracking shape for the controller
    bool getTrackingShape(PSVRTrackingShape &outTrackingShape) const;

	// Get if the region-of-interest optimization is disabled for this controller
	inline bool getIsROIDisabled() const { return m_roi_disable_count > 0; }
	
	// Request the controller to not use the ROI optimization
	inline void pushDisableROI() { ++m_roi_disable_count; }

	// Undo the request to not use the ROI optimization
	inline void popDisableROI() { assert(m_roi_disable_count > 0); --m_roi_disable_count;  }

	// Get the prediction time used for ROI tracking
	float getROIPredictionTime() const;

	// Get the pose estimate relative to the given tracker id on the tracker worker thread
	inline const ControllerOpticalPoseEstimation *getOpticalPoseEstimateOnTrackerThread(int trackerId) const {
		return (m_optical_pose_estimations != nullptr) ? &m_optical_pose_estimations[trackerId] : nullptr;
	}

	// return true if one or more trackers saw this HMD last update
	inline bool getIsCurrentlyTracking() const {
		return getIsTrackingEnabled() ? m_currentlyTrackingBitmask.load() != 0 : false;
	}

	// return true if the given tracker saw this HMD last update
	inline bool getIsTrackedByTracker(int tracker_id) const {
		unsigned long tracker_bitmask= (1 << tracker_id);
		return getIsTrackingEnabled() ? (m_currentlyTrackingBitmask.load() & tracker_bitmask) > 0 : false;
	}

	// Incoming device data callbacks
	void notifyTrackerDataReceived(class ServerTrackerView* tracker);
	void notifySensorDataReceived(const CommonSensorState *sensor_state) override;

protected:
    void set_tracking_enabled_internal(bool bEnabled);
    void update_LED_color_internal();
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_controller_data_frame_for_stream(
        const ServerControllerView *controller_view,
        const struct ControllerStreamInfo *stream_info,
        DeviceOutputDataFrame &data_frame);

private:
    // Tracking color state
    std::tuple<unsigned char, unsigned char, unsigned char> m_tracking_color;
    int m_tracking_listener_count;
    std::atomic_bool m_tracking_enabled;
    
	// Region-of-Interest state
	int m_roi_disable_count;

	// Device State
    IControllerInterface *m_device;
    
    // Override color state
    std::tuple<unsigned char, unsigned char, unsigned char> m_LED_override_color;
    bool m_LED_override_active;
   
	// Filter State (Tracker Thread)
    class IShapeTrackingModel **m_shape_tracking_models;  // array of size TrackerManager::k_max_devices
	struct ControllerOpticalPoseEstimation *m_optical_pose_estimations; // array of size TrackerManager::k_max_devices

	// Filter State (IMU Thread)
	std::chrono::time_point<std::chrono::high_resolution_clock> m_lastSensorDataTimestamp;
	bool m_bIsLastSensorDataTimestampValid;

	// Filter State (Shared)
	t_controller_imu_sensor_queue m_PoseSensorIMUPacketQueue;
	t_controller_optical_sensor_queue *m_PoseSensorOpticalPacketQueues; // array of size TrackerManager::k_max_devices
	AtomicObject<ShapeTimestampedPose> *m_sharedFilteredPose;
	std::atomic_ulong m_currentlyTrackingBitmask;

	// Filter State (Main Thread)
	struct PoseSensorPacket *m_lastIMUSensorPacket;
	struct PoseSensorPacket *m_lastOpticalSensorPacket; // array of size TrackerManager::k_max_devices
	class IPoseFilter *m_pose_filter;
	class PoseFilterSpace *m_pose_filter_space;
    int m_lastPollSeqNumProcessed;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFilterUpdateTimestamp;
	bool m_bIsLastFilterUpdateTimestampValid;
};

#endif // SERVER_CONTROLLER_VIEW_H
