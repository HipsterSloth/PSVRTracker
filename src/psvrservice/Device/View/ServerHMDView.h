#ifndef SERVER_HMD_VIEW_H
#define SERVER_HMD_VIEW_H

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
using t_hmd_pose_sensor_queue= moodycamel::ReaderWriterQueue<PoseSensorPacket, 1024>;

// -- declarations -----
struct HMDOpticalPoseEstimation
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
		bValidTimestamps = false;

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

class ServerHMDView : public ServerDeviceView, public IHMDListener
{
public:
    ServerHMDView(const int device_id);
    ~ServerHMDView();

    bool open(const class DeviceEnumerator *enumerator) override;
    void close() override;

	// Update Pose Filter using update packets from the tracker and IMU threads
	void updatePoseFilter();

	// Recreate and initialize the pose filter for the HMD
	void resetPoseFilter();

    IDeviceInterface* getDevice() const override { return m_device; }
	inline class IPoseFilter * getPoseFilterMutable() { return m_pose_filter; }
	inline const class IPoseFilter * getPoseFilter() const { return m_pose_filter; }

	// Estimate the given pose if the controller at some point into the future
	PSVRPosef getFilteredPose(float time = 0.f) const;

	// Get the current physics from the filter position and orientation
	PSVRPhysicsData getFilteredPhysics() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

	// Returns the "controller_" + serial number for the controller
	std::string getConfigIdentifier() const;

    // Returns what type of HMD this HMD view represents
    CommonSensorState::eDeviceType getHMDDeviceType() const;

	// Get the last IMU sensor packet posted for this HMD
	const PoseSensorPacket *getLastIMUSensorPacket() const;

	// Get the last optical sensor packet posted for this HMD from the given tracker ID
	const PoseSensorPacket *getLastOpticalSensorPacket(int tracker_id) const;

	// Get the tracking is enabled on this controller
	inline bool getIsTrackingEnabled() const { return m_tracking_enabled.load(); }

	// Increment the position tracking listener count
	// Starts position tracking this HMD if the count was zero
	void startTracking();

	// Decrements the position tracking listener count
	// Stop tracking this HMD if this count becomes zero
	void stopTracking();

	// Get the tracking shape for the controller
	bool getTrackingShape(PSVRTrackingShape &outTrackingShape) const;

	// Get the currently assigned tracking color ID for the controller
	PSVRTrackingColorType getTrackingColorID() const;

    // Set the assigned tracking color ID for the controller
    bool setTrackingColorID(PSVRTrackingColorType colorID);

	// Get if the region-of-interest optimization is disabled for this HMD
	inline bool getIsROIDisabled() const { return m_roi_disable_count > 0; }

	// Request the HMD to not use the ROI optimization
	inline void pushDisableROI() { ++m_roi_disable_count; }

	// Undo the request to not use the ROI optimization
	inline void popDisableROI() { assert(m_roi_disable_count > 0); --m_roi_disable_count; }

	// get the prediction time used for region of interest calculation
	float getROIPredictionTime() const;

	// Get the pose estimate relative to the given tracker id on the tracker worker thread
	inline const HMDOpticalPoseEstimation *getOpticalPoseEstimateOnTrackerThread(int trackerId) const {
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
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_hmd_data_frame_for_stream(
        const ServerHMDView *hmd_view,
        const struct HMDStreamInfo *stream_info,
        DeviceOutputDataFrame &data_frame);

private:
	// Tracking color state
	int m_tracking_listener_count;
	std::atomic_bool m_tracking_enabled;

	// ROI state
	int m_roi_disable_count;

	// Device State
    IHMDInterface *m_device;

	// Filter State (Tracker Thread)
    class IShapeTrackingModel **m_shape_tracking_models;  // array of size TrackerManager::k_max_devices
	struct HMDOpticalPoseEstimation *m_optical_pose_estimations; // array of size TrackerManager::k_max_devices

	// Filter State (IMU Thread)
	std::chrono::time_point<std::chrono::high_resolution_clock> m_lastSensorDataTimestamp;
	bool m_bIsLastSensorDataTimestampValid;

	// Filter State (Shared)
	t_hmd_pose_sensor_queue m_PoseSensorIMUPacketQueue;
	t_hmd_pose_sensor_queue m_PoseSensorOpticalPacketQueue;
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

#endif // SERVER_HMD_VIEW_H