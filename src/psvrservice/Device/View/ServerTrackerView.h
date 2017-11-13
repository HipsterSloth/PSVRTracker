#ifndef SERVER_TRACKER_VIEW_H
#define SERVER_TRACKER_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"
#include "PSVRServiceInterface.h"
#include <vector>

// -- pre-declarations -----
namespace PSVRProtocol
{
    class Response_ResultTrackerSettings;
    class TrackingColorPreset;
};

// -- declarations -----
class ServerTrackerView : public ServerDeviceView
{
public:
    ServerTrackerView(const int device_id);
    ~ServerTrackerView();

    bool open(const class DeviceEnumerator *enumerator) override;
    void close() override;

    // Starts or stops streaming of the video feed to the shared memory buffer.
    // Keep a ref count of how many clients are following the stream.
    void startSharedMemoryVideoStream();
    void stopSharedMemoryVideoStream();

    // Fetch the next video frame and copy to shared memory
    bool poll() override;

    IDeviceInterface* getDevice() const override {return m_device;}

    // Returns what type of tracker this tracker view represents
    CommonDeviceState::eDeviceType getTrackerDeviceType() const;

    // Returns what type of driver this tracker uses
    ITrackerInterface::eDriverType getTrackerDriverType() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

    // Returns the name of the shared memory block video frames are written to
    std::string getSharedMemoryStreamName() const;
    
    void loadSettings();
    void saveSettings();

	double getFrameWidth() const;
	void setFrameWidth(double value, bool bUpdateConfig);

	double getFrameHeight() const;
	void setFrameHeight(double value, bool bUpdateConfig);

	double getFrameRate() const;
	void setFrameRate(double value, bool bUpdateConfig);

    double getExposure() const;
    void setExposure(double value, bool bUpdateConfig);

    double getGain() const;
    void setGain(double value, bool bUpdateConfig);
    
    bool computeProjectionForController(
        const class ServerControllerView* tracked_controller, 
		const PSVRTrackingShape *tracking_shape,
        struct ControllerOpticalPoseEstimation *out_pose_estimate);
    bool computeProjectionForHMD(
		const class ServerHMDView* tracked_hmd,
		const PSVRTrackingShape *tracking_shape,
		struct HMDOpticalPoseEstimation *out_pose_estimate);
    bool computePoseForProjection(
		const PSVRTrackingProjection *projection,
		const PSVRTrackingShape *tracking_shape,
		const PSVRPosef *pose_guess,
		struct ControllerOpticalPoseEstimation *out_pose_estimate);
    
    std::vector<PSVRVector2f> projectTrackerRelativePositions(
                                const std::vector<PSVRVector3f> &objectPositions) const;
    
    PSVRVector2f projectTrackerRelativePosition(const PSVRVector3f *trackerRelativePosition) const;
    
    PSVRVector3f computeWorldPosition(const PSVRVector3f *tracker_relative_position) const;
    PSVRQuatf computeWorldOrientation(const PSVRQuatf *tracker_relative_orientation) const;

    PSVRVector3f computeTrackerPosition(const PSVRVector3f *world_relative_position) const;
    PSVRQuatf computeTrackerOrientation(const PSVRQuatf *world_relative_orientation) const;

    /// Given a single screen location on two different trackers, compute the triangulated world space location
    static PSVRVector3f triangulateWorldPosition(
        const ServerTrackerView *tracker, const PSVRVector2f *screen_location,
        const ServerTrackerView *other_tracker, const PSVRVector2f *other_screen_location);

	/// Given a set of screen locations on two different trackers, compute the triangulated world space locations
	static void triangulateWorldPositions(
		const ServerTrackerView *tracker, 
		const PSVRVector2f *screen_locations,
		const ServerTrackerView *other_tracker,
		const PSVRVector2f *other_screen_locations,
		const int screen_location_count,
		PSVRVector3f *out_result);

    /// Given screen projections on two different trackers, compute the triangulated world space location
    static PSVRPosef triangulateWorldPose(
        const ServerTrackerView *tracker, const PSVRTrackingProjection *tracker_relative_projection,
        const ServerTrackerView *other_tracker, const PSVRTrackingProjection *other_tracker_relative_projection);

    void getCameraIntrinsics(
        float &outFocalLengthX, float &outFocalLengthY,
        float &outPrincipalX, float &outPrincipalY,
        float &outDistortionK1, float &outDistortionK2, float &outDistortionK3,
        float &outDistortionP1, float &outDistortionP2) const;
    void setCameraIntrinsics(
        float focalLengthX, float focalLengthY,
        float principalX, float principalY,
        float distortionK1, float distortionK2, float distortionK3,
        float distortionP1, float distortionP2);

    PSVRPosef getTrackerPose() const;
    void setTrackerPose(const PSVRPosef *pose);

    void getPixelDimensions(float &outWidth, float &outHeight) const;
    void getFOV(float &outHFOV, float &outVFOV) const;
    void getZRange(float &outZNear, float &outZFar) const;

    void gatherTrackerOptions(PSVRProtocol::Response_ResultTrackerSettings* settings) const;
    bool setOptionIndex(const std::string &option_name, int option_index);
    bool getOptionIndex(const std::string &option_name, int &out_option_index) const;

    void gatherTrackingColorPresets(const class ServerControllerView *controller, PSVRProtocol::Response_ResultTrackerSettings* settings) const;
	void gatherTrackingColorPresets(const class ServerHMDView *hmd, PSVRProtocol::Response_ResultTrackerSettings* settings) const;

    void setControllerTrackingColorPreset(const class ServerControllerView *controller, PSVRTrackingColorType color, const PSVR_HSVColorRange *preset);
    void getControllerTrackingColorPreset(const class ServerControllerView *controller, PSVRTrackingColorType color, PSVR_HSVColorRange *out_preset) const;

	void setHMDTrackingColorPreset(const class ServerHMDView *controller, PSVRTrackingColorType color, const PSVR_HSVColorRange *preset);
	void getHMDTrackingColorPreset(const class ServerHMDView *controller, PSVRTrackingColorType color, PSVR_HSVColorRange *out_preset) const;

protected:
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_tracker_data_frame_for_stream(
        const ServerTrackerView *tracker_view, const struct TrackerStreamInfo *stream_info,
        DeviceOutputDataFrame &data_frame);

private:
    char m_shared_memory_name[256];
    class SharedVideoFrameReadWriteAccessor *m_shared_memory_accesor;
    int m_shared_memory_video_stream_count;
    class OpenCVBufferState *m_opencv_buffer_state;
    ITrackerInterface *m_device;
};

#endif // SERVER_TRACKER_VIEW_H
