#ifndef SERVER_TRACKER_VIEW_H
#define SERVER_TRACKER_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"
#include "PSVRServiceInterface.h"
#include <vector>

// -- constants -----

// -- pre-declarations -----
namespace PSVRProtocol
{
    class Response_ResultTrackerSettings;
    class TrackingColorPreset;
};

namespace cv
{
	class Mat;
}

// -- declarations -----
class ServerTrackerView : public ServerDeviceView, public ITrackerListener
{
public:
    ServerTrackerView(const int device_id);
    ~ServerTrackerView();

    // Used by virtual trackers to allocate child tracker devices
    static ITrackerInterface *allocate_tracker_interface(const class DeviceEnumerator *enumerator);

    bool open(const class DeviceEnumerator *enumerator) override;
    void close() override;
	void pollUpdatedVideoFrame();

    // Starts or stops streaming of the video feed to the shared memory buffer.
    // Keep a ref count of how many clients are following the stream.
    void startSharedMemoryVideoStream();
    void stopSharedMemoryVideoStream();

    IDeviceInterface* getDevice() const override {return m_device;}
	ITrackerInterface* getTrackerDevice() const { return static_cast<ITrackerInterface *>(getDevice()); }

    // Returns what type of tracker this tracker view represents
    CommonSensorState::eDeviceType getTrackerDeviceType() const;

    // Returns what type of driver this tracker uses
    ITrackerInterface::eDriverType getTrackerDriverType() const;

    // Returns true if this is a stereo camera
    bool getIsStereoCamera() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

    // Returns the name of the shared memory block video frames are written to
    std::string getSharedMemoryStreamName() const;

    // Returns a pointer the memory buffer the video frames are published to
    class SharedVideoFrameBuffer *getSharedVideoFrameBuffer() const;
    
    void loadSettings();
    void saveSettings();

	bool getAvailableTrackerModes(std::vector<std::string> &out_mode_names) const;
	const struct TrackerModeConfig *getTrackerMode() const;
	bool setTrackerMode(const std::string &new_mode);

	double getFrameWidth() const;
	double getFrameHeight() const;
	double getFrameRate() const;

	bool getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const;

	int getVideoProperty(const PSVRVideoPropertyType property_type) const;
	void setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool save_setting);

    bool computeProjectionForController(
		const class ServerControllerView* tracked_controller,
		const PSVRTrackingShape *tracking_shape,
		PSVRTrackingProjection *out_projection);
    bool computeProjectionForHMD(
		const class ServerHMDView* tracked_hmd,
		const PSVRTrackingShape *tracking_shape,
		PSVRTrackingProjection *out_projection);

	cv::Mat *getDebugDrawingBuffer(PSVRVideoFrameSection section) const;
	void drawPoseProjection(const PSVRTrackingProjection *projection) const;
    
    std::vector<PSVRVector2f> projectTrackerRelativePositions(
		const PSVRVideoFrameSection section,
        const std::vector<PSVRVector3f> &objectPositions) const;
    
    PSVRVector2f projectTrackerRelativePosition(
		const PSVRVideoFrameSection section,
		const PSVRVector3f *trackerRelativePosition) const;
    
    void computeWorldShape(const PSVRTrackingShape *tracker_relative_shape, PSVRTrackingShape *out_shape) const;
    PSVRVector3f computeWorldPosition(const PSVRVector3f *tracker_relative_position) const;
    PSVRQuatf computeWorldOrientation(const PSVRQuatf *tracker_relative_orientation) const;

    PSVRVector3f computeTrackerPosition(const PSVRVector3f *world_relative_position) const;
    PSVRQuatf computeTrackerOrientation(const PSVRQuatf *world_relative_orientation) const;

    void getCameraIntrinsics(PSVRTrackerIntrinsics &out_tracker_intrinsics) const;
    void setCameraIntrinsics(const PSVRTrackerIntrinsics &tracker_intrinsics);

    PSVRPosef getTrackerPose() const;
    void setTrackerPose(const PSVRPosef *pose);

    void getPixelDimensions(float &outWidth, float &outHeight) const;
    void getFOV(float &outHFOV, float &outVFOV) const;
    void getZRange(float &outZNear, float &outZFar) const;

	void gatherTrackingColorPresets(const class ServerControllerView *hmd, PSVRClientTrackerSettings* settings) const;
	void setControllerTrackingColorPreset(const class ServerControllerView *controller, PSVRTrackingColorType color, const PSVR_HSVColorRange *preset);
	void getControllerTrackingColorPreset(const class ServerControllerView *controller, PSVRTrackingColorType color, PSVR_HSVColorRange *out_preset) const;

	void gatherTrackingColorPresets(const class ServerHMDView *hmd, PSVRClientTrackerSettings* settings) const;
	void setHMDTrackingColorPreset(const class ServerHMDView *hmd, PSVRTrackingColorType color, const PSVR_HSVColorRange *preset);
	void getHMDTrackingColorPreset(const class ServerHMDView *hmd, PSVRTrackingColorType color, PSVR_HSVColorRange *out_preset) const;

	//-- ITrackerListener
	virtual void notifyVideoFrameReceived(const unsigned char *raw_video_frame_buffer) override;

protected:
    void reallocate_shared_memory();
    void reallocate_opencv_buffer_state();
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_tracker_data_frame_for_stream(
        const ServerTrackerView *tracker_view, const struct TrackerStreamInfo *stream_info,
        DeviceOutputDataFrame &data_frame);

    bool computeProjectionForControllerInSection(
        const ServerControllerView* tracked_controller,
        const PSVRTrackingShape *tracking_shape,
        const PSVRVideoFrameSection section,
        PSVRTrackingProjection *out_projection);
    bool computeProjectionForHmdInSection(
        const ServerHMDView* tracked_controller,
        const PSVRTrackingShape *tracking_shape,
        const PSVRVideoFrameSection section,
        PSVRTrackingProjection *out_projection);

private:
    char m_shared_memory_name[256];
    class SharedVideoFrameBuffer *m_shared_memory_accesor;
    std::atomic_int m_shared_memory_video_stream_count;
	int m_lastVideoFrameIndexPolled;
    class OpenCVBufferState *m_opencv_buffer_state[MAX_PROJECTION_COUNT];
    ITrackerInterface *m_device;

};

#endif // SERVER_TRACKER_VIEW_H
