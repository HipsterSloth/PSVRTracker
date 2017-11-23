#ifndef VIRTUAL_HMD_H
#define VIRTUAL_HMD_H

#include "PSVRConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathUtility.h"
#include <string>
#include <vector>
#include <deque>
#include <array>


class VirtualHMDConfig : public PSVRConfig
{
public:
    static const int CONFIG_VERSION;

    VirtualHMDConfig(const std::string &fnamebase = "VirtualHMDConfig")
        : PSVRConfig(fnamebase)
		, is_valid(false)
		, version(CONFIG_VERSION)
		, position_filter_type("LowPassOptical")
        , orientation_filter_type("PassThru")
        , max_velocity(1.f)
		, mean_update_time_delta(0.008333f)
		, position_variance_exp_fit_a(0.0994158462f)
		, position_variance_exp_fit_b(-0.000567041978f)
        , orientation_variance(0.005f)
        , prediction_time(0.f)
		, tracking_color_id(PSVRTrackingColorType_Blue)
    {
        trackingShape.shape_type = PSVRTrackingShape_PointCloud;
        trackingShape.shape.pointcloud.points[0] = {0.00f, 0.00f, 0.00f}; // 0
        trackingShape.shape.pointcloud.points[1] = {7.25f, 4.05f, 3.75f}; // 1
        trackingShape.shape.pointcloud.points[2] = {9.05f, 0.00f, 9.65f}; // 2
        trackingShape.shape.pointcloud.points[3] = {7.25f, -4.05f, 3.75f}; // 3
        trackingShape.shape.pointcloud.points[4] = {-7.25f, 4.05f, 3.75f}; // 4
        trackingShape.shape.pointcloud.points[5] = {-9.05f, 0.00f, 9.65f}; // 5
        trackingShape.shape.pointcloud.points[6] = {-7.25f, -4.05f, 3.75f}; // 6
        trackingShape.shape.pointcloud.points[7] = {5.65f, -1.07f, 27.53f}; // 7
        trackingShape.shape.pointcloud.points[8] = {-5.65f, -1.07f, 27.53f}; // 8
	    trackingShape.shape.pointcloud.point_count = 9;
    };

    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

    bool is_valid;
    long version;

	// The type of position filter to use
	std::string position_filter_type;

	// The type of orientation filter to use
	std::string orientation_filter_type;

	// Maximum velocity for the controller physics (meters/second)
	float max_velocity;

	// The average time between updates in seconds
	float mean_update_time_delta;

	// The variance of the controller position as a function of pixel area
	float position_variance_exp_fit_a;
	float position_variance_exp_fit_b;

	inline float get_position_variance(float projection_area) const {
		return position_variance_exp_fit_a*exp(position_variance_exp_fit_b*projection_area);
	}

	// The variance of the hmd orientation (when sitting still) in rad^2
	float orientation_variance;

	float prediction_time;

	PSVRTrackingColorType tracking_color_id;
    PSVRTrackingShape trackingShape;
};

struct VirtualHMDSensorState : public CommonHMDSensorState
{
    VirtualHMDSensorState()
    {
        clear();
    }

    void clear()
    {
        CommonHMDSensorState::clear();
		DeviceType = VirtualHMD;
    }
};

class VirtualHMD : public IHMDInterface 
{
public:
    VirtualHMD();
    virtual ~VirtualHMD();

    // VirtualHMD
    bool open(); // Opens the first virtualHMD listed in the HMD manager

    // -- IDeviceInterface
    bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    bool open(const DeviceEnumerator *enumerator) override;
    bool getIsOpen() const override;
    bool getIsReadyToPoll() const override;
    IDeviceInterface::ePollResult poll() override;
    void close() override;
    long getMaxPollFailureCount() const override;
    CommonSensorState::eDeviceType getDeviceType() const override
    {
        return CommonSensorState::VirtualHMD;
    }
    static CommonSensorState::eDeviceType getDeviceTypeStatic()
    {
        return CommonSensorState::VirtualHMD;
    }
    const CommonSensorState * getSensorState(int lookBack = 0) const override;

    // -- IHMDInterface
    std::string getUSBDevicePath() const override;
	void getTrackingShape(PSVRTrackingShape &outTrackingShape) const override;
	bool setTrackingColorID(const PSVRTrackingColorType tracking_color_id) override;
	bool getTrackingColorID(PSVRTrackingColorType &out_tracking_color_id) const override;
	float getPredictionTime() const override;

    // -- Getters
    inline const VirtualHMDConfig *getConfig() const
    {
        return &cfg;
    }
    inline VirtualHMDConfig *getConfigMutable()
    {
        return &cfg;
    }

    // -- Setters
	void setTrackingEnabled(bool bEnableTracking);

private:
    // Constant while the HMD is open
    VirtualHMDConfig cfg;
    std::string device_identifier;
    bool bIsOpen;

    // Read HMD State
    int NextPollSequenceNumber;
    std::deque<VirtualHMDSensorState> HMDStates;

	bool bIsTracking;
};

#endif // VIRTUAL_HMD_H
