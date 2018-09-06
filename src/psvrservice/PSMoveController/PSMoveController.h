#ifndef PSMOVE_CONTROLLER_H
#define PSMOVE_CONTROLLER_H

#include "PSVRConfig.h"
#include "PSVRClient_CAPI.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathUtility.h"
#include "hidapi.h"

#include <string>
#include <array>
#include <deque>
#include <chrono>

enum PSMoveControllerModelPID
{
	_psmove_controller_ZCM1= 0x03d5,
	_psmove_controller_ZCM2= 0x0C5E 
};

struct PSMoveHIDDetails {
	int vendor_id;
	int product_id;
    std::string Device_path;
    hid_device *Handle;
    std::string Device_path_addr; // only needed by Win > 8.1, otherwise ignored.
    hid_device *Handle_addr; // only needed by Win > 8.1, otherwise ignored.
    std::string Bt_addr;      // The bluetooth address of the controller
    std::string Host_bt_addr; // The bluetooth address of the adapter registered with the controller
};

class PSMoveControllerConfig : public PSVRConfig
{
public:
    static const int CONFIG_VERSION;

    PSMoveControllerConfig(const std::string &fnamebase = "PSMoveControllerConfig")
        : PSVRConfig(fnamebase)
        , is_valid(false)
        , version(CONFIG_VERSION)
		, firmware_version(0)
		, bt_firmware_version(0)
		, firmware_revision(0)
        , poll_timeout_ms(1000) 
        , prediction_time(0.f)
		, position_filter_type("LowPassExponential")
		, orientation_filter_type("ComplementaryMARG")
        , cal_ag_xyz_kb({{ 
            {{ {{0, 0}}, {{0, 0}}, {{0, 0}} }},
            {{ {{0, 0}}, {{0, 0}}, {{0, 0}} }} 
        }})
        , magnetometer_fit_error(0.f)
		, magnetometer_variance(0.00059f) // rounded value from config tool measurement
		, accelerometer_variance(7.2e-06f) // rounded value from config tool measurement
        , accelerometer_noise_radius(0.014f) // rounded value from config tool measurement
        , gyro_variance(0.00035f) // rounded value from config tool measurement (rad/s)^2
        , gyro_drift(0.027f) // rounded value from config tool measurement (rad/s)
        , max_velocity(1.f)
		, mean_update_time_delta(0.008333f)
		, position_variance_exp_fit_a(0.0994158462f)
		, position_variance_exp_fit_b(-0.000567041978f)
		, orientation_variance(18.75f)
		, tracking_color_id(PSVRTrackingColorType_INVALID)
		, hand("Any")
    {
        magnetometer_identity = *k_PSVR_float_vector3_zero;
        magnetometer_center = *k_PSVR_float_vector3_zero;
        magnetometer_basis_x = *k_PSVR_float_vector3_zero;
        magnetometer_basis_y = *k_PSVR_float_vector3_zero;
        magnetometer_basis_z = *k_PSVR_float_vector3_zero;
        magnetometer_extents = *k_PSVR_float_vector3_zero;
        magnetometer_identity = *k_PSVR_float_vector3_zero;
    };

    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

    void getMagnetometerEllipsoid(struct EigenFitEllipsoid *out_ellipsoid) const;

    bool is_valid;
    long version;

	// Move's firmware version number
	unsigned short firmware_version;

	// Move Bluetooth module's firmware version number
	unsigned short bt_firmware_version; 

	// Move's firmware revision number
	unsigned short firmware_revision;


	long poll_timeout_ms;

	// The amount of prediction to apply to the controller pose after filtering
    float prediction_time;

	// The type of position filter to use
	std::string position_filter_type;

	// The type of orientation filter to use
	std::string orientation_filter_type;

	// The accelerometer and gyroscope scale and bias values read from the USB calibration packet
    std::array<std::array<std::array<float, 2>, 3>, 2> cal_ag_xyz_kb;

	// The direction of the magnetometer when in the identity pose
    PSVRVector3f magnetometer_identity;

	// The bias of the magnetometer readings
    PSVRVector3f magnetometer_center;

	// The basis vectors of the best fit ellipsoid (typically i,j,k)
    PSVRVector3f magnetometer_basis_x;
    PSVRVector3f magnetometer_basis_y;
    PSVRVector3f magnetometer_basis_z;
    PSVRVector3f magnetometer_extents;

	// The squared error of the magnetometer fit ellipsoid
    float magnetometer_fit_error;

	/// The variance of the magnetometer sensor
	float magnetometer_variance; // units^2

	// The variance of the accelerometer readings
	float accelerometer_variance; // g-units^2

    // The bounding radius of the accelerometer noise
    float accelerometer_noise_radius;
    
    // The variance of the calibrated gyro readings in (rad/s)^2
    float gyro_variance;

    // The drift of the calibrated gyro readings in rad/s
    float gyro_drift;

    // The maximum velocity allowed in the position filter in cm/s
    float max_velocity;

	// The average time between updates in seconds
    float mean_update_time_delta;

	// The variance(cm^2) of the controller position (meters^2) as a function of pixel area
    float position_variance_exp_fit_a; 
    float position_variance_exp_fit_b;

	// The variance of the controller orientation (when sitting still) in rad^2
    float orientation_variance;

	inline float get_position_variance(float projection_area) const {
		return position_variance_exp_fit_a*exp(position_variance_exp_fit_b*projection_area);
	}

	// The assigned tracking color for this controller
	PSVRTrackingColorType tracking_color_id;

	// The assigned hand for this controller
	std::string hand;
};

struct PSMoveControllerInputState : public CommonControllerState
{
    int RawSequence;                            // 4-bit (1..16).
                                                // Sometimes frames are dropped.
    
    unsigned int RawTimeStamp;                  // 16-bit (time since ?, units?)
                                                // About 1150 between in-order frames.
    PSVRButtonState Triangle;
    PSVRButtonState Circle;
    PSVRButtonState Cross;
    PSVRButtonState Square;
    PSVRButtonState Select;
    PSVRButtonState Start;
    PSVRButtonState PS;
    PSVRButtonState Move;
    PSVRButtonState Trigger;

    unsigned char TriggerValue;  // 0-255. Average of last two frames.
    unsigned char BatteryValue;  // 0-5 for level, EE-EF for charging/charged 

    std::array< std::array<int, 3>, 2> RawAccel;    // Two frames of 3 dimensions
    std::array< std::array<int, 3>, 2> RawGyro;     // Two frames of 3 dimensions
    std::array<int, 3> RawMag;                      // One frame of 3 dimensions

    std::array< std::array<float, 3>, 2> CalibratedAccel;    // Two frames of 3 dimensions
    std::array< std::array<float, 3>, 2> CalibratedGyro;     // Two frames of 3 dimensions
    std::array<float, 3> CalibratedMag;                       // One frame of 3 dimensions

    int TempRaw;

    //TODO: high-precision timestamp. Need to do in hidapi?
    
    PSMoveControllerInputState();

    void clear();
	void parseDataInput(
		const PSMoveControllerConfig *config, 
		const PSMoveControllerModelPID model,
		const struct PSMoveDataInputCommon *previous_hid_packet,
		const struct PSMoveDataInputCommon *new_hid_packet);
};

struct PSMoveControllerOutputState 
{
    unsigned char r;        // red value, 0x00..0xff
    unsigned char g;        // green value, 0x00..0xff
    unsigned char b;        // blue value, 0x00..0xff

    unsigned char rumble;   // rumble value, 0x00..0xff
};

class PSMoveController : public IControllerInterface {
public:
    PSMoveController();
    virtual ~PSMoveController();

    // PSMoveController
    bool open(); // Opens the first HID device for the controller
	bool open(PSMoveControllerModelPID model);
    
    // -- IDeviceInterface
    virtual bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    virtual bool open(const DeviceEnumerator *enumerator) override;
    virtual bool getIsOpen() const override;
    virtual void close() override;
    CommonSensorState::eDeviceType getDeviceType() const override
    {
        return CommonSensorState::PSMove;
    }
    static CommonSensorState::eDeviceType getDeviceTypeStatic()
    {
        return CommonSensorState::PSMove;
    }
    
    // -- IControllerInterface
    virtual bool setHostBluetoothAddress(const std::string &address) override;
	virtual bool setTrackingColorID(const PSVRTrackingColorType tracking_color_id) override;
    virtual bool getIsBluetooth() const override;
    virtual std::string getUSBDevicePath() const override;
	virtual int getVendorID() const override;
	virtual int getProductID() const override;
    virtual std::string getAssignedHostBluetoothAddress() const override;
    virtual std::string getSerial() const override;
    virtual const std::tuple<unsigned char, unsigned char, unsigned char> getColour() const override;
    virtual void getTrackingShape(PSVRTrackingShape &outTrackingShape) const override;
	virtual bool getTrackingColorID(PSVRTrackingColorType &out_tracking_color_id) const override;
	virtual float getIdentityForwardDegrees() const override;
	virtual float getPredictionTime() const override;

    // -- Getters
    inline const PSMoveControllerConfig *getConfig() const
    { return &cfg; }
    inline PSMoveControllerConfig *getConfigMutable()
    { return &cfg; }
    float getTempCelsius() const;
	bool getSupportsMagnetometer() const
	{ return SupportsMagnetometer; }        
    const unsigned long getLEDPWMFrequency() const
    { return LedPWMF; }
	const bool getIsPS4Controller() const 
	{ return HIDDetails.product_id == _psmove_controller_ZCM2; }
    
    // -- Setters
    bool setLED(unsigned char r, unsigned char g, unsigned char b); // 0x00..0xff. TODO: vec3
    bool setLEDPWMFrequency(unsigned long freq);    // 733..24e6
    bool setRumbleIntensity(unsigned char value);
	bool enableDFUMode(); // Device Firmware Update mode
	void setControllerListener(IControllerListener *listener) override;

private:    
    bool getBTAddress(std::string& host, std::string& controller);
    void loadCalibrationZCM1();                         // Use USB or file if on BT
	void loadCalibrationZCM2();                         // Use USB or file if on BT
	bool loadFirmwareInfo();
    
    // Constant while a controller is open
    PSMoveControllerConfig cfg;
    PSMoveHIDDetails HIDDetails;
    bool IsBluetooth;                               // true if valid serial number on device opening
	bool SupportsMagnetometer;                      // true if controller emits valid magnetometer data

    // Cached Setter State
    unsigned long LedPWMF;
	PSMoveControllerOutputState m_cachedOutputState;

    // Controller State
	class PSMoveHidPacketProcessor* m_HIDPacketProcessor;
	IControllerListener* m_controllerListener;

};
#endif // PSMOVE_CONTROLLER_H