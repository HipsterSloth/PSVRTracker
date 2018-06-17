//-- includes -----
#include "MorpheusHMD.h"
#include "DeviceInterface.h"
#include "DeviceManager.h"
#include "HMDDeviceEnumerator.h"
#include "HidHMDDeviceEnumerator.h"
#include "MathUtility.h"
#include "Logger.h"
#include "Utility.h"
#include "USBDeviceManager.h"
#include "WorkerThread.h"
#include "hidapi.h"
#include <vector>
#include <cstdlib>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

//-- constants -----
#define INTERRUPT_TRANSFER_TIMEOUT	  500 /* timeout in ms */

#define MORPHEUS_VENDOR_ID 0x054c
#define MORPHEUS_PRODUCT_ID 0x09af

#define MORPHEUS_CONFIGURATION_PSVR  1
#define MORPHEUS_ENDPOINT_IN 0x80

#define MORPHEUS_SENSOR_INTERFACE 4
#define MORPHEUS_COMMAND_INTERFACE 5

#define MORPHEUS_COMMAND_MAGIC 0xAA
#define MORPHEUS_COMMAND_MAX_PAYLOAD_LEN 60

#define MORPHEUS_HMD_STATE_BUFFER_MAX 4
#define METERS_TO_CENTIMETERS 100

enum eMorpheusRequestType
{
	Morpheus_Req_EnableTracking= 0x11,
	Morpheus_Req_TurnOffProcessorUnit = 0x13,
	Morpheus_Req_SetLEDBrightness = 0x15,
	Morpheus_Req_SetHeadsetPower= 0x17,
	Morpheus_Req_SetCinematicConfiguration= 0x21,
	Morpheus_Req_SetVRMode= 0x23,
};

enum eMorpheusLED
{                           // Sides relative to looking 
	_MorpheusLED_A= 1 << 0, // Front-Bottom-Right
	_MorpheusLED_B= 1 << 1, // Front-Bottom-Left
	_MorpheusLED_C= 1 << 2, // Front-Top-Right
	_MorpheusLED_D= 1 << 3, // Front-Top-Left
	_MorpheusLED_E= 1 << 4, // Front-Middle-Center
	_MorpheusLED_F= 1 << 5, // Front-Middle-Right
	_MorpheusLED_G= 1 << 6, // Front-Middle-Left
	_MorpheusLED_H= 1 << 7, // Rear-Right
	_MorpheusLED_I= 1 << 8, // Rear-Left

	_MorpheusLED_ALL_FRONT=_MorpheusLED_A|_MorpheusLED_B|_MorpheusLED_C|_MorpheusLED_D|_MorpheusLED_E|_MorpheusLED_F|_MorpheusLED_G,
	_MorpheusLED_ALL_BACK=_MorpheusLED_H|_MorpheusLED_I,
	_MorpheusLED_ALL= _MorpheusLED_ALL_FRONT|_MorpheusLED_ALL_BACK
};

// -- private definitions -----
class MorpheusUSBContext 
{
public:
	std::string device_identifier;

	// HIDApi state
    std::string sensor_device_path;
	hid_device *sensor_device_handle;
	
	// USB state
    std::string usb_device_path;
	t_usb_device_handle usb_device_handle;

    MorpheusUSBContext()
    {
        Reset();
    }

    void Reset()
    {
		device_identifier = "";
        sensor_device_path = "";
		sensor_device_handle = nullptr;
        usb_device_path= "";
        usb_device_handle= k_invalid_usb_device_handle;
    }
};

enum eMorpheusButton : unsigned char
{
	VolumePlus = 2,
	VolumeMinus = 4,
	MicrophoneMute = 8,
};

struct MorpheusRawSensorFrame
{
	unsigned char seq_frame[2];
	unsigned char gyro_yaw[2];
	unsigned char gyro_pitch[2];
	unsigned char gyro_roll[2];
	unsigned char accel_x[2]; //It's the X value of the sensor, but it's mounted rotated on the headset
	unsigned char accel_y[2];
	unsigned char accel_z[2];
};

#pragma pack(1)
//See https://github.com/gusmanb/PSVRFramework/wiki/Sensor-report
struct MorpheusSensorData
{
	eMorpheusButton buttons;					// byte 0
	unsigned char unk0;							// byte 1
	unsigned char volume;                       // byte 2
	unsigned char unk1[5];                      // byte 3-7

	union
	{
		unsigned char asByte;
		struct
		{
			unsigned char hmdOnHead : 1;
			unsigned char displayIsOn : 1;
			unsigned char HDMIDisconnected : 1;
			unsigned char microphoneMuted : 1;
			unsigned char headphonesPresent : 1;
			unsigned char unk1 : 2;
			unsigned char timer : 1;
		};
	} headsetFlags;								// byte 8

	unsigned char unkFlags;     				// byte 9
	unsigned char unk2[8];						// byte 10-17

	MorpheusRawSensorFrame imu_frame_0;         // byte 18-31
	unsigned char unk3[2];						// byte 32-33
	MorpheusRawSensorFrame imu_frame_1;         // byte 34-47

	unsigned char calibration_status;           // byte 48: 255 = boot, 0 - 3 calibrating ? 4 = calibrated, maybe a bit mask with sensor status ? (0 bad, 1 good) ?
	unsigned char sensors_ready;                // byte 49 
	unsigned char unk4[3];						// byte 50-52 
	unsigned char unk5;                         // byte 53: Voltage reference ? starts in 0 and suddenly jumps to 3
	unsigned char unk6;	                        // byte 54: Voltage value ? starts on 0, ranges very fast to 255, when switched from VR to Cinematic and back varies between 255 and 254 and sometimes oscillates between them
	unsigned char face_distance[2];             // byte 55-56: Infrared headset sensor, 0 to 1023, used to measure distance between the face / head and visor
	unsigned char unk7[6];                      // byte 57-62
	unsigned char sequence;                     // byte 63

    MorpheusSensorData()
    {
        Reset();
    }

    void Reset()
    {
        memset(this, 0, sizeof(MorpheusSensorData));
    }
};

struct MorpheusCommandHeader
{
	unsigned char request_id;
	unsigned char command_status;
	unsigned char magic;
	unsigned char length;
};

struct MorpheusCommand
{
	MorpheusCommandHeader header;
	unsigned char payload[MORPHEUS_COMMAND_MAX_PAYLOAD_LEN]; // variable sized payload depending on report id
};
#pragma pack()

class MorpheusSensorProcessor : public WorkerThread
{
public:
	MorpheusSensorProcessor(const MorpheusHMDConfig &cfg) 
		: WorkerThread("MorpheusSensorProcessor")
		, m_cfg(cfg)
		, m_hidDevice(nullptr)
		, m_nextPollSequenceNumber(0)
		, m_rawHIDPacket(nullptr)
	{
		m_rawHIDPacket = new MorpheusSensorData;
	}

	~MorpheusSensorProcessor()
	{
		delete m_rawHIDPacket;
	}

    void start(hid_device *in_hid_device, IHMDListener *hmd_listener)
    {
		if (!hasThreadStarted())
		{
			m_hidDevice= in_hid_device;
			m_hmdListener= hmd_listener;
			WorkerThread::startThread();
		}
    }

	void stop()
	{
		WorkerThread::stopThread();
	}

protected:
	virtual bool doWork() override
    {
		bool bWorking= true;

		// Attempt to read the next sensor update packet from the HMD
		int res = hid_read_timeout(
			m_hidDevice, 
			(unsigned char*)m_rawHIDPacket,
			sizeof(MorpheusSensorData),
			m_cfg.poll_timeout_ms);

		if (res > 0)
		{
			// https://github.com/hrl7/node-psvr/blob/master/lib/psvr.js
			MorpheusHMDSensorState newState;

			// Increment the sequence for every new polling packet
			newState.PollSequenceNumber = m_nextPollSequenceNumber;
			++m_nextPollSequenceNumber;

			// Processes the IMU data
			newState.parse_data_input(&m_cfg, m_rawHIDPacket);

			m_hmdListener->notifySensorDataReceived(&newState);
		}
		else if (res < 0)
		{
			char hidapi_err_mbs[256];
			bool valid_error_mesg = 
				Utility::convert_wcs_to_mbs(hid_error(m_hidDevice), hidapi_err_mbs, sizeof(hidapi_err_mbs));

			// Device no longer in valid state.
			if (valid_error_mesg)
			{
				PSVR_MT_LOG_ERROR("MorpheusSensorProcessor::workerThreadFunc") << "HID ERROR: " << hidapi_err_mbs;
			}

			bWorking= false;
		}

		return bWorking;
    }

    // Multithreaded state
	const MorpheusHMDConfig m_cfg;
	hid_device *m_hidDevice;
	IHMDListener *m_hmdListener;

    // Worker thread state
    int m_nextPollSequenceNumber;
    struct MorpheusSensorData *m_rawHIDPacket;                        // Buffer to hold most recent MorpheusAPI tracking state
};

// -- private methods
static bool morpheus_open_usb_device(MorpheusUSBContext *morpheus_context);
static void morpheus_close_usb_device(MorpheusUSBContext *morpheus_context);
static bool morpheus_enable_tracking(MorpheusUSBContext *morpheus_context);
static bool morpheus_set_headset_power(MorpheusUSBContext *morpheus_context, bool bIsOn);
static bool morpheus_set_led_brightness(MorpheusUSBContext *morpheus_context, unsigned short led_bitmask, unsigned char intensity);
static bool morpheus_turn_off_processor_unit(MorpheusUSBContext *morpheus_context);
static bool morpheus_set_vr_mode(MorpheusUSBContext *morpheus_context, bool bIsOn);
static bool morpheus_set_cinematic_configuration(
	MorpheusUSBContext *morpheus_context,
	unsigned char ScreenDistance, unsigned char ScreenSize, unsigned char Brightness, unsigned char MicVolume, bool UnknownVRSetting);
static bool morpheus_send_command(MorpheusUSBContext *morpheus_context, MorpheusCommand &command);

// -- public interface
// -- Morpheus HMD Config
const int MorpheusHMDConfig::CONFIG_VERSION = 2;

const configuru::Config
MorpheusHMDConfig::writeToJSON()
{
    configuru::Config pt{
	    {"is_valid", is_valid},
	    {"version", MorpheusHMDConfig::CONFIG_VERSION},
	    {"disable_command_interface", disable_command_interface},
	    {"Calibration.Accel.X.k", accelerometer_gain.x},
	    {"Calibration.Accel.Y.k", accelerometer_gain.y},
	    {"Calibration.Accel.Z.k", accelerometer_gain.z},
	    {"Calibration.Accel.X.b", raw_accelerometer_bias.x},
	    {"Calibration.Accel.Y.b", raw_accelerometer_bias.y},
	    {"Calibration.Accel.Z.b", raw_accelerometer_bias.z},
	    {"Calibration.Accel.Variance", raw_accelerometer_variance},
	    {"Calibration.Gyro.X.k", gyro_gain.x},
	    {"Calibration.Gyro.Y.k", gyro_gain.y},
	    {"Calibration.Gyro.Z.k", gyro_gain.z},
	    {"Calibration.Gyro.X.b", raw_gyro_bias.x},
	    {"Calibration.Gyro.Y.b", raw_gyro_bias.y},
	    {"Calibration.Gyro.Z.b", raw_gyro_bias.z},
	    {"Calibration.Gyro.Variance", raw_gyro_variance},
	    {"Calibration.Gyro.Drift", raw_gyro_drift},
	    {"Calibration.Identity.Gravity.X", identity_gravity_direction.x},
	    {"Calibration.Identity.Gravity.Y", identity_gravity_direction.y},
	    {"Calibration.Identity.Gravity.Z", identity_gravity_direction.z},
	    {"Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a},
	    {"Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b},
	    {"Calibration.Orientation.Variance", orientation_variance},
	    {"Calibration.Time.MeanUpdateTime", mean_update_time_delta},
	    {"OrientationFilter.FilterType", orientation_filter_type},
	    {"PositionFilter.FilterType", position_filter_type},
	    {"PositionFilter.MaxVelocity", max_velocity},
	    {"prediction_time", prediction_time},
	    {"poll_timeout_ms", poll_timeout_ms}
    };

	writeTrackingColor(pt, tracking_color_id);

    return pt;
}

void
MorpheusHMDConfig::readFromJSON(const configuru::Config &pt)
{
    version = pt.get_or<int>("version", 0);

    if (version == MorpheusHMDConfig::CONFIG_VERSION)
    {
		is_valid = pt.get_or<bool>("is_valid", false);

		disable_command_interface= pt.get_or<bool>("disable_command_interface", disable_command_interface);

		prediction_time = pt.get_or<float>("prediction_time", 0.f);
		poll_timeout_ms = pt.get_or<long>("poll_timeout_ms", poll_timeout_ms);

		// Use the current accelerometer values (constructor defaults) as the default values
		accelerometer_gain.x = pt.get_or<float>("Calibration.Accel.X.k", accelerometer_gain.x);
		accelerometer_gain.y = pt.get_or<float>("Calibration.Accel.Y.k", accelerometer_gain.y);
		accelerometer_gain.z = pt.get_or<float>("Calibration.Accel.Z.k", accelerometer_gain.z);
		raw_accelerometer_bias.x = pt.get_or<float>("Calibration.Accel.X.b", raw_accelerometer_bias.x);
		raw_accelerometer_bias.y = pt.get_or<float>("Calibration.Accel.Y.b", raw_accelerometer_bias.y);
		raw_accelerometer_bias.z = pt.get_or<float>("Calibration.Accel.Z.b", raw_accelerometer_bias.z);
		raw_accelerometer_variance = pt.get_or<float>("Calibration.Accel.Variance", raw_accelerometer_variance);

		// Use the current gyroscope values (constructor defaults) as the default values
		gyro_gain.x = pt.get_or<float>("Calibration.Gyro.X.k", gyro_gain.x);
		gyro_gain.y = pt.get_or<float>("Calibration.Gyro.Y.k", gyro_gain.y);
		gyro_gain.z = pt.get_or<float>("Calibration.Gyro.Z.k", gyro_gain.z);
		raw_gyro_bias.x = pt.get_or<float>("Calibration.Gyro.X.b", raw_gyro_bias.x);
		raw_gyro_bias.y = pt.get_or<float>("Calibration.Gyro.Y.b", raw_gyro_bias.y);
		raw_gyro_bias.z = pt.get_or<float>("Calibration.Gyro.Z.b", raw_gyro_bias.z);
		raw_gyro_variance = pt.get_or<float>("Calibration.Gyro.Variance", raw_gyro_variance);
		raw_gyro_drift = pt.get_or<float>("Calibration.Gyro.Drift", raw_gyro_drift);

		position_variance_exp_fit_a = pt.get_or<float>("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
		position_variance_exp_fit_b = pt.get_or<float>("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

		orientation_variance = pt.get_or<float>("Calibration.Orientation.Variance", orientation_variance);

		mean_update_time_delta = pt.get_or<float>("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

		orientation_filter_type = pt.get_or<std::string>("OrientationFilter.FilterType", orientation_filter_type);

		position_filter_type = pt.get_or<std::string>("PositionFilter.FilterType", position_filter_type);
		max_velocity = pt.get_or<float>("PositionFilter.MaxVelocity", max_velocity);

		// Get the calibration direction for "down"
		identity_gravity_direction.x = pt.get_or<float>("Calibration.Identity.Gravity.X", identity_gravity_direction.x);
		identity_gravity_direction.y = pt.get_or<float>("Calibration.Identity.Gravity.Y", identity_gravity_direction.y);
		identity_gravity_direction.z = pt.get_or<float>("Calibration.Identity.Gravity.Z", identity_gravity_direction.z);

		// Read the tracking color
		tracking_color_id = static_cast<PSVRTrackingColorType>(readTrackingColor(pt));
    }
    else
    {
        PSVR_LOG_WARNING("MorpheusHMDConfig") <<
            "Config version " << version << " does not match expected version " <<
            MorpheusHMDConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- Morpheus HMD Sensor Frame -----
void MorpheusHMDSensorFrame::parse_data_input(
	const MorpheusHMDConfig *config,
	const MorpheusRawSensorFrame *data_input)
{
	short raw_seq = static_cast<short>((data_input->seq_frame[1] << 8) | data_input->seq_frame[0]);

	// Piece together the 12-bit accelerometer data 
	// rotate data 90degrees about Z so that sensor Y is up, flip X and Z)
	// +X - goes out the left of the headset
	// +Y - goes out the top of the headset
	// +Z - goes out the back of the headset
	short raw_accelX = static_cast<short>(((data_input->accel_y[1] << 8) | data_input->accel_y[0])) >> 4;						
	short raw_accelY = static_cast<short>(((data_input->accel_x[1] << 8) | data_input->accel_x[0])) >> 4;
	short raw_accelZ = -(static_cast<short>(((data_input->accel_z[1] << 8) | data_input->accel_z[0])) >> 4);

	// Piece together the 16-bit gyroscope data
	short raw_gyroYaw = static_cast<short>((data_input->gyro_yaw[1] << 8) | data_input->gyro_yaw[0]);
	short raw_gyroPitch = static_cast<short>((data_input->gyro_pitch[1] << 8) | data_input->gyro_pitch[0]);
	short raw_gyroRoll = -static_cast<short>((data_input->gyro_roll[1] << 8) | data_input->gyro_roll[0]);

	// Save the sequence number
	SequenceNumber = static_cast<int>(raw_seq);

	// Save the raw accelerometer values
	RawAccel.x = static_cast<int>(raw_accelX);
	RawAccel.y = static_cast<int>(raw_accelY);
	RawAccel.z = static_cast<int>(raw_accelZ);

	// Save the raw gyro values
	RawGyro.x = static_cast<int>(raw_gyroPitch);
	RawGyro.y = static_cast<int>(raw_gyroYaw);
	RawGyro.z = static_cast<int>(raw_gyroRoll);

	// calibrated_acc= (raw_acc - acc_bias) * acc_gain
	CalibratedAccel.x = (static_cast<float>(raw_accelX) - config->raw_accelerometer_bias.x) * config->accelerometer_gain.x;
	CalibratedAccel.y = (static_cast<float>(raw_accelY) - config->raw_accelerometer_bias.y) * config->accelerometer_gain.y;
	CalibratedAccel.z = (static_cast<float>(raw_accelZ) - config->raw_accelerometer_bias.z) * config->accelerometer_gain.z;

	// calibrated_gyro= (raw_gyro - gyro_bias) * gyro_gain
	CalibratedGyro.x = (static_cast<float>(raw_gyroPitch) - config->raw_gyro_bias.x) * config->gyro_gain.x;
	CalibratedGyro.y = (static_cast<float>(raw_gyroYaw) - config->raw_gyro_bias.y) * config->gyro_gain.y;
	CalibratedGyro.z = (static_cast<float>(raw_gyroRoll) - config->raw_gyro_bias.z) * config->gyro_gain.z;
}

// -- Morpheus HMD State -----
void MorpheusHMDSensorState::parse_data_input(
	const MorpheusHMDConfig *config, 
	const struct MorpheusSensorData *data_input)
{
	SensorFrames[0].parse_data_input(config, &data_input->imu_frame_0);
	SensorFrames[1].parse_data_input(config, &data_input->imu_frame_1);
}

// -- Morpheus HMD -----
MorpheusHMD::MorpheusHMD()
    : cfg()
    , USBContext(nullptr)
    , NextPollSequenceNumber(0)
    , InData(nullptr)
	, m_sensorProcessor(nullptr)
	, bIsTracking(false)
{
    USBContext = new MorpheusUSBContext;
    InData = new MorpheusSensorData;
	m_sensorProcessor = new MorpheusSensorProcessor(cfg);
}

MorpheusHMD::~MorpheusHMD()
{
    if (getIsOpen())
    {
        PSVR_LOG_ERROR("~MorpheusHMD") << "HMD deleted without calling close() first!";
    }

	delete m_sensorProcessor;
    delete InData;
    delete USBContext;
}

bool MorpheusHMD::open()
{
    HMDDeviceEnumerator enumerator(HMDDeviceEnumerator::CommunicationType_HID);
    bool success = false;

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

bool MorpheusHMD::open(
    const DeviceEnumerator *enumerator)
{
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    const char *cur_dev_path = pEnum->get_path();
    bool success = false;

    if (getIsOpen())
    {
        PSVR_LOG_WARNING("MorpheusHMD::open") << "MorpheusHMD(" << cur_dev_path << ") already open. Ignoring request.";
        success = true;
    }
    else
    {
		PSVR_LOG_INFO("MorpheusHMD::open") << "Opening MorpheusHMD(" << cur_dev_path << ").";
		// Load the config file
		std::string config_name("MorpheusHMDConfig");
		cfg = MorpheusHMDConfig(config_name);
        cfg.load();

		USBContext->device_identifier = cur_dev_path;

		// Open the sensor interface using HIDAPI
		USBContext->sensor_device_path = pEnum->get_hid_hmd_enumerator()->get_interface_path(MORPHEUS_SENSOR_INTERFACE);
		USBContext->sensor_device_handle = hid_open_path(USBContext->sensor_device_path.c_str());
		if (USBContext->sensor_device_handle != nullptr)
		{
			// Perform blocking reads in the IMU thread
			hid_set_nonblocking(USBContext->sensor_device_handle, 0);

			// Start the HID packet processing thread
			m_sensorProcessor->start(USBContext->sensor_device_handle, m_hmdListener);
		}

		// Open the command interface using libusb.
		// NOTE: Ideally we would use one usb library for both interfaces, but there are some complications.
		// A) The command interface uses the bulk transfer endpoint and HIDApi doesn't support that endpoint.
		// B) In Windows, libusb doesn't handle a high frequency of requests coming from two different threads well.
		// In this case, PS3EyeDriver is constantly sending bulk transfer requests in its own thread to get video frames.
		// If we started sending control transfer requests for the sensor data in the main thread at the same time
		// it can lead to a crash. It shouldn't, but this was a problem previously setting video feed properties
		// from the color config tool while a video feed was running.
		if (!cfg.disable_command_interface)
		{
			morpheus_open_usb_device(USBContext);
		}
		else
		{
			PSVR_LOG_WARNING("MorpheusHMD::open") << "Morpheus command interface is flagged as DISABLED.";
		}

        if (getIsOpen())  // Controller was opened and has an index
        {
			if (USBContext->usb_device_handle != k_invalid_usb_device_handle)
			{
				if (morpheus_set_headset_power(USBContext, true))
				{
					if (morpheus_enable_tracking(USBContext))
					{
						morpheus_set_led_brightness(USBContext, _MorpheusLED_ALL_FRONT, 0);
						morpheus_set_led_brightness(USBContext, _MorpheusLED_ALL_BACK, 50);
					}
				}
			}

			// Always save the config back out in case some defaults changed
			cfg.save();

            // Reset the polling sequence counter
            NextPollSequenceNumber = 0;

			success = true;
        }
        else
        {
            PSVR_LOG_ERROR("MorpheusHMD::open") << "Failed to open MorpheusHMD(" << cur_dev_path << ")";
			close();
        }
    }

    return success;
}

void MorpheusHMD::close()
{
    if (USBContext->sensor_device_handle != nullptr || USBContext->usb_device_handle != k_invalid_usb_device_handle)
    {
		if (USBContext->sensor_device_handle != nullptr)
		{
			// halt the HID packet processing thread
			m_sensorProcessor->stop();

			PSVR_LOG_INFO("MorpheusHMD::close") << "Closing MorpheusHMD sensor interface(" << USBContext->sensor_device_path << ")";
			hid_close(USBContext->sensor_device_handle);
		}

		if (USBContext->usb_device_handle != k_invalid_usb_device_handle)
		{
			PSVR_LOG_INFO("MorpheusHMD::close") << "Closing MorpheusHMD command interface";
			morpheus_set_headset_power(USBContext, false);
			morpheus_close_usb_device(USBContext);
		}

        USBContext->Reset();
        InData->Reset();
    }
    else
    {
        PSVR_LOG_INFO("MorpheusHMD::close") << "MorpheusHMD already closed. Ignoring request.";
    }
}

// Getters
bool
MorpheusHMD::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == getDeviceType())
    {
        const char *enumerator_path = pEnum->get_path();
        const char *dev_path = USBContext->device_identifier.c_str();

#ifdef _WIN32
        matches = _stricmp(dev_path, enumerator_path) == 0;
#else
        matches = strcmp(dev_path, enumerator_path) == 0;
#endif
    }

    return matches;
}

//bool
//MorpheusHMD::getIsReadyToPoll() const
//{
//    return (getIsOpen());
//}

std::string
MorpheusHMD::getUSBDevicePath() const
{
    return USBContext->sensor_device_path;
}

bool
MorpheusHMD::getIsOpen() const
{
    return USBContext->sensor_device_handle != nullptr && 
		(USBContext->usb_device_handle != k_invalid_usb_device_handle || cfg.disable_command_interface);
}

#if 0
IDeviceInterface::ePollResult
MorpheusHMD::poll()
{
	IHMDInterface::ePollResult result = IHMDInterface::_PollResultFailure;

	if (getIsOpen())
	{
		static const int k_max_iterations = 32;

		for (int iteration = 0; iteration < k_max_iterations; ++iteration)
		{
			// Attempt to read the next update packet from the controller
			int res = hid_read(USBContext->sensor_device_handle, (unsigned char*)InData, sizeof(MorpheusSensorData));

			if (res == 0)
			{
				// Device still in valid state
				result = (iteration == 0)
					? IHMDInterface::_PollResultSuccessNoData
					: IHMDInterface::_PollResultSuccessNewData;

				// No more data available. Stop iterating.
				break;
			}
			else if (res < 0)
			{
				char hidapi_err_mbs[256];
				bool valid_error_mesg = 
					Utility::convert_wcs_to_mbs(hid_error(USBContext->sensor_device_handle), hidapi_err_mbs, sizeof(hidapi_err_mbs));

				// Device no longer in valid state.
				if (valid_error_mesg)
				{
					PSVR_LOG_ERROR("PSVRController::readDataIn") << "HID ERROR: " << hidapi_err_mbs;
				}
				result = IHMDInterface::_PollResultFailure;

				// No more data available. Stop iterating.
				break;
			}
			else
			{
				// New data available. Keep iterating.
				result = IHMDInterface::_PollResultSuccessNewData;
			}

			// https://github.com/hrl7/node-psvr/blob/master/lib/psvr.js
			MorpheusHMDSensorState newState;

			// Increment the sequence for every new polling packet
			newState.PollSequenceNumber = NextPollSequenceNumber;
			++NextPollSequenceNumber;

			// Processes the IMU data
			newState.parse_data_input(&cfg, InData);

			// Make room for new entry if at the max queue size
			if (HMDStates.size() >= MORPHEUS_HMD_STATE_BUFFER_MAX)
			{
				HMDStates.erase(HMDStates.begin(), HMDStates.begin() + HMDStates.size() - MORPHEUS_HMD_STATE_BUFFER_MAX);
			}

			HMDStates.push_back(newState);
		}
	}

	return result;
}
#endif 

void
MorpheusHMD::getTrackingShape(PSVRTrackingShape &outTrackingShape) const
{
	outTrackingShape.shape_type = PSVRTrackingShape_PointCloud;
    outTrackingShape.shape.pointcloud.points[0] = {0.00f, 0.00f, 0.00f}; // 0
    outTrackingShape.shape.pointcloud.points[1] = {7.25f, 4.05f, 3.75f}; // 1
    outTrackingShape.shape.pointcloud.points[2] = {9.05f, 0.00f, 9.65f}; // 2
    outTrackingShape.shape.pointcloud.points[3] = {7.25f, -4.05f, 3.75f}; // 3
    outTrackingShape.shape.pointcloud.points[4] = {-7.25f, 4.05f, 3.75f}; // 4
    outTrackingShape.shape.pointcloud.points[5] = {-9.05f, 0.00f, 9.65f}; // 5
    outTrackingShape.shape.pointcloud.points[6] = {-7.25f, -4.05f, 3.75f}; // 6
    outTrackingShape.shape.pointcloud.points[7] = {5.65f, -1.07f, 27.53f}; // 7
    outTrackingShape.shape.pointcloud.points[8] = {-5.65f, -1.07f, 27.53f}; // 8
	outTrackingShape.shape.pointcloud.point_count = 9;
}

bool 
MorpheusHMD::setTrackingColorID(const PSVRTrackingColorType tracking_color_id)
{
    return false;
}

bool 
MorpheusHMD::getTrackingColorID(PSVRTrackingColorType &out_tracking_color_id) const
{
	out_tracking_color_id = PSVRTrackingColorType_Blue;
	return true;
}

float 
MorpheusHMD::getPredictionTime() const
{
	return getConfig()->prediction_time;
}

void 
MorpheusHMD::setHMDListener(IHMDListener *listener)
{
	m_hmdListener= listener;
}

//long MorpheusHMD::getMaxPollFailureCount() const
//{
//    return cfg.max_poll_failure_count;
//}

void MorpheusHMD::setTrackingEnabled(bool bEnable)
{
	if (USBContext->usb_device_handle != k_invalid_usb_device_handle)
	{
		if (!bIsTracking && bEnable)
		{
			morpheus_set_led_brightness(USBContext, _MorpheusLED_ALL, 50);
			bIsTracking = true;
		}
		else if (bIsTracking && !bEnable)
		{
			morpheus_set_led_brightness(USBContext, _MorpheusLED_ALL_FRONT, 0);
			morpheus_set_led_brightness(USBContext, _MorpheusLED_ALL_BACK, 50);
			bIsTracking = false;
		}
	}
}

//-- private morpheus commands ---
static bool morpheus_open_usb_device(
	MorpheusUSBContext *morpheus_context)
{
	bool bSuccess = false;

    USBDeviceEnumerator* usb_device_enumerator= usb_device_enumerator_allocate();

    if (usb_device_enumerator != nullptr)
    {
        while (!bSuccess && usb_device_enumerator_is_valid(usb_device_enumerator))
        {
            USBDeviceFilter deviceInfo;
            if (usb_device_enumerator_get_filter(usb_device_enumerator, deviceInfo) &&
                deviceInfo.vendor_id == MORPHEUS_VENDOR_ID &&
                deviceInfo.product_id == MORPHEUS_PRODUCT_ID &&
                (deviceInfo.interface_mask & (1 << MORPHEUS_COMMAND_INTERFACE)) > 0)
            {
                bSuccess= true;
                break;
            }
            else
            {
                usb_device_enumerator_next(usb_device_enumerator);
            }
        }        
    }

    if (bSuccess)
    {
	    const t_usb_device_handle usb_device_handle = usb_device_open(usb_device_enumerator, MORPHEUS_COMMAND_INTERFACE);
	    if (usb_device_handle != k_invalid_usb_device_handle)
	    {
		    PSVR_LOG_INFO("morpeus_open_usb_devicen") << "  Successfully opened USB handle " << usb_device_handle;

            char szPathBuffer[512];
            if (usb_device_enumerator_get_path(usb_device_enumerator, szPathBuffer, sizeof(szPathBuffer)))
            {
                morpheus_context->usb_device_path = szPathBuffer;
            }

		    morpheus_context->usb_device_handle = usb_device_handle;
	    }
	    else
	    {
		    PSVR_LOG_ERROR("morpeus_open_usb_device") << "  Failed to open USB handle " << usb_device_handle;
	    }
    }
    else
    {
        PSVR_LOG_ERROR("morpeus_open_usb_device") << "  Failed to find Morpheus USB command interface (VID:0x054c, PID: 0x09af, Interface: 5)";
    }

    if (usb_device_enumerator != nullptr)
    {
        usb_device_enumerator_free(usb_device_enumerator);
    }

	return bSuccess;
}

static void morpheus_close_usb_device(
	MorpheusUSBContext *morpheus_context)
{
	if (morpheus_context->usb_device_handle != k_invalid_usb_device_handle)
	{
		PSVR_LOG_INFO("morpheus_close_usb_device") << "Closing PSNaviController(" << morpheus_context->usb_device_path << ")";
		usb_device_close(morpheus_context->usb_device_handle);
		morpheus_context->usb_device_handle = k_invalid_usb_device_handle;
	}
}

static bool morpheus_enable_tracking(
	MorpheusUSBContext *morpheus_context)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_EnableTracking;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 8;
	((int*)command.payload)[0] = 0xFFFFFF00; // Magic numbers!  Turns on the VR mode and the blue lights on the front
	((int*)command.payload)[1] = 0x00000000;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_set_headset_power(
	MorpheusUSBContext *morpheus_context,
	bool bIsOn)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetHeadsetPower;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = bIsOn ? 0x00000001 : 0x00000000;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_set_led_brightness(
	MorpheusUSBContext *morpheus_context,
	unsigned short led_bitmask,
	unsigned char intensity)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetLEDBrightness;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 16;
	((unsigned short*)command.payload)[0] = led_bitmask;

	unsigned short mask = led_bitmask;
	for (int led_index = 0; led_index < 9; ++led_index)
	{
		command.payload[2 + led_index] = ((mask & 0x001) > 0) ? intensity : 0;
		mask = mask >> 1;
	}

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_turn_off_processor_unit(
	MorpheusUSBContext *morpheus_context)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_TurnOffProcessorUnit;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = 0x00000001;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_set_vr_mode(
	MorpheusUSBContext *morpheus_context,
	bool bIsOn)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetVRMode;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = bIsOn ? 0x00000001 : 0x00000000;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_set_cinematic_configuration(
	MorpheusUSBContext *morpheus_context,
	unsigned char ScreenDistance, 
	unsigned char ScreenSize, 
	unsigned char Brightness, 
	unsigned char MicVolume, 
	bool UnknownVRSetting)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetCinematicConfiguration;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 16;
	command.payload[1] = ScreenSize;
	command.payload[2] = ScreenDistance;
	command.payload[10] = Brightness;
	command.payload[11] = MicVolume;
	command.payload[14] = UnknownVRSetting ? 0 : 1;

	return morpheus_send_command(morpheus_context, command);
}

static bool morpheus_send_command(
	MorpheusUSBContext *morpheus_context,
	MorpheusCommand &command)
{
    bool bSuccess= false;

	if (morpheus_context->usb_device_handle != k_invalid_usb_device_handle)
	{
		//###HipsterSloth $TODO Don't hard code the endpoint address.
		// We should instead specify a direction (IN or OUT)
		// The direction plus the transfer type (interrupt/bulk) is enough to
		// look up the endpoint address on the USBApi side.
        const int endpointAddress = 0x04;
		//const int endpointAddress =
		//	(morpheus_context->usb_device_descriptor->interface[MORPHEUS_COMMAND_INTERFACE]
		//		.altsetting[0]
		//		.endpoint[0]
		//		.bEndpointAddress) & ~MORPHEUS_ENDPOINT_IN;

        const size_t command_length = static_cast<size_t>(command.header.length) + sizeof(command.header);
	    int result = 0;

	    USBTransferRequest transfer_request;
	    transfer_request.request_type = _USBRequestType_InterruptTransfer;

	    USBRequestPayload_InterruptTransfer &interrupt_transfer = transfer_request.payload.interrupt_transfer;
	    interrupt_transfer.usb_device_handle = morpheus_context->usb_device_handle;
	    interrupt_transfer.endpoint = endpointAddress;
	    interrupt_transfer.length = static_cast<unsigned int>(command_length);
	    interrupt_transfer.timeout = INTERRUPT_TRANSFER_TIMEOUT;

		static_assert(sizeof(transfer_request.payload.bulk_transfer.data) >= sizeof(MorpheusCommand), "bulk_transfer max payload too small for MorpheusCommand");
		memcpy(transfer_request.payload.interrupt_transfer.data, (unsigned char *)&command, command_length);

	    USBTransferResult transfer_result = usb_device_submit_transfer_request_blocking(transfer_request);
	    assert(transfer_result.result_type == _USBRequestType_InterruptTransfer);

	    if (transfer_result.payload.interrupt_transfer.result_code == _USBResultCode_Completed)
	    {
		    result = transfer_result.payload.interrupt_transfer.dataLength;
            bSuccess= true;
	    }
	    else
	    {
		    const char * error_text = usb_device_get_error_string(transfer_result.payload.interrupt_transfer.result_code);
		    PSVR_LOG_ERROR("morpheus_send_command") << "interrupt transfer failed with error: " << error_text;
		    result = -static_cast<int>(transfer_result.payload.interrupt_transfer.result_code);
	    }
	}


    return bSuccess;
}
