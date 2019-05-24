//-- includes -----
#include "MorpheusHMD.h"
#include "DeviceInterface.h"
#include "DeviceManager.h"
#include "HMDDeviceEnumerator.h"
#include "HMDHidDeviceEnumerator.h"
#include "HMDUsbDeviceEnumerator.h"
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
#ifdef _WIN32 //UDP
#include <Winsock2.h> 
#pragma comment(lib,"Ws2_32.lib")
#endif

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
    std::string sensor_hid_path;
	hid_device *sensor_hid_handle;

	// Sensor Interface USB state
    std::string sensor_usb_device_path;
	t_usb_device_handle sensor_usb_device_handle;

	// Command Interface USB state
    std::string command_usb_device_path;
	t_usb_device_handle command_usb_device_handle;

    MorpheusUSBContext()
    {
        Reset();
    }

    void Reset()
    {
		device_identifier = "";
        sensor_hid_path = "";
		sensor_hid_handle = nullptr;
        sensor_usb_device_path= "";
        sensor_usb_device_handle= k_invalid_usb_device_handle;
        command_usb_device_path= "";
        command_usb_device_handle= k_invalid_usb_device_handle;
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
	unsigned char seq_frame[3];
	unsigned char seq_padding;

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
			unsigned char timer : 1; // appears to alternate every second
		};
	} headsetFlags;								// byte 8

	unsigned char unkFlags;     				// byte 9
	
	unsigned char unk2[6];						// byte 10-15

	MorpheusRawSensorFrame imu_frame_0;         // byte 16-31
	MorpheusRawSensorFrame imu_frame_1;         // byte 32-47

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

class IMorpheusSensorProcessor
{
public:
    virtual ~IMorpheusSensorProcessor() {}

    virtual void start(class MorpheusUSBContext *USBContext, IHMDListener *hmd_listener) = 0;
    virtual void stop() = 0;
};

struct MorpheusUDP
{
	class MorpheusUSBContext *ctx;
	const MorpheusHMDConfig *cfg;
	const MorpheusHMDSensorFrame *st;
	const MorpheusSensorData *st2;

	~MorpheusUDP(); 
};

class MorpheusHIDSensorProcessor : public WorkerThread, public IMorpheusSensorProcessor
{
public:
	MorpheusHIDSensorProcessor(const MorpheusHMDConfig &cfg) 
		: WorkerThread("MorpheusSensorProcessor")
		, m_cfg(cfg)
		, m_hidDevice(nullptr)
		, m_nextPollSequenceNumber(0)
		, m_rawHIDPacket(nullptr)
		, m_ctx(nullptr) //UDP
	{
		m_rawHIDPacket = new MorpheusSensorData;
	}

	virtual ~MorpheusHIDSensorProcessor()
	{
		delete m_rawHIDPacket;
	}

    void start(class MorpheusUSBContext *USBContext, IHMDListener *hmd_listener) override
    {
		if (!hasThreadStarted())
		{
			m_ctx = USBContext; //UDP

			m_hidDevice= USBContext->sensor_hid_handle;
			m_hmdListener= hmd_listener;
			WorkerThread::startThread();
		}
    }

	void stop() override
	{
		WorkerThread::stopThread();

		MorpheusUDP udp = {m_ctx,&m_cfg}; // Close		
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

			if(m_cfg.UDP_status_port>0)
			{
				MorpheusUDP udp = {m_ctx,&m_cfg,&newState.SensorFrames[0],m_rawHIDPacket};
			}
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

	class MorpheusUSBContext *m_ctx; //UDP
};

class MorpheusUSBSensorProcessor : public IMorpheusSensorProcessor
{
public:
	MorpheusUSBSensorProcessor(const MorpheusHMDConfig &cfg) 
		: m_cfg(cfg)		
		, m_usbDeviceHandle(k_invalid_usb_device_handle)
        , m_hmdListener(nullptr)
        , m_bStartedStream(false)
		, m_nextPollSequenceNumber(0)
		, m_ctx(nullptr) //UDP
	{
	}

    void start(class MorpheusUSBContext *USBContext, IHMDListener *hmd_listener) override
    {
        if (!m_bStartedStream)
        {
			m_ctx = USBContext; //UDP

		    m_usbDeviceHandle= USBContext->sensor_usb_device_handle;
		    m_hmdListener= hmd_listener;

		    // Send a request to the USBDeviceManager to start a interrupr transfer stream for sensor data.
		    // This will spin up a thread in the USB manager for reading the incoming USB packets.
		    // The usbInterruptTransferCallback_usbThread() callback will be executed on the usb worker thread.
		    USBTransferRequest request(eUSBTransferRequestType::_USBRequestType_StartTransferBundle);
		    request.payload.start_transfer_bundle.usb_device_handle= m_usbDeviceHandle;
            request.payload.start_transfer_bundle.transfer_type= _USBTransferBundleType_Interrupt;
		    request.payload.start_transfer_bundle.bAutoResubmit= true;
		    request.payload.start_transfer_bundle.in_flight_transfer_packet_count= 1;
		    request.payload.start_transfer_bundle.transfer_packet_size= sizeof(MorpheusSensorData);
		    request.payload.start_transfer_bundle.on_data_callback= usbInterruptTransferCallback_usbThread;
		    request.payload.start_transfer_bundle.transfer_callback_userdata= this;

		    USBTransferResult result= usb_device_submit_transfer_request_blocking(request);
		    PSVR_LOG_INFO("MorpheusUSBSensorProcessor::start - transfer bundle start result: ") << usb_device_get_error_string(result.payload.bulk_transfer_bundle.result_code); 
		    if (result.result_type == eUSBTransferRequestType::_USBRequestType_StartTransferBundle)
		    {
			    m_bStartedStream= true;
		    }
        }
    }

	void stop() override
	{
        if (m_bStartedStream)
        {
		    // Send a request to the USBDeviceManager to stop the interrupt transfer stream for sensor data
		    USBTransferRequest request(eUSBTransferRequestType::_USBRequestType_CancelTransferBundle);
		    request.payload.cancel_transfer_bundle.usb_device_handle= m_usbDeviceHandle;

		    // This will block until the processing USB packet processing thread exits
		    USBTransferResult result= usb_device_submit_transfer_request_blocking(request);
		    assert(result.result_type == eUSBTransferResultType::_USBResultType_TransferBundle);
		    PSVR_LOG_INFO("MorpheusUSBSensorProcessor::stop - transfer bundle stop result: ") << usb_device_get_error_string(result.payload.bulk_transfer_bundle.result_code); 

            m_bStartedStream= false;

			MorpheusUDP udp = {m_ctx,&m_cfg}; // Close
        }
	}

protected:
    static void usbInterruptTransferCallback_usbThread(unsigned char *packet_data, int packet_length, void *userdata)
    {
        MorpheusUSBSensorProcessor *processor= reinterpret_cast<MorpheusUSBSensorProcessor *>(userdata);

        if (packet_length >= sizeof(MorpheusSensorData))
        {
            processor->onNewSensorData_usbThread(reinterpret_cast<struct MorpheusSensorData *>(packet_data));
        }
    }

    void onNewSensorData_usbThread(struct MorpheusSensorData *m_rawUSBPacket)
    {
		MorpheusHMDSensorState newState;

		// Increment the sequence for every new polling packet
		newState.PollSequenceNumber = m_nextPollSequenceNumber;
		++m_nextPollSequenceNumber;

		// Processes the IMU data
		newState.parse_data_input(&m_cfg, m_rawUSBPacket);
				
		m_hmdListener->notifySensorDataReceived(&newState);

		if(m_cfg.UDP_status_port>0)
		{
			MorpheusUDP udp = {m_ctx,&m_cfg,&newState.SensorFrames[0],m_rawUSBPacket};
		}
    }

    // Multithreaded state
	const MorpheusHMDConfig m_cfg;
	t_usb_device_handle m_usbDeviceHandle;
	IHMDListener *m_hmdListener;
    bool m_bStartedStream;

    // Worker thread state
    int m_nextPollSequenceNumber;

	class MorpheusUSBContext *m_ctx; //UDP
};

// -- private methods
static bool morpheus_open_usb_command_interface(MorpheusUSBContext *morpheus_context, USBDeviceEnumerator* usb_device_enumerator);
static void morpheus_close_usb_command_interface(MorpheusUSBContext *morpheus_context);
static bool morpheus_enable_tracking(MorpheusUSBContext *morpheus_context, bool async=false);
static bool morpheus_set_headset_power(MorpheusUSBContext *morpheus_context, bool bIsOn, bool async=false);
static bool morpheus_set_led_brightness(MorpheusUSBContext *morpheus_context, unsigned short led_bitmask, unsigned char intensity, bool async=false);
static bool morpheus_turn_off_processor_unit(MorpheusUSBContext *morpheus_context, bool async=false);
static bool morpheus_set_vr_mode(MorpheusUSBContext *morpheus_context, bool bIsOn, bool async=false);
static bool morpheus_set_cinematic_configuration(
	MorpheusUSBContext *morpheus_context,
	unsigned char ScreenDistance, unsigned char ScreenSize, unsigned char Brightness, unsigned char MicVolume, bool UnknownVRSetting, bool async=false);
static bool morpheus_send_command(MorpheusUSBContext *morpheus_context, MorpheusCommand &command, bool async);

// -- public interface
// -- Morpheus HMD Config
const int MorpheusHMDConfig::CONFIG_VERSION = 2;

const configuru::Config
MorpheusHMDConfig::writeToJSON()
{
	configuru::Config pt{
		{"is_valid", is_valid},
		{"version", MorpheusHMDConfig::CONFIG_VERSION},
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
		{"poll_timeout_ms", poll_timeout_ms},
		{"UDP_status_port",UDP_status_port},
		{"UDP_command_port",UDP_command_port},
		{"screen_size",screen_size},
		{"screen_distance",screen_distance},
		{"screen_brightness",screen_brightness},
		{"mic_volume",mic_volume},
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

		// PSVRToolbox emulation
		UDP_status_port = pt.get_or<int>("UDP_status_port",UDP_status_port);
		UDP_command_port = pt.get_or<int>("UDP_command_port",UDP_command_port);
		screen_size = pt.get_or<int>("screen_size",screen_size);
		screen_distance = pt.get_or<int>("screen_distance",screen_distance);
		screen_brightness = pt.get_or<int>("screen_brightness",screen_brightness);
		mic_volume = pt.get_or<int>("mic_volume",mic_volume);
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
	//short raw_seq = static_cast<short>((data_input->seq_frame[1] << 8) | data_input->seq_frame[0]);
	int raw_seq = static_cast<int>((data_input->seq_frame[2] << 16) | (data_input->seq_frame[1] << 8) | data_input->seq_frame[0]);

	// Piece together the 12-bit accelerometer data 
	// rotate data 90degrees about Z so that sensor Y is up, flip X and Z)
	// +X - goes out the left of the headset
	// +Y - goes out the top of the headset
	// +Z - goes out the back of the headset
	short raw_accelX = static_cast<short>(((data_input->accel_y[1] << 8) | data_input->accel_y[0])) >> 4;						
	short raw_accelY = static_cast<short>(((data_input->accel_x[1] << 8) | data_input->accel_x[0])) >> 4;
	//- ?
	short raw_accelZ = -(static_cast<short>(((data_input->accel_z[1] << 8) | data_input->accel_z[0])) >> 4);

	// Piece together the 16-bit gyroscope data
	short raw_gyroYaw = static_cast<short>((data_input->gyro_yaw[1] << 8) | data_input->gyro_yaw[0]);
	short raw_gyroPitch = static_cast<short>((data_input->gyro_pitch[1] << 8) | data_input->gyro_pitch[0]);
	//- ?
	short raw_gyroRoll = -static_cast<short>((data_input->gyro_roll[1] << 8) | data_input->gyro_roll[0]);

	// Save the sequence number
	SequenceNumber = static_cast<int>(raw_seq);

	// Save the raw accelerometer values
	RawAccel.x = static_cast<int>(raw_accelX); //y
	RawAccel.y = static_cast<int>(raw_accelY); //x
	RawAccel.z = static_cast<int>(raw_accelZ); //-

	// Save the raw gyro values
	RawGyro.x = static_cast<int>(raw_gyroPitch); //y
	RawGyro.y = static_cast<int>(raw_gyroYaw); //x
	RawGyro.z = static_cast<int>(raw_gyroRoll); //-

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
	m_sensorProcessor = nullptr;
}

MorpheusHMD::~MorpheusHMD()
{
    if (getIsOpen())
    {
        PSVR_LOG_ERROR("~MorpheusHMD") << "HMD deleted without calling close() first!";
    }

    if (m_sensorProcessor != nullptr)
    {
    	delete m_sensorProcessor;
    }

    delete InData;
    delete USBContext;
}

bool MorpheusHMD::open()
{
    HMDDeviceEnumerator enumerator(HMDDeviceEnumerator::CommunicationType_ALL);
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

        // If the enumerator found a HID driver for the USB sensor interface, use that API
        if (pEnum->get_api_type() == HMDDeviceEnumerator::CommunicationType_HID)
        {
		    // Open the sensor interface using HIDAPI
		    USBContext->sensor_hid_path = pEnum->get_hmd_hid_enumerator()->get_hid_interface_path();
		    USBContext->sensor_hid_handle = hid_open_path(USBContext->sensor_hid_path.c_str());
		    if (USBContext->sensor_hid_handle != nullptr)
		    {
			    // Perform blocking reads in the IMU thread
			    hid_set_nonblocking(USBContext->sensor_hid_handle, 0);

                // Open the command interface to the morpheus
                if (morpheus_open_usb_command_interface(USBContext, nullptr))
                {
                    // Start the HID packet processing thread
                    m_sensorProcessor = new MorpheusHIDSensorProcessor(cfg);
                    m_sensorProcessor->start(USBContext, m_hmdListener);
                }
		    }
        }
        // Otherwise fall back to using interrupt transfers over USB for the sensor data
        else if (pEnum->get_api_type() == HMDDeviceEnumerator::CommunicationType_USB)
        {             
            USBContext->sensor_usb_device_path= pEnum->get_hmd_usb_enumerator()->get_path();
            USBContext->sensor_usb_device_handle= usb_device_open(
                pEnum->get_hmd_usb_enumerator()->get_usb_device_enumerator(), 
                MORPHEUS_SENSOR_INTERFACE);

            if (USBContext->sensor_usb_device_handle != k_invalid_usb_device_handle)
            {
                // Open the command interface to the morpheus
                if (morpheus_open_usb_command_interface(
                    USBContext,
                    pEnum->get_hmd_usb_enumerator()->get_usb_device_enumerator()))
                {
                    // Start the HID packet processing thread
                    m_sensorProcessor = new MorpheusUSBSensorProcessor(cfg);
                    m_sensorProcessor->start(USBContext, m_hmdListener);
                }
            }
        }

        if (getIsOpen())
        {
			if (USBContext->command_usb_device_handle != k_invalid_usb_device_handle)
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
    if (USBContext->sensor_hid_handle != nullptr || 
        USBContext->sensor_usb_device_handle != k_invalid_usb_device_handle ||
        USBContext->command_usb_device_handle != k_invalid_usb_device_handle)
    {
		if (USBContext->sensor_hid_handle != nullptr)
		{
			// halt the HID packet processing thread
			m_sensorProcessor->stop();

			PSVR_LOG_INFO("MorpheusHMD::close") << "Closing MorpheusHMD sensor HID interface(" << USBContext->sensor_hid_path << ")";
			hid_close(USBContext->sensor_hid_handle);
		}

		if (USBContext->sensor_usb_device_handle != k_invalid_usb_device_handle)
		{
			// halt the HID packet processing thread
			m_sensorProcessor->stop();

            PSVR_LOG_INFO("MorpheusHMD::close") << "Closing MorpheusHMD sensor USB interface(" << USBContext->sensor_usb_device_path << ")";
		    usb_device_close(USBContext->sensor_usb_device_handle);
		    USBContext->sensor_usb_device_handle = k_invalid_usb_device_handle;
		}

		if (USBContext->command_usb_device_handle != k_invalid_usb_device_handle)
		{
			PSVR_LOG_INFO("MorpheusHMD::close") << "Closing MorpheusHMD command interface";
			morpheus_set_headset_power(USBContext, false);
			morpheus_close_usb_command_interface(USBContext);
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

std::string
MorpheusHMD::getUSBDevicePath() const
{
    return USBContext->sensor_hid_path;
}

bool
MorpheusHMD::getIsOpen() const
{
    return 
        (USBContext->sensor_hid_handle != nullptr  || USBContext->sensor_usb_device_handle != k_invalid_usb_device_handle) && 
        USBContext->command_usb_device_handle != k_invalid_usb_device_handle;
}

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

	//###HipsterSloth $HACK Cheezy way to approximate normals on PSVR headset LEDs
	PSVRVector3f center= {0.f, 0.f, 12.f}; 
    for (int index = 0; index < 9; ++index)
    {
		PSVRVector3f &point= outTrackingShape.shape.pointcloud.points[index];
		PSVRVector3f normal= PSVR_Vector3fSubtract(&point, &center);

		outTrackingShape.shape.pointcloud.normals[index]=
			PSVR_Vector3fNormalizeWithDefault(&normal, k_PSVR_float_vector3_zero);
	}
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
	if (USBContext->command_usb_device_handle != k_invalid_usb_device_handle)
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
static bool morpheus_open_usb_command_interface(
	MorpheusUSBContext *morpheus_context,
    USBDeviceEnumerator* usb_device_enumerator)
{
	bool bSuccess = true;

    USBDeviceEnumerator* local_usb_device_enumerator= nullptr;

    // If an enumerator to a valid USB device wasn't provided,
    // go ahead and create a local one now
    if (usb_device_enumerator == nullptr)
    {
        local_usb_device_enumerator= usb_device_enumerator_allocate();
        usb_device_enumerator= local_usb_device_enumerator;

        bSuccess = false;

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
    }

    if (bSuccess)
    {
	    const t_usb_device_handle usb_device_handle = usb_device_open(usb_device_enumerator, MORPHEUS_COMMAND_INTERFACE);
	    if (usb_device_handle != k_invalid_usb_device_handle)
	    {
		    PSVR_LOG_INFO("morpeus_open_usb_device") << "  Successfully opened USB handle " << usb_device_handle.unique_id;

            char szPathBuffer[512];
            if (usb_device_enumerator_get_path(usb_device_enumerator, szPathBuffer, sizeof(szPathBuffer)))
            {
                morpheus_context->command_usb_device_path = szPathBuffer;
            }

		    morpheus_context->command_usb_device_handle = usb_device_handle;
	    }
	    else
	    {
		    PSVR_LOG_ERROR("morpeus_open_usb_device") << "  Failed to open USB handle " << usb_device_handle.unique_id;
	    }
    }
    else
    {
        PSVR_LOG_ERROR("morpeus_open_usb_device") << "  Failed to find Morpheus USB command interface (VID:0x054c, PID: 0x09af, Interface: 5)";
    }

    if (local_usb_device_enumerator != nullptr)
    {
        usb_device_enumerator_free(local_usb_device_enumerator);
    }

	return bSuccess;
}

static void morpheus_close_usb_command_interface(
	MorpheusUSBContext *morpheus_context)
{
	if (morpheus_context->command_usb_device_handle != k_invalid_usb_device_handle)
	{
		PSVR_LOG_INFO("morpheus_close_usb_device") << "Closing Morpheus USB command interface (" << morpheus_context->command_usb_device_path << ")";
		usb_device_close(morpheus_context->command_usb_device_handle);
		morpheus_context->command_usb_device_handle = k_invalid_usb_device_handle;
	}
}

static bool morpheus_enable_tracking(
	MorpheusUSBContext *morpheus_context, bool async)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_EnableTracking;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 8;
	((int*)command.payload)[0] = 0xFFFFFF00; // Magic numbers!  Turns on the VR mode and the blue lights on the front
	((int*)command.payload)[1] = 0x00000000;

	return morpheus_send_command(morpheus_context, command,async);
}

static bool morpheus_set_headset_power(
	MorpheusUSBContext *morpheus_context, bool bIsOn, bool async)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetHeadsetPower;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = bIsOn ? 0x00000001 : 0x00000000;

	return morpheus_send_command(morpheus_context, command,async);
}

static bool morpheus_set_led_brightness(
	MorpheusUSBContext *morpheus_context,
	unsigned char (&intensity)[9], 
	bool async)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetLEDBrightness;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 16;

	unsigned short led_bitmask = 0;
	for (int led_index = 0; led_index < 9; ++led_index)
	{
		if(intensity[led_index]) led_bitmask|=1<<led_index;
	}
	((unsigned short*)command.payload)[0] = led_bitmask;
	memcpy(command.payload+2,intensity,9);

	return morpheus_send_command(morpheus_context,command,async);
}
static bool morpheus_set_led_brightness(
	MorpheusUSBContext *morpheus_context,
	unsigned short led_bitmask,
	unsigned char intensity, bool async)
{
	unsigned char i[9];
	for (int led_index = 0; led_index < 9; ++led_index)
	{
		i[led_index] = led_bitmask&1?intensity:0;
		led_bitmask>>=1;
	}

	return morpheus_set_led_brightness(morpheus_context,i,async);
}

static bool morpheus_turn_off_processor_unit(
	MorpheusUSBContext *morpheus_context, bool async)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_TurnOffProcessorUnit;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = 0x00000001;

	return morpheus_send_command(morpheus_context, command,async);
}

static bool morpheus_set_vr_mode(
	MorpheusUSBContext *morpheus_context, bool bIsOn, bool async)
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetVRMode;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 4;
	((int*)command.payload)[0] = bIsOn ? 0x00000001 : 0x00000000;

	return morpheus_send_command(morpheus_context, command,async);
}

static bool morpheus_set_cinematic_configuration(
	MorpheusUSBContext *morpheus_context,
	unsigned char ScreenDistance, 
	unsigned char ScreenSize, 
	unsigned char Brightness, 
	unsigned char MicVolume, 
	bool UnknownVRSetting, bool async) // Is this necessary?
{
	MorpheusCommand command = { {0} };
	command.header.request_id = Morpheus_Req_SetCinematicConfiguration;
	command.header.magic = MORPHEUS_COMMAND_MAGIC;
	command.header.length = 16;
	command.payload[1] = ScreenSize;
	command.payload[2] = ScreenDistance;
	command.payload[10] = Brightness;
	command.payload[11] = MicVolume;
	// https://github.com/gusmanb/PSVRFramework/wiki/Report-0x21---Display-settings
	// Social Screen Frequency? (0=60Hz, 1=58Hz? Intermittent) 
	command.payload[14] = UnknownVRSetting ? 0 : 1;

	return morpheus_send_command(morpheus_context,command,async);
}

static void morpheus_send_command_success(const USBTransferResult &transfer_result)
{
	if (transfer_result.payload.interrupt_transfer.result_code == _USBResultCode_Completed)
	{
		//result = transfer_result.payload.interrupt_transfer.dataLength;
        //return true; //bSuccess= true;
	}
	else
	{
		const char * error_text = usb_device_get_error_string(transfer_result.payload.interrupt_transfer.result_code);
		PSVR_LOG_ERROR("morpheus_send_command") << "interrupt transfer failed with error: " << error_text;
		//result = -static_cast<int>(transfer_result.payload.interrupt_transfer.result_code);
		//return false;
	}
}

static bool morpheus_send_command(
	MorpheusUSBContext *morpheus_context,
	MorpheusCommand &command, bool async=false)
{
    bool bSuccess= false;

	if (morpheus_context->command_usb_device_handle != k_invalid_usb_device_handle)
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
	  
		//int result = 0; //UNUSED

	    USBTransferRequest transfer_request;
	    transfer_request.request_type = _USBRequestType_InterruptTransfer;

	    USBRequestPayload_InterruptTransfer &interrupt_transfer = transfer_request.payload.interrupt_transfer;
	    interrupt_transfer.usb_device_handle = morpheus_context->command_usb_device_handle;
	    interrupt_transfer.endpoint = endpointAddress;
	    interrupt_transfer.length = static_cast<unsigned int>(command_length);
	    interrupt_transfer.timeout = INTERRUPT_TRANSFER_TIMEOUT;

		static_assert(sizeof(transfer_request.payload.bulk_transfer.data) >= sizeof(MorpheusCommand), "bulk_transfer max payload too small for MorpheusCommand");
		memcpy(transfer_request.payload.interrupt_transfer.data, (unsigned char *)&command, command_length);

		USBTransferResult transfer_result; if(!async)
		{
			transfer_result = usb_device_submit_transfer_request_blocking(transfer_request);
			assert(transfer_result.result_type == _USBRequestType_InterruptTransfer);
			if(!transfer_result.payload.interrupt_transfer.result_code!=_USBResultCode_Completed)
			{
				morpheus_send_command_success(transfer_result);
			}
			else bSuccess = true;
		}
		else bSuccess = usb_device_submit_transfer_request_async(transfer_request,morpheus_send_command_success); 	    
	}

    return bSuccess;
}

struct MorpheusUDP_pair : std::pair<const char*,int>
{
	typedef std::vector<MorpheusUDP_pair*> vec;

	MorpheusUDP_pair(const char *f, int s, vec &v):pair(f,s)
	{
		v.push_back(this); 
	}

	void operator=(int v){ second = v; }
};

namespace MorpheusUDP_PSVRToolbox_status
{
	static MorpheusUDP_pair::vec vec;
	#define _(x) static MorpheusUDP_pair x(#x,0,vec);
	_(Buttons)
	_(Volume)
	_(Worn) //bool
	_(DisplayActive) //bool
	_(Muted) //bool
	_(EarphonesConnected) //bool ("true" or "false")
	_(Timestamp1)
	_(RawGyroYaw1)
	_(RawGyroPitch1)
	_(RawGyroRoll1)
	_(RawMotionX1)
	_(RawMotionY1)
	_(RawMotionZ1)
	_(Timestamp2)
	_(RawGyroYaw2)
	_(RawGyroPitch2)
	_(RawGyroRoll2)
	_(RawMotionX2)
	_(RawMotionY2)
	_(RawMotionZ2)
	_(IRSensor)
	_(CalStatus)
	_(Ready)
	_(PacketSequence)
	_(VoltageReference)
	_(VoltageValue)
	//Note: configuru converts these into std:map objects
	//{"LinearAcceleration1",{"X",-0.0224609375},{"Y",-0.198242188},{"Z",0.959960938}},
	//{"AngularAcceleration1",{"X",-0.00425650924},{"Y",-0.00106412731},{"Z",-0.007448891}},
	//{"LinearAcceleration2",{"X",-0.015625},{"Y",-0.198242188},{"Z",0.9589844}},
	//{"AngularAcceleration2",{"X",-0.00106412731},{"Y",0.00212825462},{"Z",0.0}},
	//{"Pose",{"X",-0.000336476718},{"Y",0.00708646653},{"Z",-0.0353542566},{"W",0.9993495},{"IsIdentity",false}},
	//{"Orientation",{"X",3.140419},{"Y",0.0141403927},{"Z",-0.0707333162}},
	//Note: These aren't the LED lights
	//_(A)_(B)_(C)_(D)_(E)_(F)_(G)_(H)_(I)
	#undef _
	static int fill(char(&buf)[1024])
	{
		char *b = buf;
		*b++ = '{';
		MorpheusUDP_pair::vec::iterator it = vec.begin();		
		for(int n,s=sizeof(buf)-1;it<vec.end();it++,s-=n,b+=n)
		{
			n = snprintf(b,s,"\"%s\":%d,",(*it)->first,(*it)->second);			
			assert(n>=0);
		}
		b[-1] = '}'; return int(b-buf);
	}
}
namespace MorpheusUDP_PSVRToolbox_command
{
	static MorpheusUDP_pair::vec vec;
	#define _(x) static MorpheusUDP_pair x(#x,sizeof(#x)-1,vec);
	_(HeadsetOn) //HeadSetOn
	_(HeadsetOff) //HeadSetOff
	_(EnableVRTracking)
	_(EnableVRMode)
	_(EnableCinematicMode)
	_(Shutdown)
	_(CinematicSettings)
	_(LedSettings)
	//_(StoreSettings)
	//_(DiscardSettings)
	#undef _
		
	//This is my settings.json file. It contains the "StoreSettings" values. 
	/*{
	"ControlModifier":true,
	"ShiftModifier":false,
	"AltModifier":false,
	"HeadSetOn":0, //hotkey
	"HeadSetOff":0, //hotkey
	"EnableVRAndTracking":0, //hotkey
	"EnableVR":13, //hotkey
	"EnableTheater":106, //hotkey
	"Recenter":32, //hotkey
	"Shutdown":0, //hotkey
	"EnableUDPBroadcast":true,
	"UDPBroadcastAddress":"255.255.255.255",
	"UDPBroadcastPort":9090,
	"OpenTrackPort":4242,
	"StartMinimized":false,
	"ScreenSize":41, //CinematicSettings
	"ScreenDistance":50, //CinematicSettings
	"Brightness":32, //CinematicSettings
	"MicVol":0,
	"LedAIntensity":49,"LedBIntensity":49,"LedCIntensity":49,"LedDIntensity":49,"LedEIntensity":49,
	"LedFIntensity":49,"LedGIntensity":49,"LedHIntensity":49,"LedIIntensity":49
	}*/

	static void send(const char *c, MorpheusUSBContext *mc)
	{
		if(*c=='{')
		{
			if(*++c==' ') c++;
			if(c==strstr(c,"\"Command\":\"")) c+=10;
			else return;			
		}		
		size_t s = 0; if(*c=='\"')
		{
			c++; s = strchr(c,'"')-c;
		}
		else while(c[s]&&c[s]!=',') s++;

		MorpheusUDP_pair::vec::iterator it = vec.begin();		
		while(it<vec.end()) if(s==(*it)->second&&!memcmp(c,(*it)->first,s))
		break; else it++; if(it==vec.end()) return;

		const bool async = true;
		MorpheusUDP_pair *p = *it;
		if(p==&HeadsetOn)
		morpheus_set_headset_power(mc,true,async);
		if(p==&HeadsetOff)
		morpheus_set_headset_power(mc,false,async);
		if(p==&EnableVRTracking)
		morpheus_enable_tracking(mc,async);
		if(p==&EnableVRMode)
		morpheus_set_vr_mode(mc,true,async);
		if(p==&EnableCinematicMode)
		morpheus_set_vr_mode(mc,false,async);
		if(p==&Shutdown)
		morpheus_turn_off_processor_unit(mc,async);
		if(p==&CinematicSettings)
		{
			//{ "Command":"CinematicSettings", 
			//"Brightness":(value), "Size":(size), "Distance":(distance) }    

			unsigned char cc[4] = {}; //Mic Volume?

			for(c+=p->second;*c;c++) if(*c>='A'&&*c<='Z')
			{
				int i = *c;

				while(*c<'1'||*c>'9'&&*c) c++;

				unsigned char l = 0xFF&strtoul(c,(char**)&c,10);

				switch(i) //TODO? Defaults?
				{
				case 'D': cc[0] = l; break; //Distance
				case 'S': cc[1] = l; break; //Size
				case 'B': cc[2] = l; break; //Brightness

				default: cc[3] = 0xFF&l; //Mic Volume?
				}
			}
			morpheus_set_cinematic_configuration(mc,cc[0],cc[1],cc[2],cc[3],0,async);
		}
		if(p==&LedSettings)
		{
			//{ "Command":"LedSettings", 
			//"LedA":(value), "LedB":(value), "LedI":(value) }    

			if(s==11&&!memcmp(c,"LedSettings",s))
			{
				unsigned char i['I'-'A'+1] = {};
				const char *cc = c; cc+=p->second;
				for(int a='A';a<='I';a++) if(c=strchr(cc,a))
				{
					while(*c==','||*c=='"'||*c==' ') c++;

					i[a-'A'] = 0xFF&strtoul(c,(char**)&c,10);
				}
				else continue;

				morpheus_set_led_brightness(mc,i,async);
			}
		}
	}
}

static struct MorpheusUDP_PSVRToolbox
{		
	//Can the port be bidirectional? I guess would have to wade
	//through its own packets if so? Or would it?

	struct Sym 
	{
		SOCKADDR_IN in; SOCKET udp;

		Sym():udp(INVALID_SOCKET){}

		bool valid(){ return udp!=INVALID_SOCKET; }

		void open(int i, int port)
		{
			if(port<=0) return;

			udp = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP); 
			u_long mode = 1; // 1 to enable non-blocking socket
			if(!i) ioctlsocket(udp,FIONBIO,&mode);

			//localhost
			hostent *lh = gethostbyname("");
			char *ip = inet_ntoa(*(struct in_addr*)*lh->h_addr_list);

			memset(&in,0x00,sizeof(in));
			in.sin_family = AF_INET;		
			in.sin_port = htons(port);
			//I think this needs setsockopt to work?
			//in.sin_addr.s_addr = INADDR_BROADCAST; 
			in.sin_addr.s_addr = inet_addr(ip);	
			/*Picky: Only recv sockets should bind
			int err = bind(udp,(SOCKADDR*)&in,sizeof(in));
			err = err; //breakpoint*/
		}
		void close()
		{
			closesocket(udp); //udp = INVALID_SOCKET;
		}

		void send(char *d, int len)
		{
			if(-1==sendto(udp,d,len,0,(SOCKADDR*)&in,sizeof(in)))
			{
				len = WSAGetLastError();
				len = len; //breakpoint
			}	
		}
		int recv(char *d, int len)
		{
			return recvfrom(udp,d,len,0,0,0);
		}
		void recv_bind() // Only recv sockets should bind
		{
			int err = bind(udp,(SOCKADDR*)&in,sizeof(in));
			err = err; //breakpoint
		}

	}io[2]; 

	~MorpheusUDP_PSVRToolbox()
	{			
		io[0].close(); io[1].close();
	}
	MorpheusUDP_PSVRToolbox(const MorpheusHMDConfig &cfg, MorpheusUSBContext *ctx)
	{
		WSADATA w; 
		if(0!=WSAStartup(0x202,&w))
		return;

		morpheus_set_cinematic_configuration(ctx,
		cfg.screen_distance,cfg.screen_size,cfg.screen_brightness,cfg.mic_volume,0,true);

		io[0].open(0,cfg.UDP_command_port);
		io[0].recv_bind();
		io[1].open(1,cfg.UDP_status_port);
	}

	void send(char *str, int len, MorpheusUSBContext *ctx)
	{
		io[1].send(str,len+1);

		if(ctx&&io[0].valid())
		{
			char json[1024];
			len = io[0].recv(json,sizeof(json)-1);
			if(len>0)
			{
				json[len] = '\0';
				MorpheusUDP_PSVRToolbox_command::send(json,ctx);
			}
		}
	}

}*MorpheusUDP_PSVRToolbox = nullptr;

MorpheusUDP::~MorpheusUDP()
{	
	if(!MorpheusUDP_PSVRToolbox)
	{
		if(!st) return;

		#ifdef _WIN32
		//2018: Read somewhere this gives A/V applications priority
		HMODULE av = LoadLibraryA("Avrt.dll"); if(av)
		{
			//AvRevertMmThreadCharacteristics exists
			//AvSetMmMaxThreadCharacteristics also works?
			DWORD avid = 0;
			if(void*pa=GetProcAddress(av,"AvSetMmThreadCharacteristicsA"))
			{
				HANDLE avh = (HANDLE(WINAPI*)(LPCSTR,LPDWORD))("Games",&avid);
				assert(avh&&avh!=INVALID_HANDLE_VALUE);		
				//should it be freed?
				//FreeLibrary(av);
			}
			else assert(0);
		}
		#endif

		MorpheusUDP_PSVRToolbox = new struct MorpheusUDP_PSVRToolbox(*cfg,ctx);
	}
	else if(!st)
	{
		delete MorpheusUDP_PSVRToolbox; MorpheusUDP_PSVRToolbox = nullptr; 
		
		return;
	}
	
	char json[1024];
	{	
		using namespace MorpheusUDP_PSVRToolbox_status;
 
		Timestamp1    = st[0].SequenceNumber;
		RawGyroPitch1 = st[0].RawGyro.x; //y
		RawGyroYaw1   = st[0].RawGyro.y; //x
		RawGyroRoll1  =-st[0].RawGyro.z; //-
		RawMotionX1   = st[0].RawAccel.y; //y
		RawMotionY1   = st[0].RawAccel.x; //x
		RawMotionZ1   =-st[0].RawAccel.z; //-
		Timestamp2    = st[1].SequenceNumber;
		RawGyroPitch2 = st[1].RawGyro.x; //y
		RawGyroYaw2   = st[1].RawGyro.y; //x
		RawGyroRoll2  =-st[1].RawGyro.z; //-
		RawMotionX2   = st[1].RawAccel.y; //x
		RawMotionY2   = st[1].RawAccel.x; //y
		RawMotionZ2   =-st[1].RawAccel.z; //-

		Buttons = (int)st2->buttons;
		Volume  = st2->volume;
		Worn    = st2->headsetFlags.hmdOnHead;
		DisplayActive = st2->headsetFlags.displayIsOn;
		Muted   = st2->headsetFlags.microphoneMuted;
		EarphonesConnected = st2->headsetFlags.headphonesPresent;
		IRSensor = (int)st2->face_distance[1]<<8+st2->face_distance[0];
		CalStatus = st2->calibration_status;
		Ready = st2->sensors_ready;
		PacketSequence = st2->sequence;
		VoltageReference = st2->unk5; //iffy
		VoltageValue = st2->unk6; //iffy
		
		/*Note: These aren't the LED lights
		A = st2->unk4[0];
		B = st2->unk4[1];
		C = st2->unk4[2];
		D = st2->unk7[1];
		E = st2->unk4[2];
		F = st2->unk4[3];
		G = st2->unk4[4];
		H = st2->unk4[5];*/
	}
	int len = MorpheusUDP_PSVRToolbox_status::fill(json);

	//These aren't filled out.
	//{"LinearAcceleration1",{"X",-0.0224609375},{"Y",-0.198242188},{"Z",0.959960938}},
	//{"AngularAcceleration1",{"X",-0.00425650924},{"Y",-0.00106412731},{"Z",-0.007448891}},
	//{"LinearAcceleration2",{"X",-0.015625},{"Y",-0.198242188},{"Z",0.9589844}},
	//{"AngularAcceleration2",{"X",-0.00106412731},{"Y",0.00212825462},{"Z",0.0}},
	//{"Pose",{"X",-0.000336476718},{"Y",0.00708646653},{"Z",-0.0353542566},{"W",0.9993495},{"IsIdentity",false}},
	//{"Orientation",{"X",3.140419},{"Y",0.0141403927},{"Z",-0.0707333162}},

	MorpheusUDP_PSVRToolbox->send(json,len,ctx);
}
