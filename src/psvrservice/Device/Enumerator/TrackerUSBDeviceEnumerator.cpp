// -- includes -----
#include "TrackerUSBDeviceEnumerator.h"
#include "TrackerCapabilitiesConfig.h"
#include "Utility.h"
#include "USBDeviceManager.h"
#include "Logger.h"
#include "assert.h"
#include "string.h"

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_CAMERA_TYPE_INDEX               GET_DEVICE_TYPE_INDEX(CommonSensorState::SUPPORTED_CAMERA_TYPE_COUNT)

// -- statics -----
TrackerCapabilitiesSet *TrackerUSBDeviceEnumerator::s_supportedTrackers= nullptr;

// -- globals -----
struct TrackerFilter
{
	USBDeviceFilter filter;
	bool bUSBApiSupported;
    bool bTestCanOpen;
};

// NOTE: This list must match the tracker order in CommonSensorState::eDeviceType
TrackerFilter k_supported_tracker_infos[MAX_CAMERA_TYPE_INDEX] = {
    {{ 0x1415, 0x2000 }, true, true}, // PS3Eye
    {{ 0x0000, 0x0000 }, false, false}, // Virtual Stereo Camera
	{{ 0x0000, 0x0000 }, false, false} // WMF Stereo Camera
};

// -- private prototypes -----
static bool is_tracker_supported(
    USBDeviceEnumerator* enumerator, 
    CommonSensorState::eDeviceType device_type_filter, 
    CommonSensorState::eDeviceType &out_device_type,
    bool &out_needs_to_test_device_open);

// -- methods -----
TrackerUSBDeviceEnumerator::TrackerUSBDeviceEnumerator()
	: DeviceEnumerator()
	, m_usb_enumerator(nullptr)
    , m_cameraIndex(-1)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	m_deviceType= CommonSensorState::PS3EYE;
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	// If the first USB device handle isn't a tracker, move on to the next device
	if (testUSBEnumerator())
	{
		m_cameraIndex= 0;
	}
	else
	{
		next();
	}
}

TrackerUSBDeviceEnumerator::TrackerUSBDeviceEnumerator(CommonSensorState::eDeviceType deviceTypeFilter)
	: DeviceEnumerator(deviceTypeFilter)
	, m_usb_enumerator(nullptr)
    , m_cameraIndex(-1)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	m_deviceType= CommonSensorState::PS3EYE;
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	// If the first USB device handle isn't a tracker, move on to the next device
	if (testUSBEnumerator())
	{
		m_cameraIndex= 0;
	}
	else
	{
		next();
	}
}

TrackerUSBDeviceEnumerator::TrackerUSBDeviceEnumerator(const std::string &usb_path)
	: DeviceEnumerator()
	, m_usb_enumerator(nullptr)
    , m_cameraIndex(-1)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	m_deviceType= CommonSensorState::PS3EYE;
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	// If the first USB device handle isn't a tracker, move on to the next device
    const char *szTestUSBPath= usb_path.c_str();
	while (is_valid())
	{
        // Find the device with the matching usb path
        if (testUSBEnumerator() && 
            strncmp(m_currentUSBPath, szTestUSBPath, sizeof(m_currentUSBPath)) == 0)
        {
            m_cameraIndex= std::max(m_cameraIndex, 0);
            break;
        }
        else
        {
            next();
        }
	}
}

TrackerUSBDeviceEnumerator::~TrackerUSBDeviceEnumerator()
{
	if (m_usb_enumerator != nullptr)
	{
		usb_device_enumerator_free(m_usb_enumerator);
	}
}

int TrackerUSBDeviceEnumerator::get_vendor_id() const
{
	USBDeviceFilter devInfo;
	int vendor_id = -1;

	if (is_valid() && usb_device_enumerator_get_filter(m_usb_enumerator, devInfo))
	{
		vendor_id = devInfo.vendor_id;
	}

	return vendor_id;
}

int TrackerUSBDeviceEnumerator::get_product_id() const
{
	USBDeviceFilter devInfo;
	int product_id = -1;

	if (is_valid() && usb_device_enumerator_get_filter(m_usb_enumerator, devInfo))
	{
		product_id = devInfo.product_id;
	}

	return product_id;
}

const char *TrackerUSBDeviceEnumerator::get_path() const
{
    const char *result = nullptr;

    if (is_valid())
    {
        // Return a pointer to our member variable that has the path cached
        result= m_currentUSBPath;
    }

    return result;
}

const TrackerCapabilitiesConfig *TrackerUSBDeviceEnumerator::getTrackerCapabilities() const
{
	return is_valid() ? s_supportedTrackers->getTrackerCapabilities(get_vendor_id(), get_product_id()) : nullptr;
}

bool TrackerUSBDeviceEnumerator::is_valid() const
{
	return m_usb_enumerator != nullptr && usb_device_enumerator_is_valid(m_usb_enumerator);
}

bool TrackerUSBDeviceEnumerator::next()
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();
	bool foundValid = false;

	while (is_valid() && !foundValid)
	{
		usb_device_enumerator_next(m_usb_enumerator);

		if (testUSBEnumerator())
		{
			foundValid= true;
		}
	}

	if (foundValid)
	{
		++m_cameraIndex;
	}

	return foundValid;
}

bool TrackerUSBDeviceEnumerator::testUSBEnumerator()
{
	bool found_valid= false;
    bool needs_to_test_device_open= false;

	if (is_valid() && 
        is_tracker_supported(m_usb_enumerator, m_deviceTypeFilter, m_deviceType, needs_to_test_device_open))
	{
		char USBPath[256];

		// Cache the path to the device
		usb_device_enumerator_get_path(m_usb_enumerator, USBPath, sizeof(USBPath));

		// Test open the device
		char errorReason[256];
		if (!needs_to_test_device_open ||
            usb_device_can_be_opened(m_usb_enumerator, errorReason, sizeof(errorReason)))
		{
			// Remember the last successfully opened tracker path
			strncpy(m_currentUSBPath, USBPath, sizeof(m_currentUSBPath));

			found_valid = true;
		}
		else
		{
			PSVR_LOG_INFO("TrackerUSBDeviceEnumerator") << "Skipping device (" <<  USBPath << ") - " << errorReason;
		}
	}

	return found_valid;
}

//-- private methods -----
static bool is_tracker_supported(
	USBDeviceEnumerator *enumerator, 
	CommonSensorState::eDeviceType device_type_filter,
	CommonSensorState::eDeviceType &out_device_type,
    bool &out_needs_to_test_device_open)
{
	USBDeviceFilter devInfo;
	bool bIsValidDevice = false;

	if (usb_device_enumerator_get_filter(enumerator, devInfo))
	{
		// See if the next filtered device is a camera that we care about
		for (int tracker_type_index = 0; tracker_type_index < MAX_CAMERA_TYPE_INDEX; ++tracker_type_index)
		{
			const TrackerFilter &supported_type = k_supported_tracker_infos[tracker_type_index];

			if (supported_type.bUSBApiSupported &&
				devInfo.product_id == supported_type.filter.product_id &&
				devInfo.vendor_id == supported_type.filter.vendor_id)
			{
				CommonSensorState::eDeviceType device_type = 
					static_cast<CommonSensorState::eDeviceType>(CommonSensorState::TrackingCamera + tracker_type_index);

				if (device_type_filter == CommonSensorState::INVALID_DEVICE_TYPE || // i.e. no filter
					device_type_filter == device_type)
				{
                    out_needs_to_test_device_open= supported_type.bTestCanOpen;
					out_device_type = device_type;
					bIsValidDevice = true;
					break;
				}
			}
		}
	}

	return bIsValidDevice;
}