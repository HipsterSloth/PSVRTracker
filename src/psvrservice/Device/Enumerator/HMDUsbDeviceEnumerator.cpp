// -- includes -----
#include "HMDUsbDeviceEnumerator.h"
#include "Utility.h"
#include "USBDeviceManager.h"
#include "Logger.h"
#include "assert.h"
#include "string.h"

// -- macros ----
#define MAX_HMD_TYPE_INDEX               GET_DEVICE_TYPE_INDEX(CommonSensorState::SUPPORTED_HMD_TYPE_COUNT)

// -- globals -----
// NOTE: This list must match the tracker order in CommonSensorState::eDeviceType
USBDeviceFilter g_supported_hmd_infos[MAX_HMD_TYPE_INDEX] = {
    { 0x054c, 0x09af }, // Sony Morpheus
};

// -- private prototypes -----
static bool is_hmd_supported(
    USBDeviceEnumerator* enumerator, 
    CommonSensorState::eDeviceType device_type_filter, 
    CommonSensorState::eDeviceType &out_device_type);

// -- methods -----
HMDUsbDeviceEnumerator::HMDUsbDeviceEnumerator()
	: DeviceEnumerator()
	, m_usb_enumerator(nullptr)
    , m_hmdIndex(-1)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	m_deviceType= CommonSensorState::Morpheus;
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	// If the first USB device handle isn't an HMD, move on to the next device
	if (testUSBEnumerator())
	{
		m_hmdIndex= 0;
	}
	else
	{
		next();
	}
}

HMDUsbDeviceEnumerator::HMDUsbDeviceEnumerator(CommonSensorState::eDeviceType deviceTypeFilter)
	: DeviceEnumerator(deviceTypeFilter)
	, m_usb_enumerator(nullptr)
    , m_hmdIndex(-1)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	m_deviceType= CommonSensorState::PS3EYE;
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	// If the first USB device handle isn't a tracker, move on to the next device
	if (testUSBEnumerator())
	{
		m_hmdIndex= 0;
	}
	else
	{
		next();
	}
}

HMDUsbDeviceEnumerator::HMDUsbDeviceEnumerator(const std::string &usb_path)
	: DeviceEnumerator()
	, m_usb_enumerator(nullptr)
    , m_hmdIndex(-1)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	m_deviceType= CommonSensorState::PS3EYE;
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	// If the first USB device handle isn't a tracker, move on to the next device
    const char *szTestUSBPath= usb_path.c_str();
	while (is_valid())
	{
        // Find the device with the matching usb path
        if (testUSBEnumerator() && 
            strncmp(m_currentUSBPath, szTestUSBPath, sizeof(m_currentUSBPath)) == 0)
        {
            m_hmdIndex= std::max(m_hmdIndex, 0);
            break;
        }
        else
        {
            next();
        }
	}
}

HMDUsbDeviceEnumerator::~HMDUsbDeviceEnumerator()
{
	if (m_usb_enumerator != nullptr)
	{
		usb_device_enumerator_free(m_usb_enumerator);
	}
}

int HMDUsbDeviceEnumerator::get_vendor_id() const
{
	USBDeviceFilter devInfo;
	int vendor_id = -1;

	if (is_valid() && usb_device_enumerator_get_filter(m_usb_enumerator, devInfo))
	{
		vendor_id = devInfo.vendor_id;
	}

	return vendor_id;
}

int HMDUsbDeviceEnumerator::get_product_id() const
{
	USBDeviceFilter devInfo;
	int product_id = -1;

	if (is_valid() && usb_device_enumerator_get_filter(m_usb_enumerator, devInfo))
	{
		product_id = devInfo.product_id;
	}

	return product_id;
}

const char *HMDUsbDeviceEnumerator::get_path() const
{
    const char *result = nullptr;

    if (is_valid())
    {
        // Return a pointer to our member variable that has the path cached
        result= m_currentUSBPath;
    }

    return result;
}

const char *HMDUsbDeviceEnumerator::get_unique_identifier() const
{
    const char *result = nullptr;

    if (is_valid())
    {
        // Return a pointer to our member variable that has the identifier cached
        result= m_currentUSBIdentifier;
    }

    return result;
}

bool HMDUsbDeviceEnumerator::is_valid() const
{
	return m_usb_enumerator != nullptr && usb_device_enumerator_is_valid(m_usb_enumerator);
}

bool HMDUsbDeviceEnumerator::next()
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
		++m_hmdIndex;
	}

	return foundValid;
}

bool HMDUsbDeviceEnumerator::testUSBEnumerator()
{
	bool found_valid= false;

	if (is_valid() && 
        is_hmd_supported(m_usb_enumerator, m_deviceTypeFilter, m_deviceType))
	{
		char USBPath[256];
		char USBIdentifier[256];

		usb_device_enumerator_get_path(m_usb_enumerator, USBPath, sizeof(USBPath));
		usb_device_enumerator_get_unique_identifier(m_usb_enumerator, USBIdentifier, sizeof(USBIdentifier));

		// Test open the device
		char errorReason[256];
		if (usb_device_can_be_opened(m_usb_enumerator, errorReason, sizeof(errorReason)))
		{
			// Remember the last successfully opened tracker path
			strncpy(m_currentUSBPath, USBPath, sizeof(m_currentUSBPath));
			strncpy(m_currentUSBIdentifier, USBIdentifier, sizeof(m_currentUSBIdentifier));

			m_currentDriverType= usb_device_enumerator_get_driver_type(m_usb_enumerator);

			found_valid = true;
		}
		else
		{
			PSVR_LOG_INFO("HMDUsbDeviceEnumerator") << "Skipping device (" <<  USBPath << ") - " << errorReason;
		}
	}

	return found_valid;
}

//-- private methods -----
static bool is_hmd_supported(
	USBDeviceEnumerator *enumerator, 
	CommonSensorState::eDeviceType device_type_filter,
	CommonSensorState::eDeviceType &out_device_type)
{
	USBDeviceFilter devInfo;
	bool bIsValidDevice = false;

	if (usb_device_enumerator_get_filter(enumerator, devInfo))
	{
		// See if the next filtered device is a camera that we care about
		for (int tracker_type_index = 0; tracker_type_index < MAX_HMD_TYPE_INDEX; ++tracker_type_index)
		{
			const USBDeviceFilter &supported_type = g_supported_hmd_infos[tracker_type_index];

			if (devInfo.product_id == supported_type.product_id &&
				devInfo.vendor_id == supported_type.vendor_id)
			{
				CommonSensorState::eDeviceType device_type = 
					static_cast<CommonSensorState::eDeviceType>(CommonSensorState::HeadMountedDisplay + tracker_type_index);

				if (device_type_filter == CommonSensorState::INVALID_DEVICE_TYPE || // i.e. no filter
					device_type_filter == device_type)
				{
					out_device_type = device_type;
					bIsValidDevice = true;
					break;
				}
			}
		}
	}

	return bIsValidDevice;
}