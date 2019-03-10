// -- includes -----
#include "HMDHidDeviceEnumerator.h"
#include "Utility.h"
#include "USBDeviceFilter.h" // for MAX_USB_DEVICE_PORT_PATH, t_usb_device_handle
#include "assert.h"
#include "hidapi.h"
#include "string.h"
#include <sstream>
#include <iomanip>

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_HMD_TYPE_INDEX                  GET_DEVICE_TYPE_INDEX(CommonSensorState::SUPPORTED_HMD_TYPE_COUNT)

struct HMDHidFilter
{
    USBDeviceFilter usb_filter;
    int interface_filter;
};

// -- globals -----
HMDHidFilter g_supported_hmd_infos[MAX_HMD_TYPE_INDEX] = {
    {{ 0x054c, 0x09af }, 4} // Sony Morpheus (sensor HID iterface)
};

// -- HMDHidDeviceEnumerator -----
HMDHidDeviceEnumerator::HMDHidDeviceEnumerator()
    : DeviceEnumerator(CommonSensorState::Morpheus)
{
	m_deviceType= CommonSensorState::Morpheus;
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

	find_first_valid_hid_interface();

	if (!is_valid())
	{
		next();
	}
}

const char *HMDHidDeviceEnumerator::get_path() const
{
	return current_device_identifier.c_str();
}

int HMDHidDeviceEnumerator::get_vendor_id() const
{
	return is_valid() ? g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)].usb_filter.vendor_id : -1;
}

int HMDHidDeviceEnumerator::get_product_id() const
{
	return is_valid() ? g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)].usb_filter.product_id : -1;
}

bool HMDHidDeviceEnumerator::is_valid() const
{
	return current_device_interface_info.interface_number != -1;
}

bool HMDHidDeviceEnumerator::next()
{
	bool foundValid = false;

	while (!foundValid && m_deviceType < CommonSensorState::SUPPORTED_HMD_TYPE_COUNT)
	{
		m_deviceType = static_cast<CommonSensorState::eDeviceType>(m_deviceType + 1);

		if (GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX)
		{
			find_first_valid_hid_interface();

			foundValid = is_valid();
		}
		else
		{
			current_device_identifier = "";
			current_device_interface_info.device_path= "";
            current_device_interface_info.interface_number= -1;
		}
	}

	return foundValid;
}

std::string HMDHidDeviceEnumerator::get_hid_interface_path() const
{
	return current_device_interface_info.device_path;
}

void HMDHidDeviceEnumerator::find_first_valid_hid_interface()
{
	HMDHidFilter &dev_info = g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
	hid_device_info * devs = hid_enumerate(dev_info.usb_filter.vendor_id, dev_info.usb_filter.product_id);

	current_device_identifier = "";
	current_device_interface_info.device_path= "";
    current_device_interface_info.interface_number= -1;

	if (devs != nullptr)
	{
		std::stringstream device_id_builder;
		device_id_builder << 
			"USB\\VID_" << std::hex << std::setfill('0') << std::setw(4) << dev_info.usb_filter.vendor_id <<
			"&PID_" << std::hex << std::setfill('0') << std::setw(4) << dev_info.usb_filter.product_id;

		current_device_identifier = device_id_builder.str();

		for (hid_device_info *cur_dev = devs; cur_dev != nullptr; cur_dev = cur_dev->next)
		{
            if (cur_dev->interface_number == dev_info.interface_filter)
            {
                current_device_interface_info.device_path= cur_dev->path;
                current_device_interface_info.interface_number = cur_dev->interface_number;
                break;
            }
		}

		hid_free_enumeration(devs);
	}
}