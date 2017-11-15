// -- includes -----
#include "VirtualStereoCameraEnumerator.h"
#include "Utility.h"
#include "assert.h"
#include "string.h"

//-- Statics
int VirtualStereoCameraEnumerator::virtual_stereo_camera_count= 0;

// -- VirtualControllerDeviceEnumerator -----
VirtualStereoCameraEnumerator::VirtualStereoCameraEnumerator()
    : DeviceEnumerator(CommonDeviceState::VirtualStereoCamera)
{
	m_deviceType= CommonDeviceState::VirtualStereoCamera;
    m_device_index= 0;

    m_current_device_identifier= "VirtualStereoCamera_0";
    m_device_count= virtual_stereo_camera_count;
}

const char *VirtualStereoCameraEnumerator::get_path() const
{
	return m_current_device_identifier.c_str();
}

int VirtualStereoCameraEnumerator::get_vendor_id() const
{
	return is_valid() ? 0x0000 : -1;
}

int VirtualStereoCameraEnumerator::get_product_id() const
{
	return is_valid() ? 0x0000 : -1;
}

bool VirtualStereoCameraEnumerator::is_valid() const
{
	return m_device_index < m_device_count;
}

bool VirtualStereoCameraEnumerator::next()
{
	bool foundValid = false;

	++m_device_index;
    if (m_device_index < m_device_count)
    {
        char device_path[32];
        Utility::format_string(device_path, sizeof(device_path), "VirtualStereoCamera_%d", m_device_index);

        m_current_device_identifier= device_path;
    }

	return foundValid;
}