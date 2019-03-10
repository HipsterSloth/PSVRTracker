// -- includes -----
#include "HMDDeviceEnumerator.h"
#include "HMDHidDeviceEnumerator.h"
#include "HMDUsbDeviceEnumerator.h"
#include "assert.h"
#include "string.h"

// -- globals -----

// -- HMDDeviceEnumerator -----
HMDDeviceEnumerator::HMDDeviceEnumerator(
	eAPIType _apiType)
	: DeviceEnumerator()
	, m_apiType(_apiType)
	, m_enumerators(nullptr)
	, m_enumeratorCount(0)
	, m_enumeratorIndex(0)
    , m_foundAnyValidHMD(false)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_HID:
		m_enumerators = new DeviceEnumerator *[1];
		m_enumerators[0] = new HMDHidDeviceEnumerator;
		m_enumeratorCount = 1;
		break;
	case eAPIType::CommunicationType_USB:
		m_enumerators = new DeviceEnumerator *[1];
		m_enumerators[0] = new HMDUsbDeviceEnumerator;
		m_enumeratorCount = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		m_enumerators = new DeviceEnumerator *[2];
		m_enumerators[0] = new HMDHidDeviceEnumerator;
        m_enumerators[1] = new HMDUsbDeviceEnumerator;
		m_enumeratorCount = 2;
		break;
	}

	if (is_valid())
	{
		m_deviceType= m_enumerators[m_enumeratorIndex]->get_device_type();

        // Don't bother looking for any more HMDs once the first valid one is found
        m_foundAnyValidHMD = true;
	}
	else
    {
        next();
    }
}

HMDDeviceEnumerator::~HMDDeviceEnumerator()
{
	for (int index = 0; index < m_enumeratorCount; ++index)
	{
		delete m_enumerators[index];
	}
	delete[] m_enumerators;
}

const char *HMDDeviceEnumerator::get_path() const
{
    return (m_enumeratorIndex < m_enumeratorCount) ? m_enumerators[m_enumeratorIndex]->get_path() : nullptr;
}

int HMDDeviceEnumerator::get_vendor_id() const
{
	return (m_enumeratorIndex < m_enumeratorCount) ? m_enumerators[m_enumeratorIndex]->get_vendor_id() : -1;
}

int HMDDeviceEnumerator::get_product_id() const
{
	return (m_enumeratorIndex < m_enumeratorCount) ? m_enumerators[m_enumeratorIndex]->get_product_id() : -1;
}

HMDDeviceEnumerator::eAPIType HMDDeviceEnumerator::get_api_type() const
{
	HMDDeviceEnumerator::eAPIType result= HMDDeviceEnumerator::CommunicationType_INVALID;

	switch (m_apiType)
	{
	case eAPIType::CommunicationType_HID:
		result = (m_enumeratorIndex < m_enumeratorCount) ? HMDDeviceEnumerator::CommunicationType_HID : HMDDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_USB:
		result = (m_enumeratorIndex < m_enumeratorCount) ? HMDDeviceEnumerator::CommunicationType_USB : HMDDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_ALL:
		if (m_enumeratorIndex < m_enumeratorCount)
		{
			switch (m_enumeratorIndex)
			{
			case 0:
				result = HMDDeviceEnumerator::CommunicationType_HID;
				break;
			case 1:
				result = HMDDeviceEnumerator::CommunicationType_USB;
				break;
			default:
				result = HMDDeviceEnumerator::CommunicationType_INVALID;
				break;
			}
		}
		else
		{
			result = HMDDeviceEnumerator::CommunicationType_INVALID;
		}
		break;
	}

	return result;
}

const HMDHidDeviceEnumerator *HMDDeviceEnumerator::get_hmd_hid_enumerator() const
{
	HMDHidDeviceEnumerator *enumerator = nullptr;

	switch (m_apiType)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = (m_enumeratorIndex < m_enumeratorCount) ? static_cast<HMDHidDeviceEnumerator *>(m_enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (m_enumeratorIndex < m_enumeratorCount)
		{
			enumerator = (m_enumeratorIndex == 0) ? static_cast<HMDHidDeviceEnumerator *>(m_enumerators[0]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const HMDUsbDeviceEnumerator *HMDDeviceEnumerator::get_hmd_usb_enumerator() const
{
	HMDUsbDeviceEnumerator *enumerator = nullptr;

	switch (m_apiType)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = (m_enumeratorIndex < m_enumeratorCount) ? static_cast<HMDUsbDeviceEnumerator *>(m_enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (m_enumeratorIndex < m_enumeratorCount)
		{
			enumerator = (m_enumeratorIndex == 1) ? static_cast<HMDUsbDeviceEnumerator *>(m_enumerators[1]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

bool HMDDeviceEnumerator::is_valid() const
{
    bool bIsValid = false;

	if (m_enumeratorIndex < m_enumeratorCount)
	{
		bIsValid = m_enumerators[m_enumeratorIndex]->is_valid();
	}

    return bIsValid;
}

bool HMDDeviceEnumerator::next()
{
    bool foundValid = false;

    while (!foundValid && m_enumeratorIndex < m_enumeratorCount)
    {
		if (!m_foundAnyValidHMD && m_enumerators[m_enumeratorIndex]->is_valid())
		{
			m_enumerators[m_enumeratorIndex]->next();
			foundValid = m_enumerators[m_enumeratorIndex]->is_valid();
		}
		else
		{
			++m_enumeratorIndex;

			if (!m_foundAnyValidHMD && m_enumeratorIndex < m_enumeratorCount)
			{
				foundValid = m_enumerators[m_enumeratorIndex]->is_valid();
			}
		}
    }

	if (foundValid)
	{
		m_deviceType = m_enumerators[m_enumeratorIndex]->get_device_type();

        // Don't bother looking for any more HMDs once the first valid one is found
        m_foundAnyValidHMD= true;
	}
	else
	{
		m_deviceType = CommonSensorState::INVALID_DEVICE_TYPE; // invalid
	}

    return foundValid;
}