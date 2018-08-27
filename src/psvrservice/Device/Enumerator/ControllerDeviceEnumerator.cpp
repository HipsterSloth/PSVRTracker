// -- includes -----
#include "ControllerDeviceEnumerator.h"
#include "ControllerHidDeviceEnumerator.h"
#include "assert.h"
#include "string.h"

// -- globals -----

// -- ControllerDeviceEnumerator -----
ControllerDeviceEnumerator::ControllerDeviceEnumerator(
	eAPIType _apiType)
	: DeviceEnumerator()
	, api_type(_apiType)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_HID:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerHidDeviceEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerHidDeviceEnumerator;
		enumerator_count = 1;
		break;
	}

	if (is_valid())
	{
		m_deviceType= enumerators[enumerator_index]->get_device_type();
	}
	else
    {
        next();
    }
}

ControllerDeviceEnumerator::~ControllerDeviceEnumerator()
{
	for (int index = 0; index < enumerator_count; ++index)
	{
		delete enumerators[index];
	}
	delete[] enumerators;
}

const char *ControllerDeviceEnumerator::get_path() const
{
    return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_path() : nullptr;
}

int ControllerDeviceEnumerator::get_vendor_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_vendor_id() : -1;
}

int ControllerDeviceEnumerator::get_product_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_product_id() : -1;
}

bool ControllerDeviceEnumerator::get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const
{
    bool success = false;

    if ((api_type == eAPIType::CommunicationType_HID) ||
		(api_type == eAPIType::CommunicationType_ALL && enumerator_index == 0))
    {
		ControllerHidDeviceEnumerator *hid_enumerator = static_cast<ControllerHidDeviceEnumerator *>(enumerators[enumerator_index]);

        success = hid_enumerator->get_serial_number(out_mb_serial, mb_buffer_size);
    }

    return success;
}

ControllerDeviceEnumerator::eAPIType ControllerDeviceEnumerator::get_api_type() const
{
	ControllerDeviceEnumerator::eAPIType result= ControllerDeviceEnumerator::CommunicationType_INVALID;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_HID : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			switch (enumerator_index)
			{
			case 0:
				result = ControllerDeviceEnumerator::CommunicationType_HID;
				break;
			default:
				result = ControllerDeviceEnumerator::CommunicationType_INVALID;
				break;
			}
		}
		else
		{
			result = ControllerDeviceEnumerator::CommunicationType_INVALID;
		}
		break;
	}

	return result;
}

const ControllerHidDeviceEnumerator *ControllerDeviceEnumerator::get_hid_controller_enumerator() const
{
	ControllerHidDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<ControllerHidDeviceEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 0) ? static_cast<ControllerHidDeviceEnumerator *>(enumerators[0]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

bool ControllerDeviceEnumerator::is_valid() const
{
    bool bIsValid = false;

	if (enumerator_index < enumerator_count)
	{
		bIsValid = enumerators[enumerator_index]->is_valid();
	}

    return bIsValid;
}

bool ControllerDeviceEnumerator::next()
{
    bool foundValid = false;

    while (!foundValid && enumerator_index < enumerator_count)
    {
		if (enumerators[enumerator_index]->is_valid())
		{
			enumerators[enumerator_index]->next();
			foundValid = enumerators[enumerator_index]->is_valid();
		}
		else
		{
			++enumerator_index;

			if (enumerator_index < enumerator_count)
			{
				foundValid = enumerators[enumerator_index]->is_valid();
			}
		}
    }

	if (foundValid)
	{
		m_deviceType = enumerators[enumerator_index]->get_device_type();
	}
	else
	{
		m_deviceType = CommonSensorState::INVALID_DEVICE_TYPE; // invalid
	}

    return foundValid;
}