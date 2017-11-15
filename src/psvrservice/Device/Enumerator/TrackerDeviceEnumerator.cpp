// -- includes -----
#include "TrackerDeviceEnumerator.h"
#include "TrackerUSBDeviceEnumerator.h"
#include "VirtualStereoCameraEnumerator.h"
#include "assert.h"
#include "string.h"

// -- globals -----

// -- TrackerDeviceEnumerator -----
TrackerDeviceEnumerator::TrackerDeviceEnumerator(
	eAPIType _apiType)
	: DeviceEnumerator()
	, api_type(_apiType)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
    , camera_index(-1)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_USB:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = nullptr;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_VIRTUAL_STEREO:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = nullptr;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[2];
        enumerators[0] = nullptr;
		enumerators[1] = nullptr;
		enumerator_count = 2;
		break;
	}

    allocate_child_enumerator(enumerator_index);

	if (is_valid())
	{
        camera_index= 0;
		m_deviceType= enumerators[enumerator_index]->get_device_type();
	}
	else
    {
        next();
    }
}

TrackerDeviceEnumerator::TrackerDeviceEnumerator(
	eAPIType _apiType,
	CommonDeviceState::eDeviceType deviceTypeFilter)
    : DeviceEnumerator(deviceTypeFilter)
	, api_type(_apiType)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
    , camera_index(-1)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_USB:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = nullptr;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_VIRTUAL_STEREO:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = nullptr;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[2];
		enumerators[0] = nullptr;
        enumerators[1] = nullptr;
		enumerator_count = 2;
		break;
	}

    allocate_child_enumerator(enumerator_index);

	if (is_valid())
	{
        camera_index= 0;
		m_deviceType = enumerators[enumerator_index]->get_device_type();
	}
	else
	{
		next();
	}
}

TrackerDeviceEnumerator::TrackerDeviceEnumerator(const std::string &usb_path)
    : DeviceEnumerator()
	, api_type(eAPIType::CommunicationType_USB)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
    , camera_index(0)
{
	enumerators = new DeviceEnumerator *[1];
	enumerators[0] = new TrackerUSBDeviceEnumerator(usb_path);
	enumerator_count = 1;

	if (is_valid())
	{
		m_deviceType = enumerators[0]->get_device_type();
        camera_index = static_cast<TrackerUSBDeviceEnumerator *>(enumerators[0])->get_camera_index();
	}
	else
	{
		m_deviceType= CommonDeviceState::INVALID_DEVICE_TYPE;
	}
}

TrackerDeviceEnumerator::~TrackerDeviceEnumerator()
{
	for (int index = 0; index < enumerator_count; ++index)
	{
		delete enumerators[index];
	}
	delete[] enumerators;
}

const char *TrackerDeviceEnumerator::get_path() const
{
    return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_path() : nullptr;
}

int TrackerDeviceEnumerator::get_vendor_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_vendor_id() : -1;
}

int TrackerDeviceEnumerator::get_product_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_product_id() : -1;
}

TrackerDeviceEnumerator::eAPIType TrackerDeviceEnumerator::get_api_type() const
{
	TrackerDeviceEnumerator::eAPIType result= TrackerDeviceEnumerator::CommunicationType_INVALID;

	switch (api_type)
	{
	case eAPIType::CommunicationType_USB:
		result = (enumerator_index < enumerator_count) ? TrackerDeviceEnumerator::CommunicationType_USB : TrackerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_VIRTUAL_STEREO:
		result = (enumerator_index < enumerator_count) ? TrackerDeviceEnumerator::CommunicationType_VIRTUAL_STEREO : TrackerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			switch (enumerator_index)
			{
            case 0:
				result = TrackerDeviceEnumerator::CommunicationType_VIRTUAL_STEREO;
				break;
            case 1:
				result = TrackerDeviceEnumerator::CommunicationType_USB;
				break;
			default:
				result = TrackerDeviceEnumerator::CommunicationType_INVALID;
				break;
			}
		}
		else
		{
			result = TrackerDeviceEnumerator::CommunicationType_INVALID;
		}
		break;
	}

	return result;
}

const VirtualStereoCameraEnumerator *TrackerDeviceEnumerator::get_virtual_stereo_camera_enumerator() const
{
	VirtualStereoCameraEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_USB:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL_STEREO:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<VirtualStereoCameraEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 0) ? static_cast<VirtualStereoCameraEnumerator *>(enumerators[0]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const TrackerUSBDeviceEnumerator *TrackerDeviceEnumerator::get_usb_tracker_enumerator() const
{
	TrackerUSBDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_USB:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<TrackerUSBDeviceEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL_STEREO:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 1) ? static_cast<TrackerUSBDeviceEnumerator *>(enumerators[1]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

bool TrackerDeviceEnumerator::is_valid() const
{
    bool bIsValid = false;

	if (enumerator_index < enumerator_count)
	{
		bIsValid = enumerators[enumerator_index]->is_valid();
	}

    return bIsValid;
}

bool TrackerDeviceEnumerator::next()
{
    bool foundValid = false;

    while (!foundValid && enumerator_index < enumerator_count)
    {
		if (enumerators[enumerator_index] != nullptr && enumerators[enumerator_index]->is_valid())
		{
            ++camera_index;

			enumerators[enumerator_index]->next();
			foundValid = enumerators[enumerator_index]->is_valid();
		}
		else
		{
			++enumerator_index;

			if (enumerator_index < enumerator_count &&
                camera_index < 0) //###HipsterSloth $HACK - Don't use usb iterator if virtual stereo cameras are in use
                                  // This can be removed once we have a good way to filter out usb cameras already in use
			{
                camera_index= 0;
                allocate_child_enumerator(enumerator_index);
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
		m_deviceType = CommonDeviceState::SUPPORTED_CAMERA_TYPE_COUNT; // invalid
	}

    return foundValid;
}

void TrackerDeviceEnumerator::allocate_child_enumerator(int enumerator_index)
{
    switch (api_type)
	{
	case eAPIType::CommunicationType_USB:
        assert(enumerator_index == 0);
        if (enumerators[0] == nullptr)
        {
		    enumerators[0] = new TrackerUSBDeviceEnumerator;
        }
		break;
	case eAPIType::CommunicationType_VIRTUAL_STEREO:
        assert(enumerator_index == 0);
        if (enumerators[0] == nullptr)
        {
		    enumerators[0] = new VirtualStereoCameraEnumerator;
        }
		break;
	case eAPIType::CommunicationType_ALL:
        if (enumerator_index == 0)
        {
            if (enumerators[0] == nullptr)
            {
                enumerators[0] = new VirtualStereoCameraEnumerator;
            }
        }
        else if (enumerator_index == 1)
        {
            if (enumerators[1] == nullptr)
            {
    		    enumerators[1] = new TrackerUSBDeviceEnumerator;
            }
        }
		break;
	}
}