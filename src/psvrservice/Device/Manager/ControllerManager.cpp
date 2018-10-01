//-- includes -----
#include "BluetoothQueries.h"
#include "ControllerManager.h"
#include "ControllerDeviceEnumerator.h"
#include "Logger.h"
#include "PSVRClient_CAPI.h"
#include "ServerControllerView.h"
#include "ServerDeviceView.h"

#include "hidapi.h"

//-- methods -----
//-- Tracker Manager Config -----
const int ControllerManagerConfig::CONFIG_VERSION = 1;

ControllerManagerConfig::ControllerManagerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
{

};

const configuru::Config
ControllerManagerConfig::writeToJSON()
{
    configuru::Config pt{
        {"version", ControllerManagerConfig::CONFIG_VERSION}
    };

    return pt;
}

void
ControllerManagerConfig::readFromJSON(const configuru::Config &pt)
{
    version = pt.get_or<int>("version", 0);

    if (version == ControllerManagerConfig::CONFIG_VERSION)
    {
    }
    else
    {
        PSVR_LOG_WARNING("ControllerManagerConfig") <<
            "Config version " << version << " does not match expected version " <<
            ControllerManagerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

//-- Controller Manager -----
ControllerManager::ControllerManager()
    : DeviceTypeManager(1000, 2)
{
}

bool
ControllerManager::startup()
{
    bool success = false;

    if (DeviceTypeManager::startup())
    {
		// Load any config from disk
		cfg.load();

        // Save back out the config in case there were updated defaults
        cfg.save();

        // Initialize HIDAPI
        if (hid_init() == -1)
        {
            PSVR_LOG_ERROR("ControllerManager::startup") << "Failed to initialize HIDAPI";
            success = false;
        }

        success = true;
    }

    if (success)
    {
		if (!bluetooth_get_host_address(m_bluetooth_host_address))
		{
			m_bluetooth_host_address = "00:00:00:00:00:00";
		}
	}

    return success;
}

void
ControllerManager::shutdown()
{
    DeviceTypeManager::shutdown();

	// Shutdown HIDAPI
	hid_exit();
}

void
ControllerManager::notifyVideoFrameReceived(ServerTrackerView* tracker_view)
{
	for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
	{
		ServerControllerViewPtr controllerView = getControllerViewPtr(device_id);

		if (controllerView->getIsOpen())
		{
			controllerView->notifyTrackerDataReceived(tracker_view);
		}
	}
}

void 
ControllerManager::updatePoseFilters()
{
	for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
	{
		ServerControllerViewPtr controllerView = getControllerViewPtr(device_id);

		if (controllerView->getIsOpen())
		{
			controllerView->updatePoseFilter();
		}
	}
}

ServerControllerViewPtr
ControllerManager::getControllerViewPtr(int device_id)
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerControllerView>(m_deviceViews[device_id]);
}

bool
ControllerManager::can_update_connected_devices()
{
    return true;
}

DeviceEnumerator *
ControllerManager::allocate_device_enumerator()
{
    return new ControllerDeviceEnumerator(ControllerDeviceEnumerator::CommunicationType_ALL);
}

void
ControllerManager::free_device_enumerator(DeviceEnumerator *enumerator)
{
    delete static_cast<ControllerDeviceEnumerator *>(enumerator);
}

ServerDeviceView *
ControllerManager::allocate_device_view(int device_id)
{
    return new ServerControllerView(device_id);
}

int 
ControllerManager::getListUpdatedResponseType()
{
    return PSVREvent_controllerListUpdated;
}