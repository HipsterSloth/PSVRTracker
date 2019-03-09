//-- includes -----
#include "HMDManager.h"
#include "HMDDeviceEnumerator.h"
#include "Logger.h"
#include "PSVRClient_CAPI.h"
#include "ServerHMDView.h"
#include "ServerDeviceView.h"

//-- methods -----
//-- Tracker Manager Config -----
const int HMDManagerConfig::CONFIG_VERSION = 1;

HMDManagerConfig::HMDManagerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
{

};

const configuru::Config
HMDManagerConfig::writeToJSON()
{
    configuru::Config pt{
        {"version", HMDManagerConfig::CONFIG_VERSION}
    };

    return pt;
}

void
HMDManagerConfig::readFromJSON(const configuru::Config &pt)
{
    version = pt.get_or<int>("version", 0);

    if (version == HMDManagerConfig::CONFIG_VERSION)
    {
    }
    else
    {
        PSVR_LOG_WARNING("HMDManagerConfig") <<
            "Config version " << version << " does not match expected version " <<
            HMDManagerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

//-- HMD Manager -----
HMDManager::HMDManager()
    : DeviceTypeManager(1000, 2)
{
}

bool
HMDManager::startup()
{
    bool success = false;

    if (DeviceTypeManager::startup())
    {
		// Load any config from disk
		cfg.load();

        // Save back out the config in case there were updated defaults
        cfg.save();

        success = true;
    }

    return success;
}

void
HMDManager::shutdown()
{
    DeviceTypeManager::shutdown();
}

void
HMDManager::notifyVideoFrameReceived(ServerTrackerView* tracker_view)
{
	for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
	{
		ServerHMDViewPtr hmdView = getHMDViewPtr(device_id);

		if (hmdView->getIsOpen())
		{
			hmdView->notifyTrackerDataReceived(tracker_view);
		}
	}
}

void 
HMDManager::updatePoseFilters()
{
	for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
	{
		ServerHMDViewPtr hmdView = getHMDViewPtr(device_id);

		if (hmdView->getIsOpen())
		{
			hmdView->updatePoseFilter();
		}
	}
}

ServerHMDViewPtr
HMDManager::getHMDViewPtr(int device_id)
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerHMDView>(m_deviceViews[device_id]);
}

bool
HMDManager::can_update_connected_devices()
{
    return true;
}

DeviceEnumerator *
HMDManager::allocate_device_enumerator()
{
    return new HMDDeviceEnumerator(HMDDeviceEnumerator::CommunicationType_ALL);
}

void
HMDManager::free_device_enumerator(DeviceEnumerator *enumerator)
{
    delete static_cast<HMDDeviceEnumerator *>(enumerator);
}

ServerDeviceView *
HMDManager::allocate_device_view(int device_id)
{
    return new ServerHMDView(device_id);
}

int 
HMDManager::getListUpdatedResponseType()
{
    return PSVREvent_hmdListUpdated;
}