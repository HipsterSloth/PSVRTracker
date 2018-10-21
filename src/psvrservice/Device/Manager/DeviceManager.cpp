//-- includes -----
#include "DeviceManager.h"

#include "DeviceEnumerator.h"
#include "ControllerManager.h"
#include "HMDManager.h"
#include "OrientationFilter.h"
#ifdef WIN32
#include "PlatformDeviceAPIWin32.h"
#endif // WIN32
#include "ServerControllerView.h"
#include "ServerHMDView.h"
#include "ServerTrackerView.h"
#include "ServiceRequestHandler.h"
#include "Logger.h"
#include "ServerDeviceView.h"
#include "Utility.h"
#include "PSVRConfig.h"
#include "TrackerManager.h"

#include <chrono>

//-- constants -----
static const int k_default_controller_reconnect_interval= 1000; // ms
static const int k_default_controller_poll_interval= 2; // ms
static const int k_default_tracker_reconnect_interval= 10000; // ms
static const int k_default_tracker_poll_interval= 13; // 1000/75 ms
static const int k_default_hmd_reconnect_interval= 10000; // ms
static const int k_default_hmd_poll_interval= 2; // ms

class DeviceManagerConfig : public PSVRConfig
{
public:
    static const int CONFIG_VERSION= 1;

    DeviceManagerConfig(const std::string &fnamebase = "DeviceManagerConfig")
        : PSVRConfig(fnamebase)
        , controller_reconnect_interval(k_default_controller_reconnect_interval)
        , controller_poll_interval(k_default_controller_poll_interval)
        , tracker_reconnect_interval(k_default_tracker_reconnect_interval)
        , tracker_poll_interval(k_default_tracker_poll_interval)
        , hmd_reconnect_interval(k_default_hmd_reconnect_interval)
        , hmd_poll_interval(k_default_hmd_poll_interval)
		, gamepad_api_enabled(true)
		, platform_api_enabled(true)
    {};

    const configuru::Config
    writeToJSON()
    {
        configuru::Config pt{
            {"version", DeviceManagerConfig::CONFIG_VERSION+0},
			{"controller_reconnect_interval", controller_reconnect_interval},
			{"controller_poll_interval", controller_poll_interval},
            {"tracker_reconnect_interval", tracker_reconnect_interval},
            {"tracker_poll_interval", tracker_poll_interval},
            {"hmd_reconnect_interval", hmd_reconnect_interval},
            {"hmd_poll_interval", hmd_poll_interval},
		    {"gamepad_api_enabled", gamepad_api_enabled},
		    {"platform_api_enabled", platform_api_enabled}
        };
    
        return pt;
    }

    void
    readFromJSON(const configuru::Config &pt)
    {
        version = pt.get_or<int>("version", 0);

        if (version == (DeviceManagerConfig::CONFIG_VERSION+0))
        {
            controller_reconnect_interval = pt.get_or<int>("controller_reconnect_interval", k_default_controller_reconnect_interval);
            controller_poll_interval = pt.get_or<int>("controller_poll_interval", k_default_controller_poll_interval);
            tracker_reconnect_interval = pt.get_or<int>("tracker_reconnect_interval", k_default_tracker_reconnect_interval);
            tracker_poll_interval = pt.get_or<int>("tracker_poll_interval", k_default_tracker_poll_interval);
            hmd_reconnect_interval = pt.get_or<int>("hmd_reconnect_interval", k_default_hmd_reconnect_interval);
            hmd_poll_interval = pt.get_or<int>("hmd_poll_interval", k_default_hmd_poll_interval);
		    gamepad_api_enabled = pt.get_or<bool>("gamepad_api_enabled", gamepad_api_enabled);
		    platform_api_enabled = pt.get_or<bool>("platform_api_enabled", platform_api_enabled);
        }
        else
        {
            PSVR_LOG_WARNING("DeviceManagerConfig") <<
                "Config version " << version << " does not match expected version " <<
                (DeviceManagerConfig::CONFIG_VERSION+0) << ", Using defaults.";
        }
    }

    int version;
    int controller_reconnect_interval;
    int controller_poll_interval;
    int tracker_reconnect_interval;
    int tracker_poll_interval;
    int hmd_reconnect_interval;
    int hmd_poll_interval;    
	bool gamepad_api_enabled;
	bool platform_api_enabled;
};

// DeviceManager - This is the interface used by PSVRSERVICE
DeviceManager *DeviceManager::m_instance= nullptr;

DeviceManager::DeviceManager()
    : m_config() // NULL config until startup
	, m_platform_api_type(_eDevicePlatformApiType_None)
	, m_platform_api(nullptr)
	, m_controller_manager(new ControllerManager())
    , m_tracker_manager(new TrackerManager())
    , m_hmd_manager(new HMDManager())
{
}

DeviceManager::~DeviceManager()
{
	delete m_controller_manager;
    delete m_tracker_manager;
    delete m_hmd_manager;

	if (m_platform_api != nullptr)
	{
		delete m_platform_api;
	}
}

bool
DeviceManager::startup()
{
    bool success= true;

    m_config = DeviceManagerConfigPtr(new DeviceManagerConfig);

	// Load the config from disk
    m_config->load();

	// Save the config back out again in case defaults changed
	m_config->save();
    
	// Optionally create the platform device hot plug API
	if (m_config->platform_api_enabled)
	{
#ifdef WIN32
		m_platform_api_type = _eDevicePlatformApiType_Win32;
		m_platform_api = new PlatformDeviceAPIWin32;
#endif
		PSVR_LOG_INFO("DeviceManager::startup") << "Platform Hotplug API is ENABLED";
	}
	else
	{
		PSVR_LOG_INFO("DeviceManager::startup") << "Platform Hotplug API is DISABLED";
	}

	if (m_platform_api != nullptr)
	{
		success &= m_platform_api->startup(this);
	}

	// Register for hotplug events if this platform supports them
	int controller_reconnect_interval = m_config->controller_reconnect_interval;
	int tracker_reconnect_interval = m_config->tracker_reconnect_interval;
	int hmd_reconnect_interval = m_config->hmd_reconnect_interval;
	if (success && m_platform_api_type != _eDevicePlatformApiType_None)
	{
		registerHotplugListener(DeviceCategory_CONTROLLER, m_controller_manager);
		controller_reconnect_interval = -1;

		registerHotplugListener(DeviceCategory_TRACKER, m_tracker_manager);
		tracker_reconnect_interval = -1;

		registerHotplugListener(DeviceCategory_HMD, m_hmd_manager);
		hmd_reconnect_interval = -1;
	}

	m_controller_manager->reconnect_interval = controller_reconnect_interval;
    m_controller_manager->poll_interval = m_config->controller_poll_interval;
	success &= m_controller_manager->startup();


    m_tracker_manager->reconnect_interval = tracker_reconnect_interval;
    m_tracker_manager->poll_interval = m_config->tracker_poll_interval;
    success &= m_tracker_manager->startup();

    m_hmd_manager->reconnect_interval = hmd_reconnect_interval;
    m_hmd_manager->poll_interval = m_config->hmd_poll_interval;
    success &= m_hmd_manager->startup();    
    
    m_instance= this;
    
    return success;
}

void
DeviceManager::update()
{
	if (m_platform_api != nullptr)
	{
		m_platform_api->pollSystemEvents(); // Send device hotplug events
	}

	m_controller_manager->pollConnectedDevices(); // Update controller count
    m_tracker_manager->pollConnectedDevices(); // Update tracker count
    m_hmd_manager->pollConnectedDevices(); // Update HMD count

	m_tracker_manager->pollUpdatedVideoFrames(); // Check for updated video frames
	m_controller_manager->updatePoseFilters(); // Process pose filter packets from the tracker and IMU threads
	m_hmd_manager->updatePoseFilters(); // Process pose filter packets from the tracker and IMU threads

    m_tracker_manager->publish(); // publish tracker state to any listening clients (probably only used by ConfigTool)
	m_controller_manager->publish(); // publish controller state to any listening clients (common case)
    m_hmd_manager->publish(); // publish hmd state to any listening clients (common case)
}

void
DeviceManager::shutdown()
{
	if (m_config)
	{
		m_config->save();
	}

	if (m_tracker_manager != nullptr)
	{
	    m_tracker_manager->shutdown();
	}

	if (m_controller_manager != nullptr)
	{
	    m_controller_manager->shutdown();
	}

	if (m_hmd_manager != nullptr)
	{
	    m_hmd_manager->shutdown();
	}

	if (m_platform_api != nullptr)
	{
		m_platform_api->shutdown();
	}

    m_instance= nullptr;
}

// -- Queries ---
bool 
DeviceManager::get_device_property(
	const DeviceClass deviceClass,
	const int vendor_id,
	const int product_id,
	const char *property_name,
	char *buffer,
	const int buffer_size)
{
	bool bSuccess = false;

	if (m_platform_api != nullptr)
	{
		bSuccess = m_platform_api->get_device_property(deviceClass, vendor_id, product_id, property_name, buffer, buffer_size);
	}

	return bSuccess;
}

int 
DeviceManager::getControllerViewMaxCount() const
{
    return m_controller_manager->getMaxDevices();
}

int 
DeviceManager::getTrackerViewMaxCount() const
{
    return m_tracker_manager->getMaxDevices();
}

int 
DeviceManager::getHMDViewMaxCount() const
{
    return m_hmd_manager->getMaxDevices();
}

ServerControllerViewPtr
DeviceManager::getControllerViewPtr(int device_id)
{
    ServerControllerViewPtr result;
    if (Utility::is_index_valid(device_id, m_controller_manager->getMaxDevices()))
    {
        result = m_controller_manager->getControllerViewPtr(device_id);
    }

    return result;
}

ServerTrackerViewPtr
DeviceManager::getTrackerViewPtr(int tracker_id)
{
    ServerTrackerViewPtr result;
    if (Utility::is_index_valid(tracker_id, m_tracker_manager->getMaxDevices()))
    {
        result = m_tracker_manager->getTrackerViewPtr(tracker_id);
    }

    return result;
}

ServerHMDViewPtr
DeviceManager::getHMDViewPtr(int hmd_id)
{
    ServerHMDViewPtr result;
    if (Utility::is_index_valid(hmd_id, m_hmd_manager->getMaxDevices()))
    {
        result = m_hmd_manager->getHMDViewPtr(hmd_id);
    }

    return result;
}

// -- Notification --
void
DeviceManager::registerHotplugListener(enum DeviceCategory device_category, IDeviceHotplugListener *listener)
{
	DeviceHotplugListener entry;
	entry.listener = listener;

	switch (device_category)
	{
	case DeviceCategory_CONTROLLER:
	case DeviceCategory_HMD:
		entry.device_class = DeviceClass::DeviceClass_HID;
		break;
	case DeviceCategory_TRACKER:
		entry.device_class = DeviceClass::DeviceClass_Camera;
		break;
	default:
		break;
	}

	m_listeners.push_back(entry);
}

void
DeviceManager::handle_device_connected(enum DeviceClass device_class, const std::string &device_path)
{
	for (auto it = m_listeners.begin(); it != m_listeners.end(); ++it)
	{
		if (it->device_class == device_class)
		{
			it->listener->handle_device_connected(device_class, device_path);
		}
	}
}

void
DeviceManager::handle_device_disconnected(enum DeviceClass device_class, const std::string &device_path)
{
	for (auto it = m_listeners.begin(); it != m_listeners.end(); ++it)
	{
		if (it->device_class == device_class)
		{
			it->listener->handle_device_disconnected(device_class, device_path);
		}
	}
}

void 
DeviceManager::handle_bluetooth_request_started()
{
	if (m_platform_api != nullptr)
	{
		m_platform_api->handle_bluetooth_request_started();
	}
}

void
DeviceManager::handle_bluetooth_request_finished()
{
	if (m_platform_api != nullptr)
	{
		m_platform_api->handle_bluetooth_request_finished();
	}

	//TODO: This is a bit of a hack to force the controller list to refresh
	for (auto it = m_listeners.begin(); it != m_listeners.end(); ++it)
	{
		if (it->device_class == DeviceClass::DeviceClass_HID)
		{
			it->listener->handle_device_connected(DeviceClass::DeviceClass_HID, "");
		}
	}
}