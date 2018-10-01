#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

//-- includes -----
#include "DeviceInterface.h"
#include "DevicePlatformInterface.h"
#include "PSVRServiceInterface.h"
#include <memory>
#include <chrono>
#include <vector>

//-- constants -----
enum eDevicePlatformApiType
{
	_eDevicePlatformApiType_None,
#ifdef WIN32
	_eDevicePlatformApiType_Win32,
#endif // WIN32
};

//-- typedefs -----
class DeviceManagerConfig;
typedef std::shared_ptr<DeviceManagerConfig> DeviceManagerConfigPtr;

class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;

class ServerTrackerView;
typedef std::shared_ptr<ServerTrackerView> ServerTrackerViewPtr;

class ServerHMDView;
typedef std::shared_ptr<ServerHMDView> ServerHMDViewPtr;

//-- definitions -----
struct DeviceHotplugListener
{
	DeviceClass device_class;
	IDeviceHotplugListener *listener;
};

/// This is the class that is actually used by the PSVRSERVICE.
class DeviceManager : public IDeviceHotplugListener
{
public:
    DeviceManager();
    virtual ~DeviceManager();

	// -- System ----
    bool startup(); /**< Initialize the interfaces for each specific manager. */
    void update();  /**< Poll all connected devices for each specific manager. */
    void shutdown();/**< Shutdown the interfaces for each specific manager. */

    static inline DeviceManager *getInstance()
    { return m_instance; }

	// -- Accessors ---
	class ControllerManager *getControllerManager() { return m_controller_manager; }
	ServerControllerViewPtr getControllerViewPtr(int controller_id);

	class TrackerManager *getTrackerManager() { return m_tracker_manager; }
	ServerTrackerViewPtr getTrackerViewPtr(int tracker_id);

	class HMDManager *getHMDManager() { return m_hmd_manager; }
	ServerHMDViewPtr getHMDViewPtr(int hmd_id);

	// -- Queries ---
	inline eDevicePlatformApiType get_api_type() const { return m_platform_api_type; }
	bool get_device_property(
		const DeviceClass deviceClass,
		const int vendor_id,
		const int product_id,
		const char *property_name,
		char *buffer,
		const int buffer_size);

	int getControllerViewMaxCount() const;
    int getTrackerViewMaxCount() const;
    int getHMDViewMaxCount() const;       

	// -- Notification --
	void registerHotplugListener(const DeviceCategory deviceClass, IDeviceHotplugListener *listener);
	void handle_device_connected(enum DeviceClass device_class, const std::string &device_path) override;
	void handle_device_disconnected(enum DeviceClass device_class, const std::string &device_path) override;
	void handle_bluetooth_request_started();
	void handle_bluetooth_request_finished();
    
private:
	/// Singleton instance of the class
	/// Assigned in startup, cleared in teardown
	static DeviceManager *m_instance;

	DeviceManagerConfigPtr m_config;

	// OS specific implementation of hotplug notification
	eDevicePlatformApiType m_platform_api_type;
	IPlatformDeviceAPI *m_platform_api;

	// List of registered hot-plug listeners
	std::vector<DeviceHotplugListener> m_listeners;

public:
	class ControllerManager *m_controller_manager;
    class TrackerManager *m_tracker_manager;
    class HMDManager *m_hmd_manager;
};

#endif  // DEVICE_MANAGER_H