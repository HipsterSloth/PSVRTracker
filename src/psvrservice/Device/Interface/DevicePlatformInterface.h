#ifndef DEVICE_PLATFORM_INTERFACE_H
#define DEVICE_PLATFORM_INTERFACE_H

// -- include -----
#include <string>

// -- constants -----
enum DeviceClass
{
	DeviceClass_INVALID= -1,

	DeviceClass_Camera,
	DeviceClass_HID,
    DeviceClass_RawUSB,
    DeviceClass_LibUSB,

	k_max_supported_device_classes
};

// -- interfaces -----
class IDeviceHotplugListener
{
public:
	virtual void handle_device_connected(enum DeviceClass device_class, const std::string &device_path) = 0;
	virtual void handle_device_disconnected(enum DeviceClass device_class, const std::string &device_path) = 0;
};

class IPlatformDeviceAPI
{
public:
	IPlatformDeviceAPI() {}
	virtual ~IPlatformDeviceAPI() {}

	// System
	virtual bool startup(IDeviceHotplugListener *broadcaster) = 0;
	virtual void pollSystemEvents() = 0;
	virtual void shutdown() = 0;

	// Events
	virtual void handle_bluetooth_request_started() {};
	virtual void handle_bluetooth_request_finished() {};

	// Queries
    virtual const void* get_device_class_platform_identifier(const DeviceClass deviceClass) const = 0;
	virtual bool get_device_property(
		const DeviceClass deviceClass,
		const int vendor_id,
		const int product_id,
		const char *property_name,
		char *buffer,
		const int buffer_size) = 0;
};

#endif // DEVICE_PLATFORM_INTERFACE_H