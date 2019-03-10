#ifndef HMD_HID_DEVICE_ENUMERATOR_H
#define HMD_HID_DEVICE_ENUMERATOR_H

// -- includes -----
#include "DeviceEnumerator.h"
#include <vector>
#include <string>

// -- definitions -----
struct HMDHidDeviceInterface
{
	std::string device_path;
	int interface_number;
};

class HMDHidDeviceEnumerator : public DeviceEnumerator
{
public:
    HMDHidDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
	int get_vendor_id() const override;
	int get_product_id() const override;
    const char *get_path() const override;

	std::string get_hid_interface_path() const;

private:
	void find_first_valid_hid_interface();

	std::string current_device_identifier;
	HMDHidDeviceInterface current_device_interface_info;
};

#endif // HID_HMD_DEVICE_ENUMERATOR_H
