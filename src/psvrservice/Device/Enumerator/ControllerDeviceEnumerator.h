#ifndef CONTROLLER_DEVICE_ENUMERATOR_H
#define CONTROLLER_DEVICE_ENUMERATOR_H

#include "DeviceEnumerator.h"

class ControllerDeviceEnumerator : public DeviceEnumerator
{
public:
	enum eAPIType
	{
		CommunicationType_INVALID= -1,
		CommunicationType_HID,
		CommunicationType_ALL
	};

    ControllerDeviceEnumerator(eAPIType api_type);
    ~ControllerDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;

	int get_vendor_id() const override;
	int get_product_id() const override;
    bool get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const;
	eAPIType get_api_type() const;
	const class ControllerHidDeviceEnumerator *get_hid_controller_enumerator() const;

private:
	eAPIType api_type;
	DeviceEnumerator **enumerators;
	int enumerator_count;
	int enumerator_index;
};

#endif // CONTROLLER_DEVICE_ENUMERATOR_H
