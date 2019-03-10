#ifndef HMD_DEVICE_ENUMERATOR_H
#define HMD_DEVICE_ENUMERATOR_H

#include "DeviceEnumerator.h"

class HMDDeviceEnumerator : public DeviceEnumerator
{
public:
	enum eAPIType
	{
		CommunicationType_INVALID= -1,
        CommunicationType_HID,
		CommunicationType_USB,
		CommunicationType_ALL
	};

    HMDDeviceEnumerator(eAPIType api_type);
    ~HMDDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;

	int get_vendor_id() const override;
	int get_product_id() const override;
	eAPIType get_api_type() const;
	const class HMDHidDeviceEnumerator *get_hmd_hid_enumerator() const;
    const class HMDUsbDeviceEnumerator *get_hmd_usb_enumerator() const;

private:
	eAPIType m_apiType;
	DeviceEnumerator **m_enumerators;
	int m_enumeratorCount;
	int m_enumeratorIndex;
    bool m_foundAnyValidHMD;
};

#endif // HMD_DEVICE_ENUMERATOR_H
