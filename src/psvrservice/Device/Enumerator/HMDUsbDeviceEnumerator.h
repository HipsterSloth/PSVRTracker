#ifndef HMD_USB_DEVICE_ENUMERATOR_H
#define HMD_USB_DEVICE_ENUMERATOR_H

#include "DeviceEnumerator.h"
#include "USBApiInterface.h"

class HMDUsbDeviceEnumerator : public DeviceEnumerator
{
public:

    HMDUsbDeviceEnumerator();
    HMDUsbDeviceEnumerator(CommonSensorState::eDeviceType deviceTypeFilter);
    HMDUsbDeviceEnumerator(const std::string &usb_path);
	~HMDUsbDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
	int get_vendor_id() const override;
	int get_product_id() const override;
    const char *get_path() const override;
	const char *get_unique_identifier() const;
    inline int get_hmd_index() const { return m_hmdIndex; }
	inline struct USBDeviceEnumerator* get_usb_device_enumerator() const { return m_usb_enumerator; }

protected: 
	bool testUSBEnumerator();

private:
    char m_currentUSBPath[256];
	char m_currentUSBIdentifier[256];
	eUSBApiType m_currentDriverType;
	struct USBDeviceEnumerator* m_usb_enumerator;
    int m_hmdIndex;
};

#endif // HMD_DEVICE_ENUMERATOR_H
