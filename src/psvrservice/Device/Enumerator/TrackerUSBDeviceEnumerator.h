#ifndef TRACKER_USB_DEVICE_ENUMERATOR_H
#define TRACKER_USB_DEVICE_ENUMERATOR_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "USBApiInterface.h"

//-- definitions -----
class TrackerUSBDeviceEnumerator : public DeviceEnumerator
{
public:
	static class TrackerCapabilitiesSet *s_supportedTrackers;

    TrackerUSBDeviceEnumerator();
    TrackerUSBDeviceEnumerator(CommonSensorState::eDeviceType deviceTypeFilter);
    TrackerUSBDeviceEnumerator(const std::string &usb_path);
	~TrackerUSBDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
	int get_vendor_id() const override;
	int get_product_id() const override;
    const char *get_path() const override;
    inline int get_camera_index() const { return m_cameraIndex; }
	inline struct USBDeviceEnumerator* get_usb_device_enumerator() const { return m_usb_enumerator; }
	const class TrackerCapabilitiesConfig *getTrackerCapabilities() const;

protected: 
	bool testUSBEnumerator();

private:
    char m_currentUSBPath[256];
	struct USBDeviceEnumerator* m_usb_enumerator;
    int m_cameraIndex;
};

#endif // TRACKER_USB_DEVICE_ENUMERATOR_H
