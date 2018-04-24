#ifndef TRACKER_DEVICE_ENUMERATOR_H
#define TRACKER_DEVICE_ENUMERATOR_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "USBApiInterface.h"
#include <string>

//-- definitions -----
class TrackerDeviceEnumerator : public DeviceEnumerator
{
public:
	enum eAPIType
	{
		CommunicationType_INVALID= -1,
		CommunicationType_USB,
		CommunicationType_WMF,
        CommunicationType_VIRTUAL_STEREO,
		CommunicationType_NON_VIRTUAL
	};

    TrackerDeviceEnumerator(eAPIType api_type);
    TrackerDeviceEnumerator(eAPIType api_type, CommonSensorState::eDeviceType deviceTypeFilter);
    TrackerDeviceEnumerator(const std::string &usb_path);
	~TrackerDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;
    
    int get_vendor_id() const override;
	int get_product_id() const override;
    inline int get_camera_index() const { return camera_index; }
    eAPIType get_api_type() const;
    const class VirtualStereoCameraEnumerator *get_virtual_stereo_camera_enumerator() const;
	const class WMFCameraEnumerator *get_windows_media_foundation_camera_enumerator() const;
	const class TrackerUSBDeviceEnumerator *get_usb_tracker_enumerator() const;

protected:
    void allocate_child_enumerator(int enumerator_index);

private:
	eAPIType api_type;
	DeviceEnumerator **enumerators;
	int enumerator_count;
	int enumerator_index;
    int camera_index;
};

#endif // TRACKER_DEVICE_ENUMERATOR_H