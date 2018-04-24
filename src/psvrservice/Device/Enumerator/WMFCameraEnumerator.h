#ifndef WMF_CAMERA_ENUMERATOR_H
#define WMF_CAMERA_ENUMERATOR_H

// -- includes -----
#include "DeviceEnumerator.h"
#include "USBDeviceFilter.h"

#include <vector>
#include <string>
#include <map>

//-- constants -----
#define INVALID_DEVICE_FORMAT_INDEX			-1
#define UNSPECIFIED_CAMERA_WIDTH			0xFFFFFFFF
#define UNSPECIFIED_CAMERA_HEIGHT			0xFFFFFFFF
#define UNSPECIFIED_CAMERA_FPS				0xFFFFFFFF
#define CAMERA_BUFFER_FORMAT_MJPG			L"MFVideoFormat_MJPG"
#define CAMERA_BUFFER_FORMAT_NV12			L"MFVideoFormat_NV12"

// -- definitions -----
struct WMFDeviceFormatInfo
{
	int device_format_index;

	unsigned int frame_size;
	unsigned int height;
	unsigned int width;

	unsigned int yuv_matrix; // MFVideoTransferMatrix Enum
	unsigned int video_lighting; // MFVideoLighting Enum
	int default_stride; // Stride is positive for top-down images, and negative for bottom-up images.
	unsigned int video_chroma_siting; //  MFVideoChromaSubsampling Enum
	
	unsigned int fixed_size_samples;
	unsigned int video_nominal_range;
	unsigned int frame_rate_numerator;
	unsigned int frame_rate_denominator;
	unsigned int pixel_aspect_ratio_horizontal;
	unsigned int pixel_aspect_ratio_vertical;
	unsigned int all_samples_independant;
	unsigned int frame_rate_range_min_numerator;
	unsigned int frame_rate_range_min_denominator;
	unsigned int sample_size;
	unsigned int video_primaries;
	unsigned int interlace_mode;
	unsigned int frame_rate_range_max_numerator;
	unsigned int frame_rate_range_max_denominator;
	
	std::wstring am_format_type_name;
	std::wstring major_type_name;
	std::wstring sub_type_name;
};

struct WMFDeviceInfo
{
	int deviceIndex;
	std::wstring deviceFriendlyName;
	std::wstring deviceSymbolicLink; // can be passed in as the value of the DevicePath argument of the SetupDiOpenDeviceInterface function.
	unsigned int usbVendorId;
	unsigned int usbProductId;
	std::vector<WMFDeviceFormatInfo> deviceAvailableFormats;

	WMFDeviceInfo()
		: deviceIndex(-1)
		, deviceFriendlyName()
		, deviceSymbolicLink()
		, usbVendorId(0)
		, usbProductId(0)
		, deviceAvailableFormats()
	{ }

	int findBestDeviceFormatIndex(
		unsigned int width, unsigned int height,
		unsigned int frameRate, const wchar_t *buffer_format) const;
};

// Enumerates over valid cameras accessible via the Windows Media Foundation API
class WMFCameraEnumerator : public DeviceEnumerator
{
public:
    WMFCameraEnumerator();
	virtual ~WMFCameraEnumerator();

    bool is_valid() const override;
    bool next() override;
	int get_vendor_id() const override;
	int get_product_id() const override;
    const char *get_path() const override;

    inline int get_device_index() const { return m_device_index; }
	const WMFDeviceInfo *get_device_info() const;

private:
	struct WMFDeviceList *m_wmf_device_list;

	USBDeviceFilterSet m_device_filter_set;
	std::string m_current_device_identifier;
    int m_device_index;
};

#endif // WMF_CAMERA_ENUMERATOR_H

