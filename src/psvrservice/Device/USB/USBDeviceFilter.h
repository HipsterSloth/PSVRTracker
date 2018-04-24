#ifndef USB_DEVICE_FILTER_H
#define USB_DEVICE_FILTER_H

//-- includes -----
#include <vector>

// -- macros ----
#define MAX_USB_DEVICE_PORT_PATH 7
#define GET_DEVICE_TYPE_INDEX(device_type)  (device_type & 0x0f)
#define GET_DEVICE_TYPE_CLASS(device_type)  (device_type & 0xf0)

#define ANY_INTERFACE_MASK 0xFFFFFFFF

//-- definitions -----
struct USBDeviceFilter
{
    unsigned short vendor_id;
    unsigned short product_id;
    unsigned int interface_mask;
};

class USBDeviceFilterSet
{
public:
	bool addFilter(unsigned short vendor_id, unsigned short product_id);
	bool addFilter(unsigned short vendor_id, unsigned short product_id, unsigned int interface_mask);
	bool containsFilter(unsigned short vendor_id, unsigned short product_id, unsigned int interface_mask);
	bool passesFilter(unsigned short vendor_id, unsigned short product_id);
	bool passesFilter(unsigned short vendor_id, unsigned short product_id, unsigned int interface_mask);

private:
	std::vector<USBDeviceFilter> filters;
};

#endif  // USB_DEVICE_FILTER_H