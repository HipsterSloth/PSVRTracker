#ifndef USB_API_INTERFACE_H
#define USB_API_INTERFACE_H

#include <stddef.h>

//-- constants -----
enum eUSBResultCode
{
	// Success Codes
	_USBResultCode_Started,
	_USBResultCode_Canceled,
	_USBResultCode_Completed,

	// Failure Codes
	_USBResultCode_GeneralError,
	_USBResultCode_BadHandle,
	_USBResultCode_NoMemory,
	_USBResultCode_SubmitFailed,
	_USBResultCode_DeviceNotOpen,
	_USBResultCode_TransferNotActive,
	_USBResultCode_TransferAlreadyStarted,
	_USBResultCode_Overflow,
	_USBResultCode_Pipe,
	_USBResultCode_TimedOut,
	_USBResultCode_InvalidAPI
};

enum eUSBApiType : short
{
	_USBApiType_INVALID= -1,

	_USBApiType_LibUSB,
	_USBApiType_WinUSB,

    _USBApiType_COUNT
};


//-- typedefs -----
struct t_usb_device_handle
{
    eUSBApiType api_type;
    short unique_id;

    inline bool operator < (const t_usb_device_handle &other) const { return unique_id < other.unique_id; }
    inline bool operator > (const t_usb_device_handle &other) const { return unique_id > other.unique_id; }
    inline bool operator == (const t_usb_device_handle &other) const { return unique_id == other.unique_id; }
    inline bool operator != (const t_usb_device_handle &other) const { return unique_id != other.unique_id; }
};
const t_usb_device_handle k_invalid_usb_device_handle = {_USBApiType_INVALID, -1};

//-- macros -----
//#define DEBUG_USB
#if defined(DEBUG_USB)
#define debug(...) fprintf(stdout, __VA_ARGS__)
#else
#define debug(...) 
#endif

//-- definitions -----
struct USBDeviceEnumerator
{
    eUSBApiType api_type;
	int device_index;
};

struct USBDeviceState
{
	t_usb_device_handle public_handle;

	void clear()
	{
		public_handle= k_invalid_usb_device_handle;
	}
};

//-- interface -----
class IUSBApi
{
public:
	IUSBApi() {}
	virtual ~IUSBApi() {}

	virtual eUSBApiType getRuntimeUSBApiType() const = 0;

	virtual bool startup() = 0;
	virtual void poll() = 0;
	virtual void shutdown() = 0;

	virtual USBDeviceEnumerator* device_enumerator_create() = 0;
	virtual bool device_enumerator_get_filter(const USBDeviceEnumerator* enumerator, struct USBDeviceFilter *outDeviceInfo) const = 0;
	virtual bool device_enumerator_get_path(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const = 0;
	virtual bool device_enumerator_get_unique_identifier(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const = 0;
	virtual bool device_enumerator_is_valid(USBDeviceEnumerator* enumerator) = 0;
	virtual void device_enumerator_next(USBDeviceEnumerator* enumerator) = 0;
	virtual void device_enumerator_dispose(USBDeviceEnumerator* enumerator) = 0;

	virtual USBDeviceState *open_usb_device(
        USBDeviceEnumerator* enumerator,
        int interface_index,
        int configuration_index,
        bool reset_device) = 0;
	virtual void close_usb_device(USBDeviceState* device_state) = 0;
	virtual bool can_usb_device_be_opened(struct USBDeviceEnumerator* enumerator, char *outReason, size_t bufferSize) = 0;

	virtual eUSBResultCode submit_interrupt_transfer(const USBDeviceState* device_state, const struct USBTransferRequestState *requestStateOnHeap) = 0;
	virtual eUSBResultCode submit_control_transfer(const USBDeviceState* device_state, const struct USBTransferRequestState *requestStateOnHeap) = 0;
    virtual eUSBResultCode submit_bulk_transfer(const USBDeviceState* device_state, const struct USBTransferRequestState *requestStateOnHeap) = 0;
	virtual class IUSBBulkTransferBundle *allocate_bulk_transfer_bundle(const USBDeviceState *device_state, const struct USBRequestPayload_BulkTransferBundle *request) = 0;

	virtual bool get_usb_device_filter(const USBDeviceState* device_state, struct USBDeviceFilter *outDeviceInfo) const = 0;
	virtual bool get_usb_device_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const = 0;
	virtual bool get_usb_device_port_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const = 0;
};

class IUSBBulkTransferBundle
{
public:
	IUSBBulkTransferBundle(const USBDeviceState *device_state, const struct USBRequestPayload_BulkTransferBundle *request) {}
	virtual ~IUSBBulkTransferBundle() {}

	// Interface
	virtual bool initialize() = 0;
	virtual bool startTransfers() = 0;
	virtual void cancelTransfers() = 0;

	// Accessors
	virtual const USBRequestPayload_BulkTransferBundle &getTransferRequest() const = 0;
	virtual t_usb_device_handle getUSBDeviceHandle() const = 0;
	virtual int getActiveTransferCount() const = 0;
};

#endif // USB_API_INTERFACE_H
