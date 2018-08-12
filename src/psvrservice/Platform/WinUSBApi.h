#ifndef WIN_USB_API_H
#define WIN_USB_API_H

#include "USBApiInterface.h"
#include "USBDeviceRequest.h"

enum eWinusbBulkTransferStatus 
{
	WINUSB_TRANSFER_PENDING,

	/** Transfer completed without error. Note that this does not indicate
	 * that the entire amount of requested data was transferred. */
	WINUSB_TRANSFER_COMPLETED,

	/** Transfer failed */
	WINUSB_TRANSFER_ERROR,

	/** Transfer timed out */
	WINUSB_TRANSFER_TIMED_OUT,

	/** Transfer was cancelled */
	WINUSB_TRANSFER_CANCELLED,

	/** halt condition detected (endpoint stalled). */
	WINUSB_TRANSFER_STALL,

	/** Device was disconnected */
	WINUSB_TRANSFER_NO_DEVICE,

	/** Device sent more data than requested */
	WINUSB_TRANSFER_OVERFLOW,
};

typedef std::function<void(struct WinUSBAsyncBulkTransfer *, eWinusbBulkTransferStatus, unsigned char *, size_t, void *)> t_winusb_bulk_transfer_callback;

struct WinUSBDeviceState : USBDeviceState
{
    std::string device_path;
    std::string unique_id;
    void* device_handle;
    void* interface_handle;
    unsigned char device_speed;
    unsigned char bulk_in_pipe;
	unsigned short bulk_in_pipe_packet_size;
    unsigned char bulk_out_pipe;
    unsigned char interrupt_in_pipe;
    unsigned char interrupt_out_pipe;
    unsigned char isochronous_in_pipe;
    unsigned char isochronous_out_pipe;
    int product_id;
    int vendor_id;
    int composite_interface_index;

	void clear()
	{
		USBDeviceState::clear();

        device_path= "";
        unique_id= "";
        device_handle= (void *)-1;
        interface_handle= NULL;
        device_speed= 0;
        bulk_in_pipe= 0xFF;
		bulk_in_pipe_packet_size= 0;
        bulk_out_pipe= 0xFF;
        interrupt_in_pipe= 0xFF;
        interrupt_out_pipe= 0xFF;
        isochronous_in_pipe= 0xFF;
        isochronous_out_pipe= 0xFF;
        product_id= -1;
        vendor_id= -1;
        composite_interface_index= -1;
	}
};

class WinUSBApi : public IUSBApi
{
public:
	WinUSBApi();

	eUSBApiType getRuntimeUSBApiType() const override { return _USBApiType_WinUSB; }
	static eUSBApiType getStaticUSBApiType() { return _USBApiType_WinUSB; }
	static WinUSBApi *getInterface();

	// IUSBApi
	bool startup() override;
	void poll() override;
	void shutdown() override;

	USBDeviceEnumerator* device_enumerator_create() override;
	bool device_enumerator_get_filter(const USBDeviceEnumerator* enumerator, struct USBDeviceFilter *outDeviceInfo) const override;
	bool device_enumerator_get_path(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const override;
	bool device_enumerator_is_valid(USBDeviceEnumerator* enumerator) override;
	void device_enumerator_next(USBDeviceEnumerator* enumerator) override;
	void device_enumerator_dispose(USBDeviceEnumerator* enumerator) override;

	USBDeviceState *open_usb_device(
        USBDeviceEnumerator* enumerator,
        int interface_index,
        int configuration_index,
        bool reset_device) override;
	void close_usb_device(USBDeviceState* device_state) override;
	bool can_usb_device_be_opened(struct USBDeviceEnumerator* enumerator, char *outReason, size_t bufferSize) override;

	eUSBResultCode submit_interrupt_transfer(const USBDeviceState* device_state, const struct USBTransferRequestState *requestState) override;
	eUSBResultCode submit_control_transfer(const USBDeviceState* device_state, const struct USBTransferRequestState *requestState) override;
    eUSBResultCode submit_bulk_transfer(const USBDeviceState* device_state, const struct USBTransferRequestState *requestState) override;
	IUSBBulkTransferBundle *allocate_bulk_transfer_bundle(const USBDeviceState *device_state, const struct USBRequestPayload_BulkTransferBundle *request) override;

	bool get_usb_device_filter(const USBDeviceState* device_state, struct USBDeviceFilter *outDeviceInfo) const override;
	bool get_usb_device_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const override;
	bool get_usb_device_port_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const override;

	// WinUSBApi
	struct WinUSBAsyncBulkTransfer * winusbAllocateAsyncBulkTransfer();
	bool winusbSetupAsyncBulkTransfer(
		void * device_handle,
		const unsigned char bulk_endpoint,
		unsigned char *transfer_buffer,
		const size_t transfer_packet_size,
		t_winusb_bulk_transfer_callback transfer_callback_function,
		void *userdata,
		struct WinUSBAsyncBulkTransfer *transfer);
	void winusbFreeAsyncBulkTransfer(struct WinUSBAsyncBulkTransfer *transfer);
	bool winusbSubmitAsyncBulkTransfer(struct WinUSBAsyncBulkTransfer *transfer);
	bool winusbCancelAsyncBulkTransfer(struct WinUSBAsyncBulkTransfer *transfer);

private:
	std::vector<struct WinUSBAsyncBulkTransfer *> m_pendingAsyncBulkTransfers;

	eWinusbBulkTransferStatus winusbPollAsyncBulkTransfer(struct WinUSBAsyncBulkTransfer *transfer);
};

#endif // WIN_USB_API_H

