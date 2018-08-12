#ifndef WIN_USB_BULK_TRANSFER_BUNDLE_H
#define WIN_USB_BULK_TRANSFER_BUNDLE_H

//-- includes -----
#include "USBApiInterface.h"
#include "USBDeviceRequest.h"

//-- definitions -----
/// Internal class used to manage a set of winusb bulk transfer packets.
class WinUSBBulkTransferBundle : public IUSBBulkTransferBundle
{
public:
    WinUSBBulkTransferBundle(
        const USBDeviceState *device_state,
		const struct USBRequestPayload_BulkTransferBundle *request);
    virtual ~WinUSBBulkTransferBundle();

    // Interface
    bool initialize() override;
    bool startTransfers() override;
    void cancelTransfers() override;

    // Events
    void notifyActiveTransfersDecremented();

    // Accessors
	const USBRequestPayload_BulkTransferBundle &getTransferRequest() const override;
	t_usb_device_handle getUSBDeviceHandle() const override;
	int getActiveTransferCount() const override;

protected:
    void dispose();

private:
    USBRequestPayload_BulkTransferBundle m_request;
    void* m_deviceHandle;
    unsigned char m_bulkInPipe;
	unsigned short m_bulkInPipePacketSize;

    int m_active_transfer_count;
    bool m_is_canceled;
    std::vector<struct WinUSBAsyncBulkTransfer*> bulk_transfer_requests;
    unsigned char* transfer_buffer;
};

#endif // WIN_USB_BULK_TRANSFER_BUNDLE_H