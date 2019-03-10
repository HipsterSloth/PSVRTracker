#ifndef WIN_USB_TRANSFER_BUNDLE_H
#define WIN_USB_TRANSFER_BUNDLE_H

//-- includes -----
#include "USBApiInterface.h"
#include "USBDeviceRequest.h"

//-- definitions -----
/// Internal class used to manage a set of winusb bulk transfer packets.
class WinUSBTransferBundle : public IUSBTransferBundle
{
public:
    WinUSBTransferBundle(
        const USBDeviceState *device_state,
		const struct USBRequestPayload_TransferBundle *request);
    virtual ~WinUSBTransferBundle();

    // Interface
    bool initialize() override;
    bool startTransfers() override;
    void cancelTransfers() override;

    // Events
    void notifyActiveTransfersDecremented();

    // Accessors
	const USBRequestPayload_TransferBundle &getTransferRequest() const override;
	t_usb_device_handle getUSBDeviceHandle() const override;
	int getActiveTransferCount() const override;

protected:
    void dispose();

private:
    USBRequestPayload_TransferBundle m_request;
    void* m_deviceHandle;
	void* m_interfaceHandle;
    unsigned char m_transferInPipe;

    int m_active_transfer_count;
    bool m_is_canceled;
    std::vector<struct WinUSBAsyncTransfer*> transfer_requests;
    unsigned char* transfer_buffer;
};

#endif // WIN_USB_BULK_TRANSFER_BUNDLE_H