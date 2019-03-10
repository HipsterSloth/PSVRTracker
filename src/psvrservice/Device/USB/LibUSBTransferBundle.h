#ifndef LIB_USB_BULK_TRANSFER_BUNDLE_H
#define LIB_USB_BULK_TRANSFER_BUNDLE_H

//-- includes -----
#include "USBApiInterface.h"
#include "USBDeviceRequest.h"

#include <vector>

//-- definitions -----
/// Internal class used to manage a set of libusb bulk transfer packets.
class LibUSBTransferBundle : public IUSBTransferBundle
{
public:
    LibUSBTransferBundle(
        const USBDeviceState *device_state,
		const struct USBRequestPayload_TransferBundle *request);
    virtual ~LibUSBTransferBundle();

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

    // Helpers
    // Search for an input transfer endpoint in the endpoint descriptor
    // of the device interfaces alt_settings
    static bool find_transfer_endpoint(
        eUSBTransferBundleType bundle_type, 
        struct libusb_device *device, 
        int interface_index, 
        unsigned char &out_endpoint_addr);

protected:
    void dispose();

private:
    USBRequestPayload_TransferBundle m_request;
    struct libusb_device *m_device;
    struct libusb_device_handle *m_device_handle;
    int m_interface_index;

    int m_active_transfer_count;
    bool m_is_canceled;
    std::vector<struct libusb_transfer*> transfer_requests;
    unsigned char* transfer_buffer;
};

#endif // USB_BULK_TRANSFER_BUNDLE_H