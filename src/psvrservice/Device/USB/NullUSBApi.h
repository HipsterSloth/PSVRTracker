#ifndef NULL_USB_API_H
#define NULL_USB_API_H

#include "USBApiInterface.h"
#include "USBDeviceRequest.h"

class NullUSBApi : public IUSBApi
{
public:
	NullUSBApi();

	eUSBApiType getRuntimeUSBApiType() const override { return _USBApiType_NullUSB; }
	static eUSBApiType getStaticUSBApiType() { return _USBApiType_NullUSB; }

	bool startup() override;
	void poll() override;
	void shutdown() override;

	USBDeviceEnumerator* device_enumerator_create() override;
	bool device_enumerator_get_filter(const USBDeviceEnumerator* enumerator, struct USBDeviceFilter *outDeviceInfo) const override;
	bool device_enumerator_get_path(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const override;
	bool device_enumerator_get_unique_identifier(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const override;
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
};

class NullUSBBulkTransferBundle : public IUSBBulkTransferBundle
{
public:
    NullUSBBulkTransferBundle(
        const USBDeviceState *device_state,
		const struct USBRequestPayload_BulkTransferBundle *request);

    // Interface
    bool initialize() override;
    bool startTransfers() override;
    void cancelTransfers() override;

    // Accessors
	const USBRequestPayload_BulkTransferBundle &getTransferRequest() const override;
	t_usb_device_handle getUSBDeviceHandle() const override;
	int getActiveTransferCount() const override;

private:
    USBRequestPayload_BulkTransferBundle m_request;
};

#endif // USB_API_INTERFACE_H

