//-- includes -----
#include "NullUSBApi.h"
#include "USBDeviceFilter.h"
#include "USBDeviceRequest.h"
#include "USBDeviceManager.h"

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

//-- public interface -----

//-- NullUSBApi -----
NullUSBApi::NullUSBApi() : IUSBApi()
{
}

bool NullUSBApi::startup()
{
	return true;
}

void NullUSBApi::poll()
{
}

void NullUSBApi::shutdown()
{
}

USBDeviceEnumerator* NullUSBApi::device_enumerator_create()
{
	USBDeviceEnumerator *nullusb_enumerator = new USBDeviceEnumerator;
	memset(nullusb_enumerator, 0, sizeof(USBDeviceEnumerator));
    nullusb_enumerator->api_type= _USBApiType_WinUSB;

	return nullusb_enumerator;
}

bool NullUSBApi::device_enumerator_is_valid(USBDeviceEnumerator* enumerator)
{
	return false;
}

bool NullUSBApi::device_enumerator_get_filter(const USBDeviceEnumerator* enumerator, USBDeviceFilter *outDeviceInfo) const
{
	return false;
}

bool NullUSBApi::device_enumerator_get_path(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const
{
	return false;
}

bool NullUSBApi::device_enumerator_get_unique_identifier(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const
{
	return false;
}

void NullUSBApi::device_enumerator_next(USBDeviceEnumerator* enumerator)
{
}

void NullUSBApi::device_enumerator_dispose(USBDeviceEnumerator* enumerator)
{
	delete enumerator;
}

USBDeviceState *NullUSBApi::open_usb_device(
    USBDeviceEnumerator* enumerator, 
    int interface_index,
    int configuration_index,
    bool reset_device)
{
	return nullptr;
}

void NullUSBApi::close_usb_device(USBDeviceState* device_state)
{
}

bool NullUSBApi::can_usb_device_be_opened(USBDeviceEnumerator* enumerator, char *outReason, size_t bufferSize)
{
	strncpy(outReason, "FAILED(Null USB API can't open devices)", bufferSize);
	return false;
}

eUSBResultCode NullUSBApi::submit_interrupt_transfer(
	const USBDeviceState* device_state,
	const USBTransferRequestState *requestState)
{
	return _USBResultCode_InvalidAPI;
}

eUSBResultCode NullUSBApi::submit_control_transfer(
	const USBDeviceState* device_state,
	const USBTransferRequestState *requestState)
{
	return _USBResultCode_InvalidAPI;
}

eUSBResultCode NullUSBApi::submit_bulk_transfer(
    const USBDeviceState* device_state,
    const struct USBTransferRequestState *requestState)
{
	return _USBResultCode_InvalidAPI;
}

IUSBBulkTransferBundle *NullUSBApi::allocate_bulk_transfer_bundle(const USBDeviceState *device_state, const USBRequestPayload_BulkTransferBundle *request)
{
	return new NullUSBBulkTransferBundle(device_state, request);
}

bool NullUSBApi::get_usb_device_filter(const USBDeviceState* device_state, struct USBDeviceFilter *outDeviceInfo) const
{
	return false;
}

bool NullUSBApi::get_usb_device_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const
{
	return false;
}

bool NullUSBApi::get_usb_device_port_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const
{
	return false;
}

//-- NullUSBBulkTransferBundle -----
NullUSBBulkTransferBundle::NullUSBBulkTransferBundle(
	const USBDeviceState *device_state,
	const struct USBRequestPayload_BulkTransferBundle *request)
	: IUSBBulkTransferBundle(device_state, request)
	, m_request(*request)
{
}

bool NullUSBBulkTransferBundle::initialize()
{
	return true;
}

bool NullUSBBulkTransferBundle::startTransfers()
{
	return true;
}

void NullUSBBulkTransferBundle::cancelTransfers()
{
}

const USBRequestPayload_BulkTransferBundle &NullUSBBulkTransferBundle::getTransferRequest() const
{
	return m_request;
}

t_usb_device_handle NullUSBBulkTransferBundle::getUSBDeviceHandle() const
{
	return m_request.usb_device_handle;
}

int NullUSBBulkTransferBundle::getActiveTransferCount() const
{
	return 0;
}