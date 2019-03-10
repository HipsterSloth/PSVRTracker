//-- includes -----
#include "LibUSBTransferBundle.h"
#include "LibUSBApi.h"
#include "Logger.h"
#include "Utility.h"
#include "USBDeviceRequest.h"
#include <assert.h>
#include <memory>
#include <cstring>

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
    #pragma warning (disable: 4200) // nonstandard extension used: zero-sized array in struct/union (from libusb)
#endif
#include "libusb.h"

//-- private methods -----
static void LIBUSB_CALL transfer_callback_function(struct libusb_transfer *bulk_transfer);

//-- implementation -----
LibUSBTransferBundle::LibUSBTransferBundle(
	const USBDeviceState *state,
	const USBRequestPayload_TransferBundle *request)
    : IUSBTransferBundle(state, request)
	, m_request(*request)
    , m_device(static_cast<const LibUSBDeviceState *>(state)->device)
    , m_device_handle(static_cast<const LibUSBDeviceState *>(state)->device_handle)
    , m_interface_index(static_cast<const LibUSBDeviceState *>(state)->claimed_interface_index)
    , m_active_transfer_count(0)
    , m_is_canceled(false)
    , transfer_buffer(nullptr)
{
	
}

LibUSBTransferBundle::~LibUSBTransferBundle()
{
    dispose();

    if (m_active_transfer_count > 0)
    {
        PSVR_MT_LOG_INFO("USBBulkTransferBundle::destructor") << "active transfer count non-zero!";
    }
}

bool LibUSBTransferBundle::initialize()
{
    bool bSuccess = (m_active_transfer_count == 0);
    uint8_t endpoint_address = 0;

    // Find the bulk transfer endpoint          
    if (bSuccess)
    {
        if (find_transfer_endpoint(m_request.transfer_type, m_device, m_interface_index, endpoint_address))
        {
            libusb_clear_halt(m_device_handle, endpoint_address);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Allocate the transfer buffer
    if (bSuccess)
    {
        transfer_requests.resize(m_request.in_flight_transfer_packet_count);
		std::fill(transfer_requests.begin(), transfer_requests.end(), nullptr);

        // Allocate the transfer buffer that the requests write data into
        size_t xfer_buffer_size = m_request.in_flight_transfer_packet_count * m_request.transfer_packet_size;
        transfer_buffer = new uint8_t[xfer_buffer_size];

        if (transfer_buffer != nullptr)
        {
            memset(transfer_buffer, 0, xfer_buffer_size);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Allocate and initialize the transfers
    if (bSuccess)
    {
        for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
        {
            transfer_requests[transfer_index] = libusb_alloc_transfer(0);

            if (transfer_requests[transfer_index] != nullptr)
            {
                if (m_request.transfer_type == _USBTransferBundleType_Bulk)
                {
                    libusb_fill_bulk_transfer(
                        transfer_requests[transfer_index],
                        m_device_handle,
                        endpoint_address,
                        transfer_buffer + transfer_index*m_request.transfer_packet_size,
                        m_request.transfer_packet_size,
                        transfer_callback_function,
                        reinterpret_cast<void*>(this),
                        0);
                }
                else if (m_request.transfer_type == _USBTransferBundleType_Interrupt)
                {
                    libusb_fill_interrupt_transfer(
	                    transfer_requests[transfer_index], 
                        m_device_handle,
	                    endpoint_address, 
                        transfer_buffer + transfer_index*m_request.transfer_packet_size, 
                        m_request.transfer_packet_size,
	                    transfer_callback_function, 
                        reinterpret_cast<void*>(this),
                        0);
                }
                else
                {
                    bSuccess = false;
                }
            }
            else
            {
                bSuccess = false;
            }
        }
    }

    return bSuccess;
}

void LibUSBTransferBundle::dispose()
{
    assert(m_active_transfer_count == 0);

    for (int transfer_index = 0;
        transfer_index < transfer_requests.size();
        ++transfer_index)
    {
        if (transfer_requests[transfer_index] != nullptr)
        {
            libusb_free_transfer(transfer_requests[transfer_index]);
        }
    }

    if (transfer_buffer != nullptr)
    {
        delete[] transfer_buffer;
        transfer_buffer = nullptr;
    }

	transfer_requests.clear();
}

bool LibUSBTransferBundle::startTransfers()
{
    bool bSuccess = (m_active_transfer_count == 0 && !m_is_canceled);

    // Start the transfers
    if (bSuccess)
    {
        for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
        {
            libusb_transfer *transfer = transfer_requests[transfer_index];

            if (libusb_submit_transfer(transfer) == 0)
            {
                ++m_active_transfer_count;
            }
            else
            {
                bSuccess = false;
                break;
            }
        }
    }

    return bSuccess;
}

void LibUSBTransferBundle::notifyActiveTransfersDecremented()
{
    assert(m_active_transfer_count > 0);
    --m_active_transfer_count;
}

static void LIBUSB_CALL transfer_callback_function(struct libusb_transfer *bulk_transfer)
{
    LibUSBTransferBundle *bundle = reinterpret_cast<LibUSBTransferBundle*>(bulk_transfer->user_data);
    const auto &request = bundle->getTransferRequest();
    enum libusb_transfer_status status = bulk_transfer->status;

    if (status == LIBUSB_TRANSFER_COMPLETED)
    {
        // NOTE: This callback is getting executed on the worker thread!
        // It should not:
        // 1) Do any expensive work
        // 2) Call any blocking functions
        // 3) Access data on the main thread, unless it can do so in an atomic way
        request.on_data_callback(
            bulk_transfer->buffer,
            bulk_transfer->actual_length,
            request.transfer_callback_userdata);
    }

    // See if the request wants to resubmitted the moment it completes.
    // If the transfer was canceled, this overrides the auto-resubmit.
    bool bRestartedTransfer = false;
    if (status != LIBUSB_TRANSFER_CANCELLED && request.bAutoResubmit)
    {
        // Start the transfer over with the same properties
        if (libusb_submit_transfer(bulk_transfer) == 0)
        {
            bRestartedTransfer = true;
        }
    }

    // If the transfer didn't restart update the active transfer count
    if (!bRestartedTransfer)
    {
        bundle->notifyActiveTransfersDecremented();
    }
}

void LibUSBTransferBundle::cancelTransfers()
{
    if (!m_is_canceled)
    {
        for (int transfer_index = 0;
            transfer_index < m_request.in_flight_transfer_packet_count;
            ++transfer_index)
        {
            libusb_transfer* bulk_transfer = transfer_requests[transfer_index];

            assert(bulk_transfer != nullptr);
            libusb_cancel_transfer(bulk_transfer);
        }

        m_is_canceled = true;
    }
}

// Search for an input transfer endpoint in the endpoint descriptor
// of the device interfaces alt_settings
bool LibUSBTransferBundle::find_transfer_endpoint(
    eUSBTransferBundleType bundle_type, 
    struct libusb_device *device, 
    int interface_index,
    uint8_t &out_endpoint_addr)
{
    bool bSuccess = false;
    libusb_config_descriptor *config = nullptr;

    libusb_get_active_config_descriptor(device, &config);

    if (config != nullptr)
    {
        const libusb_interface_descriptor *altsetting = nullptr;

        for (int i = 0; i < config->bNumInterfaces; i++)
        {
            const libusb_interface_descriptor *test_altsetting = config->interface[i].altsetting;

            if (test_altsetting[0].bInterfaceNumber == interface_index)
            {
                altsetting = test_altsetting;
                break;
            }
        }

        if (altsetting != nullptr)
        {
            for (int i = 0; i < altsetting->bNumEndpoints; i++)
            {
                const libusb_endpoint_descriptor *endpoint_desc = &altsetting->endpoint[i];

                if (endpoint_desc->wMaxPacketSize != 0)
                {
                    if (bundle_type == _USBTransferBundleType_Bulk &&
                        (endpoint_desc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK)
                    {
                        out_endpoint_addr = endpoint_desc->bEndpointAddress;
                        bSuccess = true;
                        break;
                    }
                    else if (bundle_type == _USBTransferBundleType_Interrupt &&
                        (endpoint_desc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_INTERRUPT)
                    {
                        out_endpoint_addr = endpoint_desc->bEndpointAddress;
                        bSuccess = true;
                        break;
                    }

                }
            }
        }

        libusb_free_config_descriptor(config);
    }

    return bSuccess;
}

// Accessors
const USBRequestPayload_TransferBundle &LibUSBTransferBundle::getTransferRequest() const
{
	return m_request;
}

t_usb_device_handle LibUSBTransferBundle::getUSBDeviceHandle() const
{
	return m_request.usb_device_handle;
}

int LibUSBTransferBundle::getActiveTransferCount() const
{
	return m_active_transfer_count;
}