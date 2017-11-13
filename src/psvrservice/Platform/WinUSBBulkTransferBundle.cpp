//-- includes -----
#include "WinUSBBulkTransferBundle.h"
#include "WinUSBApi.h"
#include "Logger.h"
#include "Utility.h"
#include "USBDeviceRequest.h"

#include <assert.h>
#include <memory>
#include <cstring>

//-- implementation -----
WinUSBBulkTransferBundle::WinUSBBulkTransferBundle(
	const USBDeviceState *state,
	const USBRequestPayload_BulkTransferBundle *request)
    : IUSBBulkTransferBundle(state, request)
	, m_request(*request)
    //, m_device(static_cast<const LibUSBDeviceState *>(state)->device)
    //, m_device_handle(static_cast<const LibUSBDeviceState *>(state)->device_handle)
    , m_active_transfer_count(0)
    , m_is_canceled(false)
    //, bulk_transfer_requests(nullptr)
    , transfer_buffer(nullptr)
{
	
}

WinUSBBulkTransferBundle::~WinUSBBulkTransferBundle()
{
    dispose();

    if (m_active_transfer_count > 0)
    {
        PSVR_MT_LOG_INFO("USBBulkTransferBundle::destructor") << "active transfer count non-zero!";
    }
}

bool WinUSBBulkTransferBundle::initialize()
{
    bool bSuccess = (m_active_transfer_count == 0);
    uint8_t bulk_endpoint = 0;

#if 0
    // Find the bulk transfer endpoint          
    if (bSuccess)
    {
        if (find_bulk_transfer_endpoint(m_device, bulk_endpoint))
        {
            libusb_clear_halt(m_device_handle, bulk_endpoint);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Allocate the libusb transfer request array
    if (bSuccess)
    {
        size_t xfer_array_byte_size = m_request.in_flight_transfer_packet_count * sizeof(libusb_transfer *);
        bulk_transfer_requests = (libusb_transfer **)malloc(xfer_array_byte_size);

        if (bulk_transfer_requests != nullptr)
        {
            memset(bulk_transfer_requests, 0, xfer_array_byte_size);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Allocate the transfer buffer
    if (bSuccess)
    {
        // Allocate the transfer buffer that the requests write data into
        size_t xfer_buffer_size = m_request.in_flight_transfer_packet_count * m_request.transfer_packet_size;
        transfer_buffer = (uint8_t *)malloc(xfer_buffer_size);

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
            bulk_transfer_requests[transfer_index] = libusb_alloc_transfer(0);

            if (bulk_transfer_requests[transfer_index] != nullptr)
            {
                libusb_fill_bulk_transfer(
                    bulk_transfer_requests[transfer_index],
                    m_device_handle,
                    bulk_endpoint,
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
    }
#endif

    return bSuccess;
}

void WinUSBBulkTransferBundle::dispose()
{
    assert(m_active_transfer_count == 0);

#if 0
    for (int transfer_index = 0;
        transfer_index < m_request.in_flight_transfer_packet_count;
        ++transfer_index)
    {
        if (bulk_transfer_requests[transfer_index] != nullptr)
        {
            libusb_free_transfer(bulk_transfer_requests[transfer_index]);
        }
    }

    if (transfer_buffer != nullptr)
    {
        free(transfer_buffer);
        transfer_buffer = nullptr;
    }

    if (bulk_transfer_requests != nullptr)
    {
        free(bulk_transfer_requests);
        bulk_transfer_requests = nullptr;
    }
#endif
}

bool WinUSBBulkTransferBundle::startTransfers()
{
    bool bSuccess = (m_active_transfer_count == 0 && !m_is_canceled);

#if 0
    // Start the transfers
    if (bSuccess)
    {
        for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
        {
            libusb_transfer *bulk_transfer = bulk_transfer_requests[transfer_index];

            if (libusb_submit_transfer(bulk_transfer) == 0)
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
#endif

    return bSuccess;
}

void WinUSBBulkTransferBundle::notifyActiveTransfersDecremented()
{
    assert(m_active_transfer_count > 0);
    --m_active_transfer_count;
}

#if 0
static void LIBUSB_CALL transfer_callback_function(struct libusb_transfer *bulk_transfer)
{
    WinUSBBulkTransferBundle *bundle = reinterpret_cast<WinUSBBulkTransferBundle*>(bulk_transfer->user_data);
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
#endif

void WinUSBBulkTransferBundle::cancelTransfers()
{
    //assert(bulk_transfer_requests != nullptr);

    if (!m_is_canceled)
    {
#if 0
        for (int transfer_index = 0;
            transfer_index < m_request.in_flight_transfer_packet_count;
            ++transfer_index)
        {
            libusb_transfer* bulk_transfer = bulk_transfer_requests[transfer_index];

            assert(bulk_transfer != nullptr);
            libusb_cancel_transfer(bulk_transfer);
        }
#endif

        m_is_canceled = true;
    }
}

// Accessors
const USBRequestPayload_BulkTransferBundle &WinUSBBulkTransferBundle::getTransferRequest() const
{
	return m_request;
}

t_usb_device_handle WinUSBBulkTransferBundle::getUSBDeviceHandle() const
{
	return m_request.usb_device_handle;
}

int WinUSBBulkTransferBundle::getActiveTransferCount() const
{
	return m_active_transfer_count;
}