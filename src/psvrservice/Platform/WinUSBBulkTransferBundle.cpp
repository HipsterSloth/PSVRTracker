//-- includes -----
#include "WinUSBBulkTransferBundle.h"
#include "WinUSBApi.h"
#include "Logger.h"
#include "Utility.h"
#include "USBDeviceRequest.h"

#include <assert.h>
#include <memory>
#include <cstring>

#define ANSI
#define WIN32_LEAN_AND_MEAN

#include <windows.h>  // Required for data types
#include <winuser.h>
#include <Dbt.h>
#include <guiddef.h>
#include <cfgmgr32.h>   // for MAX_DEVICE_ID_LEN and CM_Get_Device_ID
#include <setupapi.h> // Device setup APIs
#include <Devpkey.h>
#include <strsafe.h>
#include <winusb.h>
#include <usb100.h>

#pragma comment(lib, "WinUsb.lib")

#ifdef _MSC_VER
#pragma warning(disable:4996) // disable warnings about strncpy
#endif

static void transfer_callback_function(
	struct WinUSBAsyncBulkTransfer *bulk_transfer,
	eWinusbBulkTransferStatus status,
	unsigned char *transfer_buffer,
	size_t transferred_length,
	void *user_data);

//-- implementation -----
WinUSBBulkTransferBundle::WinUSBBulkTransferBundle(
	const USBDeviceState *state,
	const USBRequestPayload_BulkTransferBundle *request)
    : IUSBBulkTransferBundle(state, request)
	, m_request(*request)
    , m_deviceHandle(static_cast<const WinUSBDeviceState *>(state)->device_handle)
    , m_bulkInPipe(static_cast<const WinUSBDeviceState *>(state)->bulk_in_pipe)
	, m_bulkInPipePacketSize(static_cast<const WinUSBDeviceState *>(state)->bulk_in_pipe_packet_size)
    , m_active_transfer_count(0)
    , m_is_canceled(false)
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

    // Turn on raw I/O, because without it the transfers will not be efficient
    UCHAR raw_io = 1;
    BOOL success = WinUsb_SetPipePolicy(m_deviceHandle, m_bulkInPipe, RAW_IO, sizeof(raw_io), &raw_io);
    if (!success)
    {
        PSVR_MT_LOG_INFO("inUSBBulkTransferBundle::initialize()") << "Failed to enable raw I/O for bulk_in pipe.";
		return false;
    }
    // Allocate the transfer buffer
    if (bSuccess)
    {
        bulk_transfer_requests.resize(m_request.in_flight_transfer_packet_count);
		std::fill(bulk_transfer_requests.begin(), bulk_transfer_requests.end(), nullptr);

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
		WinUSBApi *winusb_api= WinUSBApi::getInterface();

        for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
        {
            bulk_transfer_requests[transfer_index] = winusb_api->winusbAllocateAsyncBulkTransfer();

            if (bulk_transfer_requests[transfer_index] != nullptr)
            {
				winusb_api->winusbSetupAsyncBulkTransfer(
					m_deviceHandle, 
					m_bulkInPipe, 
					transfer_buffer + transfer_index*m_request.transfer_packet_size,
					m_request.transfer_packet_size, 
					transfer_callback_function, 
					reinterpret_cast<void*>(this),
					bulk_transfer_requests[transfer_index]);
            }
            else
            {
                bSuccess = false;
            }
        }
    }

    return bSuccess;
}

void WinUSBBulkTransferBundle::dispose()
{
    assert(m_active_transfer_count == 0);

    for (int transfer_index = 0;
        transfer_index < m_request.in_flight_transfer_packet_count;
        ++transfer_index)
    {
        if (bulk_transfer_requests[transfer_index] != nullptr)
        {
            WinUSBApi::getInterface()->winusbFreeAsyncBulkTransfer(bulk_transfer_requests[transfer_index]);
        }
    }

    if (transfer_buffer != nullptr)
    {
        delete[] transfer_buffer;
        transfer_buffer = nullptr;
    }

	bulk_transfer_requests.clear();
}

bool WinUSBBulkTransferBundle::startTransfers()
{
    bool bSuccess = (m_active_transfer_count == 0 && !m_is_canceled);

    // Start the transfers
    if (bSuccess)
    {
        for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
        {
			struct WinUSBAsyncBulkTransfer *bulk_transfer = bulk_transfer_requests[transfer_index];

            if (WinUSBApi::getInterface()->winusbSubmitAsyncBulkTransfer(bulk_transfer) == 0)
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

void WinUSBBulkTransferBundle::notifyActiveTransfersDecremented()
{
    assert(m_active_transfer_count > 0);
    --m_active_transfer_count;
}

static void transfer_callback_function(
	struct WinUSBAsyncBulkTransfer *bulk_transfer,
	eWinusbBulkTransferStatus status,
	unsigned char *transfer_buffer,
	size_t transferred_length,
	void *user_data)
{
    WinUSBBulkTransferBundle *bundle = reinterpret_cast<WinUSBBulkTransferBundle*>(user_data);
    const auto &request = bundle->getTransferRequest();

    if (status == WINUSB_TRANSFER_COMPLETED)
    {
        // NOTE: This callback is getting executed on the worker thread!
        // It should not:
        // 1) Do any expensive work
        // 2) Call any blocking functions
        // 3) Access data on the main thread, unless it can do so in an atomic way
        request.on_data_callback(
            transfer_buffer,
            (int)transferred_length,
            request.transfer_callback_userdata);
    }

    // See if the request wants to resubmitted the moment it completes.
    // If the transfer was canceled, this overrides the auto-resubmit.
    bool bRestartedTransfer = false;
    if (status != WINUSB_TRANSFER_CANCELLED && request.bAutoResubmit)
    {
        // Start the transfer over with the same properties
        if (WinUSBApi::getInterface()->winusbSubmitAsyncBulkTransfer(bulk_transfer))
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

void WinUSBBulkTransferBundle::cancelTransfers()
{
    //assert(bulk_transfer_requests != nullptr);

    if (!m_is_canceled)
    {
        for (int transfer_index = 0;
            transfer_index < m_request.in_flight_transfer_packet_count;
            ++transfer_index)
        {
            struct WinUSBAsyncBulkTransfer *bulk_transfer = bulk_transfer_requests[transfer_index];

            assert(bulk_transfer != nullptr);
            WinUSBApi::getInterface()->winusbCancelAsyncBulkTransfer(bulk_transfer);
        }

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