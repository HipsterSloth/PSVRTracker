//-- includes -----
#include "Logger.h"
#include "USBDeviceInfo.h"
#include "USBDeviceRequest.h"
#include "USBDeviceManager.h"
#include "WinUSBApi.h"
#include "WinUSBBulkTransferBundle.h"

#define ANSI
#define WIN32_LEAN_AND_MEAN

#include <windows.h>  // Required for data types
#include <winuser.h>
#include <Dbt.h>
#include <guiddef.h>
#include <setupapi.h> // Device setup APIs
#include <strsafe.h>
#include <winusb.h>
#include <usb100.h>

#pragma comment(lib, "WinUsb.lib")

#ifdef _MSC_VER
#pragma warning(disable:4996) // disable warnings about strncpy
#endif

// https://msdn.microsoft.com/en-us/library/windows/hardware/ff540174(v=vs.85).aspx

//-- constants ----
GUID WINUSB_GUID_DEVCLASS_IMAGE = { 0x6bdd1fc6, 0x810f, 0x11d0, 0xbe, 0xc7, 0x08, 0x00, 0x2b, 0xe2, 0x09, 0x2f };
GUID WINUSB_GUID_DEVCLASS_HID = { 0x4d1e55b2, 0xf16f, 0x11cf, 0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30 };
GUID WINUSB_GUID_DEVCLASS_USB_RAW = { 0xa5dcbf10, 0x6530, 0x11d2, 0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed };

//-- public interface -----

//-- definitions -----
struct WinUSBDeviceEnumerator : USBDeviceEnumerator
{
public:
	WinUSBDeviceEnumerator(const GUID &deviceClassGUID)
		: m_DeviceClassGUID(deviceClassGUID)
		, m_DeviceInfoSetHandle(INVALID_HANDLE_VALUE)
        , m_DetailData(NULL)
        , m_DevicePath("")
        , m_USBUniqueID("")
		, m_EnumerationIndex(-1)
        , m_ProductId(-1)
        , m_VendorId(-1)
        , m_CompositeInterfaceIndex(-1)
        , m_bIsComposite(false)
		, m_bNoMoreItems(false)
	{
		m_DeviceInfoSetHandle = SetupDiGetClassDevs((LPGUID)&m_DeviceClassGUID, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
		m_InterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

		if (isValid())
		{
			next();
		}
	}

	virtual ~WinUSBDeviceEnumerator()
	{
        if (m_DetailData != NULL)
        {
            LocalFree(m_DetailData);
            m_DetailData= NULL;
        }

		if (m_DeviceInfoSetHandle != INVALID_HANDLE_VALUE)
		{
			SetupDiDestroyDeviceInfoList(m_DeviceInfoSetHandle);
		}
	}

	bool isValid() const
	{
		return m_DeviceInfoSetHandle != INVALID_HANDLE_VALUE && !m_bNoMoreItems;
	}

	void next()
	{
        bool bFoundValidEntry= false;

		while (!bFoundValidEntry && isValid())
		{
			++m_EnumerationIndex;
            m_DevicePath= "";

            if (m_DetailData != NULL)
            {
                LocalFree(m_DetailData);
                m_DetailData= NULL;
            }

            if (SetupDiEnumDeviceInterfaces(m_DeviceInfoSetHandle, NULL, &m_DeviceClassGUID, 0, &m_InterfaceData) == TRUE)
            {
                ULONG length;
                ULONG requiredLength=0; 

                SetupDiGetDeviceInterfaceDetail(m_DeviceInfoSetHandle, &m_InterfaceData, NULL, 0, &requiredLength, NULL);
                m_DetailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA)LocalAlloc(LMEM_FIXED, requiredLength);

                if (m_DetailData != NULL)
                {
                    m_DetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
                    length = requiredLength;

                    if (SetupDiGetDeviceInterfaceDetail(m_DeviceInfoSetHandle, &m_InterfaceData, m_DetailData, length, &requiredLength, NULL) == TRUE)
                    {
                        char raw_device_path[512];
                        #ifdef UNICODE
                        wcstombs(raw_device_path, m_DetailData->DevicePath, (unsigned)_countof(raw_device_path));  
                        #else
                        strncpy_s(raw_device_path, m_DetailData->DevicePath, (unsigned)_countof(raw_device_path));
                        #endif

                        // Save the raw multi-byte device path
                        m_DevicePath= raw_device_path;

                        // Non-composite Device Path: "USB\VID_0B05&PID_17CB\5CF3707F7940"
                        // Composite Device Path: "USB\VID_1415&PID_2000&MI_01\8&2DF96CB8&0&0001"
                        char unique_id[256];
                        if (sscanf_s(m_DetailData->DevicePath, "USB\\VID_%X&PID_%X\\%255s",
                                &m_VendorId, &m_ProductId, &unique_id, (unsigned)_countof(unique_id)) == 3)
                        {
                            m_USBUniqueID= unique_id;
                            m_CompositeInterfaceIndex= -1;
                            m_bIsComposite= false;
                            bFoundValidEntry= true;
                        }
                        else
                        {
                            if (sscanf_s(m_DetailData->DevicePath, "USB\\VID_%X&PID_%X&MI_%X\\%255s", 
                                    &m_VendorId, &m_ProductId, &m_CompositeInterfaceIndex, &unique_id, (unsigned)_countof(unique_id)) == 4)
                            {
                                // Replace all instances if '&' with '_'
                                for (char* p = unique_id; p = strchr(p, '&'); ++p) {
                                    *p = '_';
                                }

                                m_USBUniqueID= unique_id;
                                m_bIsComposite= true;
                                bFoundValidEntry= true;
                            }
                        }
                    }
                }
            }
            else
			{
				m_bNoMoreItems = true;
			}
		}
	}

	inline HDEVINFO getDeviceInfoSetHandle() const
	{
		return m_DeviceInfoSetHandle;
	}

	inline const SP_DEVICE_INTERFACE_DATA &getDeviceInterfaceData() const
	{
		return m_InterfaceData;
	}

	inline const PSP_DEVICE_INTERFACE_DETAIL_DATA getDeviceInterfaceDetailData() const
	{
		return m_DetailData;
	}

    inline std::string getDevicePath() const
    {
        return m_DevicePath;
    }

    inline int getProductId() const
    {
        return m_ProductId;
    }

    inline int getVendorId() const
    {
        return m_VendorId;
    }

    inline int getCompositeInterfaceIndex() const
    {
        return m_bIsComposite ? m_CompositeInterfaceIndex : -1;
    }

    inline std::string getUniqueID() const
    {
        return m_USBUniqueID;
    }

private:
	const GUID &m_DeviceClassGUID;
	HDEVINFO m_DeviceInfoSetHandle;
    SP_DEVICE_INTERFACE_DATA m_InterfaceData;
    PSP_DEVICE_INTERFACE_DETAIL_DATA m_DetailData; 
    std::string m_DevicePath;
    std::string m_USBUniqueID;
    int m_EnumerationIndex;
    int m_ProductId;
    int m_VendorId;
    int m_CompositeInterfaceIndex;
    bool m_bIsComposite;
	bool m_bNoMoreItems;
};

//-- private methods -----
static std::string GetLastErrorAsString();

//-- WinUSBApi -----
WinUSBApi::WinUSBApi() : IUSBApi()
{
}

bool WinUSBApi::startup()
{
	return true;
}

void WinUSBApi::poll()
{
}

void WinUSBApi::shutdown()
{
}

// This function is copied from PlatformDeviceAPIWin32.cpp to avoid dependency to 
// the DeviceManager class in the controller test programs
static const GUID * get_device_class_platform_identifier(const DeviceClass deviceClass)
{
    const GUID *result= nullptr;

	switch (deviceClass)
	{
	case DeviceClass::DeviceClass_Camera:
		result = &WINUSB_GUID_DEVCLASS_IMAGE;
		break;
	case DeviceClass::DeviceClass_HID:
		result = &WINUSB_GUID_DEVCLASS_HID;
		break;
    case DeviceClass::DeviceClass_RawUSB:
        result = &WINUSB_GUID_DEVCLASS_USB_RAW;
	default:
		assert(0 && "Unhandled device class type");
	}

    return result;
}

USBDeviceEnumerator* WinUSBApi::device_enumerator_create(const DeviceClass deviceClass)
{
    const GUID *deviceClassGUID= get_device_class_platform_identifier(deviceClass);
    WinUSBDeviceEnumerator *enumerator= nullptr;

    if (deviceClassGUID != nullptr)
    {
        enumerator= new WinUSBDeviceEnumerator(*deviceClassGUID);
    }

    return enumerator;
}

bool WinUSBApi::device_enumerator_is_valid(USBDeviceEnumerator* enumerator)
{
	WinUSBDeviceEnumerator *winusb_enumerator = static_cast<WinUSBDeviceEnumerator *>(enumerator);

	return winusb_enumerator != nullptr && winusb_enumerator->isValid();
}

bool WinUSBApi::device_enumerator_get_filter(const USBDeviceEnumerator* enumerator, USBDeviceFilter *outDeviceInfo) const
{
	const WinUSBDeviceEnumerator *winusb_enumerator = static_cast<const WinUSBDeviceEnumerator *>(enumerator);
	bool bSuccess = false;

	if (winusb_enumerator != nullptr)
	{
		outDeviceInfo->product_id = winusb_enumerator->getProductId();
		outDeviceInfo->vendor_id = winusb_enumerator->getVendorId();

        int interface_index= winusb_enumerator->getCompositeInterfaceIndex();
        if (interface_index >= 0 && interface_index < 32)
        {
            outDeviceInfo->interface_mask= (1 << interface_index);
        }
        else
        {
            outDeviceInfo->interface_mask= 0;
        }
		bSuccess = true;
	}

	return bSuccess;
}

bool WinUSBApi::device_enumerator_get_path(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const
{
	const WinUSBDeviceEnumerator *winusb_enumerator = static_cast<const WinUSBDeviceEnumerator *>(enumerator);
	bool bSuccess = false;

	if (winusb_enumerator != nullptr)
	{
        std::string devicePath= winusb_enumerator->getDevicePath();

		bSuccess = SUCCEEDED(StringCchCopy(outBuffer, bufferSize, devicePath.c_str()));
	}

	return bSuccess;
}

void WinUSBApi::device_enumerator_next(USBDeviceEnumerator* enumerator)
{
	WinUSBDeviceEnumerator *winusb_enumerator = static_cast<WinUSBDeviceEnumerator *>(enumerator);

	if (device_enumerator_is_valid(winusb_enumerator))
	{
		winusb_enumerator->next();
	}
}

void WinUSBApi::device_enumerator_dispose(USBDeviceEnumerator* enumerator)
{
    if (enumerator != nullptr)
    {
        delete enumerator;
    }
}

USBDeviceState *WinUSBApi::open_usb_device(USBDeviceEnumerator* enumerator, int interface_index)
{
    WinUSBDeviceEnumerator *winusb_enumerator = static_cast<WinUSBDeviceEnumerator *>(enumerator);
	WinUSBDeviceState *winusb_device_state = nullptr;

	if (device_enumerator_is_valid(enumerator) && 
        (winusb_enumerator->getCompositeInterfaceIndex() == interface_index ||
         winusb_enumerator->getCompositeInterfaceIndex() == -1))
	{
		bool bOpened = false;
		
		winusb_device_state = new WinUSBDeviceState;
		winusb_device_state->clear();

        winusb_device_state->device_path= winusb_enumerator->getDevicePath();
        winusb_device_state->unique_id= winusb_enumerator->getUniqueID();
        winusb_device_state->vendor_id= winusb_enumerator->getVendorId();
        winusb_device_state->product_id= winusb_enumerator->getProductId();
        winusb_device_state->composite_interface_index= winusb_enumerator->getCompositeInterfaceIndex();

        winusb_device_state->device_handle = CreateFile(
            winusb_device_state->device_path.c_str(),
            GENERIC_WRITE | GENERIC_READ,
            FILE_SHARE_WRITE | FILE_SHARE_READ,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
            NULL);

        if (winusb_device_state->device_handle != INVALID_HANDLE_VALUE)
        {
            if (WinUsb_Initialize(winusb_device_state->device_handle, &winusb_device_state->interface_handle) == TRUE)
            {
                ULONG length = sizeof(UCHAR);

                if (WinUsb_QueryDeviceInformation(
                        winusb_device_state->interface_handle,
                        DEVICE_SPEED, 
                        &length, 
                        &winusb_device_state->device_speed) == TRUE)
                {
                    USB_INTERFACE_DESCRIPTOR interface_descriptor;

                    memset(&interface_descriptor, 0, sizeof(USB_INTERFACE_DESCRIPTOR));

                    if (WinUsb_QueryInterfaceSettings(
                            winusb_device_state->interface_handle,
                            0, 
                            &interface_descriptor) == TRUE)
                    {
                        bOpened= interface_descriptor.bNumEndpoints > 0;

                        for(int i=0; i < interface_descriptor.bNumEndpoints; i++)
                        {
                            WINUSB_PIPE_INFORMATION pipeInfo;

                            if (WinUsb_QueryPipe(winusb_device_state->interface_handle, 0, (UCHAR)i, &pipeInfo) == TRUE)
                            {
                                if (USB_ENDPOINT_DIRECTION_IN(pipeInfo.PipeId))
                                {
                                    switch (pipeInfo.PipeType)
                                    {
                                    case UsbdPipeTypeBulk:
                                        winusb_device_state->bulk_in_pipe = pipeInfo.PipeId;
                                        break;
                                    case UsbdPipeTypeInterrupt:
                                        winusb_device_state->interrupt_in_pipe = pipeInfo.PipeId;
                                        break;
                                    case UsbdPipeTypeIsochronous:
                                        winusb_device_state->isochronous_in_pipe = pipeInfo.PipeId;
                                        break;
                                    }
                                }
                                else if (USB_ENDPOINT_DIRECTION_OUT(pipeInfo.PipeId))
                                {
                                    switch (pipeInfo.PipeType)
                                    {
                                    case UsbdPipeTypeBulk:
                                        winusb_device_state->bulk_out_pipe = pipeInfo.PipeId;
                                        break;
                                    case UsbdPipeTypeInterrupt:
                                        winusb_device_state->interrupt_out_pipe = pipeInfo.PipeId;
                                        break;
                                    case UsbdPipeTypeIsochronous:
                                        winusb_device_state->isochronous_out_pipe = pipeInfo.PipeId;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

		if (!bOpened)
		{
			close_usb_device(winusb_device_state);
			winusb_device_state= nullptr;
		}
    }

	return nullptr;
}

void WinUSBApi::close_usb_device(USBDeviceState* device_state)
{
	if (device_state != nullptr)
	{
		WinUSBDeviceState *winusb_device_state = static_cast<WinUSBDeviceState *>(device_state);

		if (winusb_device_state->interface_handle != nullptr)
		{
			PSVR_LOG_INFO("WinUSBApi::close_usb_device") << "Close USB interface handle " << winusb_device_state->interface_handle;
            WinUsb_Free(winusb_device_state->interface_handle);            
		}

		if (winusb_device_state->device_handle != nullptr)
		{
            PSVR_LOG_INFO("WinUSBApi::close_usb_device") << "Close USB device handle " << winusb_device_state->device_handle;
            CloseHandle(winusb_device_state->device_handle);
		}

		delete winusb_device_state;
	}
}

bool WinUSBApi::can_usb_device_be_opened(USBDeviceEnumerator* enumerator, char *outReason, size_t bufferSize)
{
	WinUSBDeviceEnumerator *winusb_enumerator = static_cast<WinUSBDeviceEnumerator *>(enumerator);
	bool bCanBeOpened= false;

	if (device_enumerator_is_valid(enumerator))
	{	
        std::string device_path= winusb_enumerator->getDevicePath();
        HANDLE device_handle = CreateFile(
            device_path.c_str(),
            GENERIC_WRITE | GENERIC_READ,
            FILE_SHARE_WRITE | FILE_SHARE_READ,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
            NULL);

		// Can be opened if we can open the device now or it's already opened
        if (device_handle != INVALID_HANDLE_VALUE)
        {
			strncpy(outReason, "SUCCESS(can be opened)", bufferSize);
			CloseHandle(device_handle);
            bCanBeOpened= true;
        }
        else
        {
            if (GetLastError() == ERROR_SHARING_VIOLATION)
            {
                strncpy(outReason, "SUCCESS(already opened)", bufferSize);
			    bCanBeOpened= true;
            }
            else
            {
                std::string error_string= GetLastErrorAsString();

			    strncpy(outReason, error_string.c_str(), bufferSize);
            }
        }
	}

	return bCanBeOpened;
}

eUSBResultCode WinUSBApi::submit_interrupt_transfer(
	const USBDeviceState* device_state,
	const USBTransferRequestState *requestState)
{
	const WinUSBDeviceState *winusb_device_state = static_cast<const WinUSBDeviceState *>(device_state);
    const USBRequestPayload_InterruptTransfer &request = requestState->request.payload.interrupt_transfer;

	USBTransferResult result;
	memset(&result, 0, sizeof(USBTransferResult));
	result.result_type = _USBResultType_InterruptTransfer;
	result.payload.interrupt_transfer.usb_device_handle = request.usb_device_handle;

    DWORD LastErrorCode= ERROR_SUCCESS;
    if (USB_ENDPOINT_DIRECTION_OUT(request.endpoint)) // Write
    {
        ULONG bytesWritten= 0;

        unsigned char data[MAX_INTERRUPT_TRANSFER_PAYLOAD];

        if (request.length > 0)
        {
            memcpy(data, request.data, request.length);
        }

        if (WinUsb_WritePipe(
                winusb_device_state->interface_handle,
                winusb_device_state->interrupt_out_pipe,
                data,
                request.length,
                &bytesWritten,
                NULL) == FALSE)
        {
            LastErrorCode= GetLastError();
        }
                 
        #if defined(DEBUG_USB)
		debug("USBMgr RESULT: interrupt transfer write - dev: %d, endpoint: 0x%X, length: %d -> %s\n",
			request.usb_device_handle,
			request.endpoint,
			request.length,
			LastErrorCode == ERROR_SUCCESS ? "SUCCESS" : "FAILED");
        #endif
    }
    else // Read
    {
        ULONG bytesRead;

        if (WinUsb_ReadPipe(
                winusb_device_state->interface_handle,
                winusb_device_state->interrupt_in_pipe,
                result.payload.interrupt_transfer.data,
                request.length,
                &bytesRead,
                NULL) == FALSE)
        {
            LastErrorCode= GetLastError();
        }

        #if defined(DEBUG_USB)
		debug("USBMgr RESULT: interrupt transfer read - dev: %d, endpoint: 0x%X, length: %d -> 0x%X (%s)\n",
			request.usb_device_handle,
			request.endpoint,
			request.length,
			LastErrorCode == ERROR_SUCCESS ? "SUCCESS" : "FAILED");
        #endif
    }

    switch (LastErrorCode)
    {
    case ERROR_SUCCESS:
        result.payload.interrupt_transfer.result_code = _USBResultCode_Completed;
        break;
    case ERROR_INVALID_HANDLE:
        result.payload.interrupt_transfer.result_code = _USBResultCode_BadHandle;
        break;
    case ERROR_NOT_ENOUGH_MEMORY:
        result.payload.interrupt_transfer.result_code = _USBResultCode_NoMemory;
        break;
    case ERROR_IO_PENDING:
        result.payload.interrupt_transfer.result_code = _USBResultCode_Pipe;
        break;
    case ERROR_SEM_TIMEOUT:
        result.payload.interrupt_transfer.result_code = _USBResultCode_TimedOut;
        break;
    default:
        result.payload.interrupt_transfer.result_code = _USBResultCode_GeneralError;
        break;
    }

	// Add the result to the outgoing result queue
	usb_device_post_transfer_result(result, requestState->callback);

    return result.payload.interrupt_transfer.result_code;
}

eUSBResultCode WinUSBApi::submit_control_transfer(
	const USBDeviceState* device_state,
	const USBTransferRequestState *requestState)
{
	const WinUSBDeviceState *winusb_device_state = static_cast<const WinUSBDeviceState *>(device_state);
    const USBRequestPayload_ControlTransfer &request = requestState->request.payload.control_transfer;

    WINUSB_SETUP_PACKET setupPacket;
    memset(&setupPacket, 0, sizeof(WINUSB_SETUP_PACKET));
    setupPacket.RequestType = request.bmRequestType;
    setupPacket.Request = request.bRequest;
    setupPacket.Index = request.wIndex;
    setupPacket.Length = request.wLength;
    setupPacket.Value = request.wValue;

	USBTransferResult result;
	memset(&result, 0, sizeof(USBTransferResult));
	result.result_type = _USBResultType_ControlTransfer;
	result.payload.control_transfer.usb_device_handle = request.usb_device_handle;

    DWORD LastErrorCode= ERROR_SUCCESS;
    if (USB_ENDPOINT_DIRECTION_OUT(request.bmRequestType)) // Write
    {
        ULONG bytesWritten= 0;
        unsigned char data[MAX_CONTROL_TRANSFER_PAYLOAD];

        if (request.wLength > 0)
        {
            memcpy(data, request.data, request.wLength);
        }

        if (WinUsb_ControlTransfer(
                winusb_device_state->interface_handle,
                setupPacket,
                data,
                request.wLength,
                &bytesWritten,
                NULL) == FALSE)
        {
            LastErrorCode= GetLastError();
        }
                 
    #if defined(DEBUG_USB)
		debug("USBMgr RESULT: control transfer write - dev: %d, reg: 0x%X, value: 0x%x -> %s\n",
			request.usb_device_handle,
			request.wIndex,
			request.data[0],
			LastErrorCode == ERROR_SUCCESS ? "SUCCESS" : "FAILED");
    #endif
    }
    else // Read
    {
        ULONG bytesRead= 0;

        if (WinUsb_ControlTransfer(
                winusb_device_state->interface_handle,
                setupPacket,
                result.payload.control_transfer.data,
                request.wLength,
                &bytesRead,
                NULL) == FALSE)
        {
            LastErrorCode= GetLastError();
        }

    #if defined(DEBUG_USB)
		debug("USBMgr RESULT: control transfer read - dev: %d, reg: 0x%X -> 0x%X (%s)\n",
			request.usb_device_handle,
			request.wIndex,
			request.data[0],
			LastErrorCode == ERROR_SUCCESS ? "SUCCESS" : "FAILED");
    #endif
    }

    switch (LastErrorCode)
    {
    case ERROR_SUCCESS:
        result.payload.control_transfer.result_code = _USBResultCode_Completed;
        break;
    case ERROR_INVALID_HANDLE:
        result.payload.control_transfer.result_code = _USBResultCode_BadHandle;
        break;
    case ERROR_NOT_ENOUGH_MEMORY:
        result.payload.control_transfer.result_code = _USBResultCode_NoMemory;
        break;
    default:
        result.payload.control_transfer.result_code = _USBResultCode_GeneralError;
        break;
    }

	// Add the result to the outgoing result queue
	usb_device_post_transfer_result(result, requestState->callback);

    return result.payload.control_transfer.result_code;
}

eUSBResultCode WinUSBApi::submit_bulk_transfer(
	const USBDeviceState* device_state,
	const USBTransferRequestState *requestState)
{
	const WinUSBDeviceState *winusb_device_state = static_cast<const WinUSBDeviceState *>(device_state);
    const USBRequestPayload_BulkTransfer &request = requestState->request.payload.bulk_transfer;

	USBTransferResult result;
	memset(&result, 0, sizeof(USBTransferResult));
	result.result_type = _USBResultType_BulkTransfer;
	result.payload.bulk_transfer.usb_device_handle = request.usb_device_handle;

    DWORD LastErrorCode= ERROR_SUCCESS;
    if (USB_ENDPOINT_DIRECTION_OUT(request.endpoint)) // Write
    {
        ULONG bytesWritten= 0;

        unsigned char data[MAX_BULK_TRANSFER_PAYLOAD];

        if (request.length > 0)
        {
            memcpy(data, request.data, request.length);
        }

        if (WinUsb_WritePipe(
                winusb_device_state->interface_handle,
                winusb_device_state->bulk_out_pipe,
                data,
                request.length,
                &bytesWritten,
                NULL) == FALSE)
        {
            LastErrorCode= GetLastError();
        }
                 
        #if defined(DEBUG_USB)
		debug("USBMgr RESULT: bulk transfer write - dev: %d, endpoint: 0x%X, length: %d -> %s\n",
			request.usb_device_handle,
			request.endpoint,
			request.length,
			LastErrorCode == ERROR_SUCCESS ? "SUCCESS" : "FAILED");
        #endif
    }
    else // Read
    {
        ULONG bytesRead;

        if (WinUsb_ReadPipe(
                winusb_device_state->interface_handle,
                winusb_device_state->bulk_in_pipe,
                result.payload.bulk_transfer.data,
                request.length,
                &bytesRead,
                NULL) == FALSE)
        {
            LastErrorCode= GetLastError();
        }

        #if defined(DEBUG_USB)
		debug("USBMgr RESULT: bulk transfer read - dev: %d, endpoint: 0x%X, length: %d -> 0x%X (%s)\n",
			request.usb_device_handle,
			request.endpoint,
			request.length,
			LastErrorCode == ERROR_SUCCESS ? "SUCCESS" : "FAILED");
        #endif
    }

    switch (LastErrorCode)
    {
    case ERROR_SUCCESS:
        result.payload.bulk_transfer.result_code = _USBResultCode_Completed;
        break;
    case ERROR_INVALID_HANDLE:
        result.payload.bulk_transfer.result_code = _USBResultCode_BadHandle;
        break;
    case ERROR_NOT_ENOUGH_MEMORY:
        result.payload.bulk_transfer.result_code = _USBResultCode_NoMemory;
        break;
    case ERROR_IO_PENDING:
        result.payload.bulk_transfer.result_code = _USBResultCode_Pipe;
        break;
    case ERROR_SEM_TIMEOUT:
        result.payload.bulk_transfer.result_code = _USBResultCode_TimedOut;
        break;
    default:
        result.payload.bulk_transfer.result_code = _USBResultCode_GeneralError;
        break;
    }

	// Add the result to the outgoing result queue
	usb_device_post_transfer_result(result, requestState->callback);

    return result.payload.bulk_transfer.result_code;
}

IUSBBulkTransferBundle *WinUSBApi::allocate_bulk_transfer_bundle(const USBDeviceState *device_state, const USBRequestPayload_BulkTransferBundle *request)
{
	return new WinUSBBulkTransferBundle(device_state, request);
}

bool WinUSBApi::get_usb_device_filter(const USBDeviceState* device_state, struct USBDeviceFilter *outDeviceInfo) const
{
	bool bSuccess = false;

	if (device_state != nullptr)
	{
		const WinUSBDeviceState *winusb_device_state = static_cast<const WinUSBDeviceState *>(device_state);

		outDeviceInfo->product_id = winusb_device_state->product_id;
		outDeviceInfo->vendor_id = winusb_device_state->vendor_id;
        outDeviceInfo->interface_mask = (winusb_device_state->composite_interface_index != -1) ? (1 << winusb_device_state->composite_interface_index) : 0;
		bSuccess = true;
	}

	return bSuccess;
}

bool WinUSBApi::get_usb_device_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const
{
	bool bSuccess = false;

	if (device_state != nullptr)
	{
		const WinUSBDeviceState *winusb_device_state = static_cast<const WinUSBDeviceState *>(device_state);

        strncpy(outBuffer, winusb_device_state->device_path.c_str(), bufferSize);
		bSuccess = true;
	}

	return bSuccess;
}

bool WinUSBApi::get_usb_device_port_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const
{
	bool bSuccess = false;

	if (device_state != nullptr)
	{
		const WinUSBDeviceState *winusb_device_state = static_cast<const WinUSBDeviceState *>(device_state);

        // This is a bit of a lazy short cut.
        // The unique ID contained in the full windows device path is unique and contains
        // "... a Windows-generated string based on things such as the physical USB port address and/or interface number."
        // but is not strictly the bus number and port path string the libusb version of this string.
        // Getting the same bus number and port path requires a much more complicated device iterator.
        // http://community.silabs.com/t5/Interface-Knowledge-Base/Windows-USB-Device-Path/ta-p/114059
        strncpy(outBuffer, winusb_device_state->unique_id.c_str(), bufferSize);
		bSuccess = true;
	}

	return bSuccess;
}

static std::string GetLastErrorAsString()
{
    DWORD errorMessageID = GetLastError();

    if(errorMessageID == 0)
        return std::string(); //No error message has been recorded

    LPSTR messageBuffer = nullptr;
    size_t size = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL, 
        errorMessageID, 
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), 
        (LPSTR)&messageBuffer, 
        0, 
        NULL);
    std::string message(messageBuffer, size);

    LocalFree(messageBuffer);

    return message;
}