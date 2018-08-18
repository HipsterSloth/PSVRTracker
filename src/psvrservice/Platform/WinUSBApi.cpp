//-- includes -----
#include "Logger.h"
#include "USBDeviceFilter.h"
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
#include <cfgmgr32.h>   // for MAX_DEVICE_ID_LEN and CM_Get_Device_ID
#include <setupapi.h> // Device setup APIs
#include <Devpkey.h>
#include <strsafe.h>
#include <winusb.h>
#include <usb100.h>

#pragma comment(lib, "WinUsb.lib")

#include <functional>

#ifdef _MSC_VER
#pragma warning(disable:4996) // disable warnings about strncpy
#endif

// https://msdn.microsoft.com/en-us/library/windows/hardware/ff540174(v=vs.85).aspx

//-- constants ----
static GUID WINUSB_GUID_DEVCLASS_IMAGE = { 0x6bdd1fc6, 0x810f, 0x11d0, 0xbe, 0xc7, 0x08, 0x00, 0x2b, 0xe2, 0x09, 0x2f };
static GUID WINUSB_GUID_DEVCLASS_HID = { 0x4d1e55b2, 0xf16f, 0x11cf, 0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30 };
static GUID WINUSB_GUID_DEVCLASS_USB_RAW = { 0xa5dcbf10, 0x6530, 0x11d2, 0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed };
static GUID WINUSB_GUID_DEVCLASS_LIBUSB = { 0xeb781aaf, 0x9c70, 0x4523, 0xa5, 0xdf, 0x64, 0x2a, 0x87, 0xec, 0xa5, 0x67 };

//-- public interface -----

//-- definitions -----
struct WinUSBDeviceInfo
{
    std::string DevicePath;
	std::string UniqueIdentifier;
    std::vector<GUID> DeviceInterfaceGUIDs;
    int ProductId;
    int VendorId;
    int CompositeInterfaceIndex;

	WinUSBDeviceInfo()
		: DevicePath()
		, UniqueIdentifier()
		, DeviceInterfaceGUIDs()
		, ProductId(-1)
		, VendorId(-1)
		, CompositeInterfaceIndex(-1)
	{
	}

	bool init(const std::string &path, const GUID &guid)
	{
		bool bSuccess= false;
		char usbSerialString[4+1];
		char usbUniqueIDString[15+1];

		DevicePath= path;
		UniqueIdentifier= "";
		DeviceInterfaceGUIDs.push_back(guid);
		ProductId= -1;
		VendorId= -1;
		CompositeInterfaceIndex= -1;

		// USB Instance ID w/ Interface Index: 
		//    "\\?\usb#vid_vvvv&pid_pppp&mi_ii#aaaaaaaaaaaaaaaa#{gggggggg-gggg-gggg-gggg-gggggggggggg}"
		// Where:
		//    vvvv is the USB vendor ID represented in 4 hexadecimal characters.
		//    pppp is the USB product ID represented in 4 hexadecimal characters.
		//    ii is the USB interface number.
		//    aaaaaaaaaaaaaaaa is a unique, Windows-generated string based on things such as the physical USB port address and/or interface number.
		//    gggggggg-gggg-gggg-gggg-gggggggggggg is the device interface GUID that is used to link applications to device with specific drivers loaded.
		if (sscanf_s(
				path.c_str(), 
				"\\\\?\\usb#vid_%X&pid_%X&mi_%X#%15s#", 
				&VendorId, 
				&ProductId, 
				&CompositeInterfaceIndex,
				usbUniqueIDString,
				(unsigned)_countof(usbUniqueIDString)) == 4)
		{
			UniqueIdentifier= usbUniqueIDString;
			bSuccess= true;
		}
		// USB Instance ID: 
		//    "\\?\usb#vid_vvvv&pid_pppp#ssss#{gggggggg-gggg-gggg-gggg-gggggggggggg}"
		//Where:
		//    vvvv is the USB vendor ID represented in 4 hexadecimal characters.
		//    pppp is the USB product ID represented in 4 hexadecimal characters.
		//    ssss is the USB serial string represented in n characters.
		//    gggggggg-gggg-gggg-gggg-gggggggggggg is the device interface GUID that is used to link applications to device with specific drivers loaded.
		else if (sscanf_s(
					path.c_str(),
					"\\\\?\\usb#vid_%X&pid_%X#%4s#",
					&VendorId, 
					&ProductId,
					usbSerialString,
					(unsigned)_countof(usbSerialString)) == 3)
		{
			UniqueIdentifier= usbSerialString;
			bSuccess= true;
		}

		// Replace all "&" with "_" to make the unique identifier filename friendly
		std::replace( UniqueIdentifier.begin(), UniqueIdentifier.end(), '&', '_');

		return bSuccess;
	}

	bool hasDeviceInterfaceGUID(const GUID &g) const
	{
		return 
			std::find_if(
				DeviceInterfaceGUIDs.begin(), DeviceInterfaceGUIDs.end(), 
				[&g](const GUID &guid) {
					return IsEqualGUID(guid, g) == TRUE;
				}) != DeviceInterfaceGUIDs.end();
	}
};

struct WinUSBAsyncBulkTransfer
{
	void* device_handle;
	void* interface_handle;
	unsigned char bulk_endpoint;
    unsigned char *buffer;
    ULONG buffer_size;
    OVERLAPPED overlapped;
    ULONG transferred_bytes;
	t_winusb_bulk_transfer_callback callback;
	void *user_data;
	eWinusbBulkTransferStatus transfer_status;
};

struct WinUSBDeviceEnumerator : USBDeviceEnumerator
{
public:
	WinUSBDeviceEnumerator()
	{
		buildUSBDeviceList();

		USBDeviceEnumerator::device_index= -1;
		next();
	}

	bool isValid() const
	{
		return USBDeviceEnumerator::device_index < (int)m_deviceInfoList.size();
	}

	void next()
	{
		++USBDeviceEnumerator::device_index;
	}

    inline std::string getDevicePath() const
    {
        return (isValid()) ? m_deviceInfoList[USBDeviceEnumerator::device_index].DevicePath : std::string();
    }

	inline std::string getUniqueID() const 
	{
		return (isValid()) ? m_deviceInfoList[USBDeviceEnumerator::device_index].UniqueIdentifier : std::string();
	}

    inline int getProductId() const
    {
        return (isValid()) ? m_deviceInfoList[USBDeviceEnumerator::device_index].ProductId : -1;
    }

    inline int getVendorId() const
    {
        return (isValid()) ? m_deviceInfoList[USBDeviceEnumerator::device_index].VendorId : -1;
    }

    inline int getCompositeInterfaceIndex() const
    {
        return (isValid()) ? m_deviceInfoList[USBDeviceEnumerator::device_index].CompositeInterfaceIndex : -1;
    }

protected:
    static bool findDevicePathsWithInterfaceGUID(
		const GUID &deviceInterfaceGuid, 
		std::vector<std::string> &outDevicePathList)
    {
		HDEVINFO deviceInfoSetHandle = 
			SetupDiGetClassDevs(&deviceInterfaceGuid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
		bool bFoundAny= false;

        if (deviceInfoSetHandle != INVALID_HANDLE_VALUE)
        {
			DWORD deviceInterfaceIndex = 0;

			SP_DEVICE_INTERFACE_DATA interfaceData;
			ZeroMemory(&interfaceData, sizeof(SP_DEVICE_INTERFACE_DATA));
			interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

			while (SetupDiEnumDeviceInterfaces(
					deviceInfoSetHandle, 
					NULL, 
					&deviceInterfaceGuid, 
					deviceInterfaceIndex, 
					&interfaceData) == TRUE)
            {
				ULONG requiredLength=0; 
				SetupDiGetDeviceInterfaceDetail(deviceInfoSetHandle, &interfaceData, NULL, 0, &requiredLength, NULL);
				PSP_DEVICE_INTERFACE_DETAIL_DATA interfaceDetailData = 
					(PSP_DEVICE_INTERFACE_DETAIL_DATA)LocalAlloc(LMEM_FIXED, requiredLength);
                
				if (interfaceDetailData != NULL)
				{
					interfaceDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
					ULONG length = requiredLength;

					if (SetupDiGetDeviceInterfaceDetail(
							deviceInfoSetHandle, 
							&interfaceData, 
							interfaceDetailData, 
							length, 
							&requiredLength, 
							NULL) == TRUE)
					{
						char raw_device_path[512];
						#ifdef UNICODE
						wcstombs(raw_device_path, interfaceDetailData->DevicePath, (unsigned)_countof(raw_device_path));  
						#else
						strncpy_s(raw_device_path, interfaceDetailData->DevicePath, (unsigned)_countof(raw_device_path));
						#endif

						// Save the raw multi-byte device path
						outDevicePathList.push_back(std::string(raw_device_path));
						bFoundAny= true;
					}

					if (interfaceDetailData != NULL)
					{
						LocalFree(interfaceDetailData);
						interfaceDetailData= NULL;
					}
				}

				++deviceInterfaceIndex;
            }
			DWORD LastError= GetLastError();

			SetupDiDestroyDeviceInfoList(deviceInfoSetHandle);
        }

		return bFoundAny;
    }

	void buildUSBDeviceList()
	{
		HDEVINFO deviceInfoSetHandle = SetupDiGetClassDevs(NULL, TEXT("USB"), NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);

		SP_DEVINFO_DATA deviceInfoData;
		ZeroMemory(&deviceInfoData, sizeof(SP_DEVINFO_DATA));
		deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

		DWORD deviceInfoSetIndex = 0;
		while(SetupDiEnumDeviceInfo(deviceInfoSetHandle, deviceInfoSetIndex, &deviceInfoData))
		{
			TCHAR szDeviceInstanceID[MAX_DEVICE_ID_LEN];
			CONFIGRET status = CM_Get_Device_ID(deviceInfoData.DevInst, szDeviceInstanceID, MAX_DEVICE_ID_LEN, 0);
			if (status != CR_SUCCESS)
			{
				continue;
			}

			DWORD propertyType;
            TCHAR propBuffer[256];
            DWORD requiredSize;
			if (SetupDiGetCustomDeviceProperty(
				  deviceInfoSetHandle,
				  &deviceInfoData,
				  TEXT("DeviceInterfaceGuids"),
				  0,
				  &propertyType,
				  (LPBYTE)propBuffer,
				  sizeof(propBuffer),
				  &requiredSize) == TRUE)
			{
				// Convert a list of '/0' separated GUID strings into a GUID string list
				std::vector<std::string> devInterfaceGuids;
				for(const TCHAR* p=propBuffer; p!=propBuffer+requiredSize; p+=devInterfaceGuids.back().size()+1)
				{
					devInterfaceGuids.push_back(p);
				}

				// For each interface GUID:
				// 1) find all device paths using that interface GUID
				// 2) Add all unique device paths to the deviceInfoList
				for(const std::string &strGUID : devInterfaceGuids)
				{
					GUID guid;
					if (sscanf(strGUID.c_str(),
						   "{%8x-%4hx-%4hx-%2hhx%2hhx-%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx}",
						   &guid.Data1, &guid.Data2, &guid.Data3,
						   &guid.Data4[0], &guid.Data4[1], &guid.Data4[2], &guid.Data4[3],
						   &guid.Data4[4], &guid.Data4[5], &guid.Data4[6], &guid.Data4[7] ) == 11)
					{
						std::vector<std::string> devicePathList;
						if (findDevicePathsWithInterfaceGUID(guid, devicePathList))
						{
							for (const std::string &devicePath : devicePathList)
							{
								// Look for existing device entry by device path
								auto &existingDevInfoIter = 
									std::find_if(
										m_deviceInfoList.begin(), m_deviceInfoList.end(), 
										[&devicePath](WinUSBDeviceInfo &devInfo) {
											return devInfo.DevicePath == devicePath;
										});

								if (existingDevInfoIter != m_deviceInfoList.end())
								{
									WinUSBDeviceInfo &existingDeviceInfo= *existingDevInfoIter;

									if (!existingDeviceInfo.hasDeviceInterfaceGUID(guid))
									{
										existingDeviceInfo.DeviceInterfaceGUIDs.push_back(guid);
									}
								}
								else
								{
									WinUSBDeviceInfo newDeviceInfo;

									if (newDeviceInfo.init(devicePath, guid))
									{
										m_deviceInfoList.push_back(newDeviceInfo);
									}
								}
							}
						}
					}
				}
			}

			++deviceInfoSetIndex;
		}

		if (deviceInfoSetHandle != INVALID_HANDLE_VALUE)
		{
			SetupDiDestroyDeviceInfoList(deviceInfoSetHandle);
		}
	}

private:
	std::vector<WinUSBDeviceInfo> m_deviceInfoList;
};

//-- private methods -----
static std::string GetLastErrorAsString();

//-- WinUSBApi -----
WinUSBApi::WinUSBApi() : IUSBApi()
{
}

WinUSBApi *WinUSBApi::getInterface()
{
	return USBDeviceManager::getTypedUSBApiInterface<WinUSBApi>();
}

bool WinUSBApi::startup()
{
	return true;
}

void WinUSBApi::poll()
{
	// Poll the state of all pending async bulk transfers.
	// Remove any transfer that has completed.
	auto iter= m_pendingAsyncBulkTransfers.begin();
	while (iter != m_pendingAsyncBulkTransfers.end())
	{
		if (winusbPollAsyncBulkTransfer(*iter) != WINUSB_TRANSFER_PENDING)
		{
			iter= m_pendingAsyncBulkTransfers.erase(iter);
		}
		else
		{
			++iter;
		}
	}
}

void WinUSBApi::shutdown()
{
}

USBDeviceEnumerator* WinUSBApi::device_enumerator_create()
{
    WinUSBDeviceEnumerator *enumerator= new WinUSBDeviceEnumerator;

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

bool WinUSBApi::device_enumerator_get_unique_identifier(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const
{
	const WinUSBDeviceEnumerator *winusb_enumerator = static_cast<const WinUSBDeviceEnumerator *>(enumerator);
	bool bSuccess = false;

	if (winusb_enumerator != nullptr)
	{
        std::string deviceIdentifier= winusb_enumerator->getUniqueID();

		bSuccess = SUCCEEDED(StringCchCopy(outBuffer, bufferSize, deviceIdentifier.c_str()));
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

USBDeviceState *WinUSBApi::open_usb_device(
    USBDeviceEnumerator* enumerator, 
    int interface_index,
    int configuration_index,
    bool reset_device)
{
    WinUSBDeviceEnumerator *winusb_enumerator = static_cast<WinUSBDeviceEnumerator *>(enumerator);
	WinUSBDeviceState *winusb_device_state = nullptr;

    // RE: reset_device
    /*
    * from the "How to Use WinUSB to Communicate with a USB Device" Microsoft white paper
    * (http://www.microsoft.com/whdc/connect/usb/winusb_howto.mspx):
    * "WinUSB does not support host-initiated reset port and cycle port operations" and
    * IOCTL_INTERNAL_USB_CYCLE_PORT is only available in kernel mode and the
    * IOCTL_USB_HUB_CYCLE_PORT ioctl was removed from Vista => the best we can do is
    * cycle the pipes (and even then, the control pipe can not be reset using WinUSB)
    */

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
                // Set the configuration index on the device if one was given
                if (configuration_index != -1)
                {
                    // Mirroring what LibUSB does to set the configuration
                    // From: https://msdn.microsoft.com/en-us/library/ff539241.aspx:
                    // "The port driver does not currently expose a service that allows higher-level drivers to set the configuration"
                    WINUSB_SETUP_PACKET setupPacket;
                    memset(&setupPacket, 0, sizeof(WINUSB_SETUP_PACKET));
                    setupPacket.RequestType = USB_ENDPOINT_OUT | USB_REQUEST_TYPE_STANDARD | USB_RECIPIENT_DEVICE;
                    setupPacket.Request = USB_REQUEST_SET_CONFIGURATION;
                    setupPacket.Index = 0;
                    setupPacket.Length = 0;
                    setupPacket.Value = static_cast<USHORT>(configuration_index);

                    if (WinUsb_ControlTransfer(
                            winusb_device_state->interface_handle,
                            setupPacket,
                            nullptr,
                            0,
                            nullptr,
                            nullptr) == FALSE)
                    {
                        #if defined(DEBUG_USB)
                        DWORD LastErrorCode= GetLastError();
		                debug("USBMgr RESULT: Failed to set configuration index(%d) - dev: %d, error: 0x%x\n",
                            configuration_index,
			                request.usb_device_handle,
                            LastErrorCode);
                        #endif
                    }                
                }

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
										winusb_device_state->bulk_in_pipe_packet_size = pipeInfo.MaximumPacketSize;
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

	return winusb_device_state;
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
		else
		{
			result.payload.interrupt_transfer.dataLength= bytesWritten;
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
		else
		{
			result.payload.interrupt_transfer.dataLength= bytesRead;
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
		// Add the result to the outgoing result queue on success only
		usb_device_post_transfer_result(result, requestState->callback);
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

	if (request.wLength > 0)
	{
		memcpy(result.payload.control_transfer.data, request.data, request.wLength);
	}

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
		else
		{
			result.payload.control_transfer.dataLength= bytesWritten;
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
		else
		{
			result.payload.control_transfer.dataLength= bytesRead;
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
		// Add the result to the outgoing result queue on success only
		usb_device_post_transfer_result(result, requestState->callback);
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
		else
		{
			result.payload.bulk_transfer.dataLength= bytesWritten;
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
		else
		{
			result.payload.bulk_transfer.dataLength= bytesRead;
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
		// Add the result to the outgoing result queue on success only
		usb_device_post_transfer_result(result, requestState->callback);
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

WinUSBAsyncBulkTransfer *WinUSBApi::winusbAllocateAsyncBulkTransfer()
{
	WinUSBAsyncBulkTransfer *transfer= new WinUSBAsyncBulkTransfer;
	
	memset(transfer, 0, sizeof(WinUSBAsyncBulkTransfer));
	transfer->overlapped.hEvent= INVALID_HANDLE_VALUE;

	return transfer;
}

bool WinUSBApi::winusbSetupAsyncBulkTransfer(
	void *device_handle,
	void *interface_handle,
	const unsigned char bulk_endpoint,
	unsigned char *transfer_buffer,
	const size_t transfer_buffer_size,
	t_winusb_bulk_transfer_callback transfer_callback_function,
	void *user_data,
	WinUSBAsyncBulkTransfer *transfer)
{
	bool bSuccess= true;

	memset(transfer, 0, sizeof(WinUSBAsyncBulkTransfer));
	transfer->device_handle= device_handle;
	transfer->interface_handle= interface_handle;
	transfer->bulk_endpoint= bulk_endpoint;
	transfer->buffer= transfer_buffer;
	transfer->buffer_size= (ULONG)transfer_buffer_size;
	transfer->callback= transfer_callback_function;
	transfer->user_data= user_data;
	transfer->transfer_status= eWinusbBulkTransferStatus::WINUSB_TRANSFER_PENDING;

	transfer->overlapped.hEvent = CreateEvent(NULL, false, false, NULL);
    if (transfer->overlapped.hEvent == nullptr)
    {
		PSVR_MT_LOG_ERROR("WinUSBApi::close_usb_device") << "Failed to create event for async bulk transfer.";
		bSuccess= false;
    }

	return bSuccess;
}

void WinUSBApi::winusbFreeAsyncBulkTransfer(struct WinUSBAsyncBulkTransfer *transfer)
{
	if (transfer != nullptr)
	{
		if (transfer->transfer_status == WINUSB_TRANSFER_PENDING)
		{
			// Unfortunately, this transfer is still pending, so we cannot free it.
			// The kernel needs to be able to write to this transfer's memory when it completes.
			// Instead we prefer to just leak the WinUSBAsyncBulkTransfer object.
			return;
		}

		if (transfer->overlapped.hEvent != INVALID_HANDLE_VALUE)
		{
			CloseHandle(transfer->overlapped.hEvent);
		}

		delete transfer;
	}
}

bool WinUSBApi::winusbSubmitAsyncBulkTransfer(struct WinUSBAsyncBulkTransfer *transfer)
{
    bool bSuccess = WinUsb_ReadPipe(
        transfer->interface_handle,
        transfer->bulk_endpoint,
        transfer->buffer,
        transfer->buffer_size,
        &transfer->transferred_bytes,
        &transfer->overlapped) == TRUE;

	if (bSuccess)
	{
		transfer->transfer_status= WINUSB_TRANSFER_COMPLETED;
	}
	else
	{
		DWORD error_code= GetLastError();

		switch (error_code)
		{
		case ERROR_IO_INCOMPLETE:
		case ERROR_IO_PENDING:
			{
				// This is the expected common case
				transfer->transfer_status= WINUSB_TRANSFER_PENDING;
				bSuccess= true;
			}
			break;
		default:
			{
				std::string error_string= GetLastErrorAsString();

				PSVR_MT_LOG_ERROR("WinUSBApi::winusbSubmitAsyncBulkTransfer") << "Failed to start async bulk transfer: " << error_string;
				transfer->transfer_status= WINUSB_TRANSFER_ERROR;
			}
			break;
		}
	}

	if (transfer->transfer_status != WINUSB_TRANSFER_PENDING)
	{
		if (transfer->callback)
		{
			transfer->callback(
				transfer, 
				transfer->transfer_status, 
				transfer->buffer, 
				transfer->transferred_bytes, 
				transfer->user_data);
		}
	}
	else
	{
		// Check the state of the transfer again in poll()
		if (std::find(m_pendingAsyncBulkTransfers.begin(), m_pendingAsyncBulkTransfers.end(), transfer) == m_pendingAsyncBulkTransfers.end())
		{
			m_pendingAsyncBulkTransfers.push_back(transfer);
		}
	}

	return bSuccess;
}

bool WinUSBApi::winusbCancelAsyncBulkTransfer(struct WinUSBAsyncBulkTransfer *transfer)
{
	bool bSuccess= false;

    if (transfer != nullptr)
    {
		bSuccess = WinUsb_AbortPipe(transfer->interface_handle, transfer->bulk_endpoint) == TRUE;

		if (!bSuccess)
		{
			std::string error_string= GetLastErrorAsString();

			PSVR_MT_LOG_ERROR("WinUSBApi::winusbCancelAsyncBulkTransfer") << "Failed to create abort async pipe: " << error_string;
		}
    }

    return bSuccess;
}

eWinusbBulkTransferStatus WinUSBApi::winusbPollAsyncBulkTransfer(WinUSBAsyncBulkTransfer *transfer)
{
    assert(transfer != NULL);

    if (transfer->transfer_status == WINUSB_TRANSFER_PENDING)
    {
	    DWORD transferred_bytes = 0;
		BOOL success = 
			WinUsb_GetOverlappedResult(
				transfer->interface_handle, 
				&transfer->overlapped, 
				&transferred_bytes, 
				false);

		if (success)
		{
			transfer->transfer_status= WINUSB_TRANSFER_COMPLETED;
		}
		else
		{
			DWORD error_code= GetLastError();

			switch (error_code)
			{
			case ERROR_IO_INCOMPLETE:
			case ERROR_IO_PENDING:
				{
					transfer->transfer_status= WINUSB_TRANSFER_PENDING;
				}
				break;
			case ERROR_OPERATION_ABORTED:
				{
					transfer->transfer_status= WINUSB_TRANSFER_CANCELLED;
				}
				break;
			default:
				{
					std::string error_string= GetLastErrorAsString();

					PSVR_MT_LOG_ERROR("WinUSBApi::winusbPollAsyncBulkTransfer") << "Failure polling async transfer status: " << error_string;
					transfer->transfer_status= WINUSB_TRANSFER_ERROR;
				}
				break;
			}
		}

		if (transfer->transfer_status != WINUSB_TRANSFER_PENDING)
		{
			if (transfer->callback)
			{
				transfer->callback(
					transfer, 
					transfer->transfer_status, 
					transfer->buffer, 
					transferred_bytes, 
					transfer->user_data);
			}
		}
	}

	return transfer->transfer_status;
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