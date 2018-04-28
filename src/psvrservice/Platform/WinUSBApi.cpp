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

#ifdef _MSC_VER
#pragma warning(disable:4996) // disable warnings about strncpy
#endif

// https://msdn.microsoft.com/en-us/library/windows/hardware/ff540174(v=vs.85).aspx

//-- constants ----
static GUID WINUSB_GUID_DEVCLASS_IMAGE = { 0x6bdd1fc6, 0x810f, 0x11d0, 0xbe, 0xc7, 0x08, 0x00, 0x2b, 0xe2, 0x09, 0x2f };
static GUID WINUSB_GUID_DEVCLASS_HID = { 0x4d1e55b2, 0xf16f, 0x11cf, 0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30 };
static GUID WINUSB_GUID_DEVCLASS_USB_RAW = { 0xa5dcbf10, 0x6530, 0x11d2, 0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed };
static GUID WINUSB_GUID_DEVCLASS_LIBUSB = { 0xeb781aaf, 0x9c70, 0x4523, 0xa5, 0xdf, 0x64, 0x2a, 0x87, 0xec, 0xa5, 0x67 };

enum libusb_endpoint_direction {
	/** In: device-to-host */
	LIBUSB_ENDPOINT_IN = 0x80,

	/** Out: host-to-device */
	LIBUSB_ENDPOINT_OUT = 0x00
};
	
/** \ingroup libusb_misc
 * Request type bits of the
 * \ref libusb_control_setup::bmRequestType "bmRequestType" field in control
 * transfers. */
enum libusb_request_type {
	/** Standard */
	LIBUSB_REQUEST_TYPE_STANDARD = (0x00 << 5),

	/** Class */
	LIBUSB_REQUEST_TYPE_CLASS = (0x01 << 5),

	/** Vendor */
	LIBUSB_REQUEST_TYPE_VENDOR = (0x02 << 5),

	/** Reserved */
	LIBUSB_REQUEST_TYPE_RESERVED = (0x03 << 5)
};

/** \ingroup libusb_misc
 * Recipient bits of the
 * \ref libusb_control_setup::bmRequestType "bmRequestType" field in control
 * transfers. Values 4 through 31 are reserved. */
enum libusb_request_recipient {
	/** Device */
	LIBUSB_RECIPIENT_DEVICE = 0x00,

	/** Interface */
	LIBUSB_RECIPIENT_INTERFACE = 0x01,

	/** Endpoint */
	LIBUSB_RECIPIENT_ENDPOINT = 0x02,

	/** Other */
	LIBUSB_RECIPIENT_OTHER = 0x03,
};

/** \ingroup libusb_misc
 * Standard requests, as defined in table 9-5 of the USB 3.0 specifications */
enum libusb_standard_request {
	/** Request status of the specific recipient */
	LIBUSB_REQUEST_GET_STATUS = 0x00,

	/** Clear or disable a specific feature */
	LIBUSB_REQUEST_CLEAR_FEATURE = 0x01,

	/* 0x02 is reserved */

	/** Set or enable a specific feature */
	LIBUSB_REQUEST_SET_FEATURE = 0x03,

	/* 0x04 is reserved */

	/** Set device address for all future accesses */
	LIBUSB_REQUEST_SET_ADDRESS = 0x05,

	/** Get the specified descriptor */
	LIBUSB_REQUEST_GET_DESCRIPTOR = 0x06,

	/** Used to update existing descriptors or add new descriptors */
	LIBUSB_REQUEST_SET_DESCRIPTOR = 0x07,

	/** Get the current device configuration value */
	LIBUSB_REQUEST_GET_CONFIGURATION = 0x08,

	/** Set device configuration */
	LIBUSB_REQUEST_SET_CONFIGURATION = 0x09,

	/** Return the selected alternate setting for the specified interface */
	LIBUSB_REQUEST_GET_INTERFACE = 0x0A,

	/** Select an alternate interface for the specified interface */
	LIBUSB_REQUEST_SET_INTERFACE = 0x0B,

	/** Set then report an endpoint's synchronization frame */
	LIBUSB_REQUEST_SYNCH_FRAME = 0x0C,

	/** Sets both the U1 and U2 Exit Latency */
	LIBUSB_REQUEST_SET_SEL = 0x30,

	/** Delay from the time a host transmits a packet to the time it is
	  * received by the device. */
	LIBUSB_SET_ISOCH_DELAY = 0x31,
};

//-- public interface -----

//-- definitions -----
struct WinUSBDeviceInfo
{
    std::string DevicePath;
    std::vector<GUID> DeviceInterfaceGUIDs;
    int ProductId;
    int VendorId;
    int CompositeInterfaceIndex;

	WinUSBDeviceInfo()
		: DevicePath()
		, DeviceInterfaceGUIDs()
		, ProductId(-1)
		, VendorId(-1)
		, CompositeInterfaceIndex(-1)
	{
	}

	bool init(const std::string &path, const GUID &guid)
	{
		bool bSuccess= false;

		DevicePath= path;
		DeviceInterfaceGUIDs.push_back(guid);
		ProductId= -1;
		VendorId= -1;
		CompositeInterfaceIndex= -1;

		// USB Instance ID w/ Interface Index: 
		//    "\\?\usb#vid_vvvv&pid_pppp&mi_ii#aaaaaaaaaaaaaaaa#{gggggggg-gggg-gggg-gggg-gggggggggggg}"
		if (sscanf_s(
				path.c_str(), 
				"\\\\?\\usb#vid_%X&pid_%X&mi_%X#", 
				&VendorId, 
				&ProductId, 
				&CompositeInterfaceIndex) == 3)
		{
			bSuccess= true;
		}
		// USB Instance ID: 
		//    "\\?\usb#vid_vvvv&pid_pppp#ssss#{gggggggg-gggg-gggg-gggg-gggggggggggg}"
		else if (sscanf_s(
					path.c_str(),
					"\\\\?\\usb#vid_%X&pid_%X#",
					&VendorId, 
					&ProductId) == 2)
		{
			bSuccess= true;
		}

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
		// https://www.silabs.com/community/interface/knowledge-base.entry.html/2013/11/21/windows_usb_devicep-aGxD
		//"Windows requires that the device path be unique for every USB device and interface. 
		//If two USB devices are plugged into the same machine with the same VID/PID/Serial string, 
		//then the USB Device Path Format described above won’t generate a unique string for the two devices. 
		//In this case, Windows generates a unique string similar to the format described in the 
		//Composite USB Device Path Format section. This method is also used if the USB device iSerial index is set to 0, 
		//indicating that the device does not have a serial string."
		return getDevicePath();
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