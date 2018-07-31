// -- includes -----
#include "WMFVideo.h"
#include "DeviceInterface.h"
#include "Logger.h"
#include "Utility.h"
#include "TrackerDeviceEnumerator.h"
#include "WMFCameraEnumerator.h"
#include "TrackerManager.h"
#include "WorkerThread.h"

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

// -- WMF Video Device -----
WMFVideoDevice::WMFVideoDevice(const int device_index, const WMFDeviceInfo &device_info) 
	: m_deviceIndex(device_index)
	, m_deviceInfo(device_info)
	, m_deviceFormatIndex(INVALID_DEVICE_FORMAT_INDEX)
	, m_mediaSource(nullptr)
	, m_videoFrameProcessor(nullptr)
{
}

WMFVideoDevice::~WMFVideoDevice()
{
	close();
}

bool WMFVideoDevice::open(
	int desiredFormatIndex, 
	WMFCommonTrackerConfig &cfg, 
	ITrackerListener *trackerListener)
{
	HRESULT hr;

	if (getIsOpen() && desiredFormatIndex == m_deviceFormatIndex)
	{
		return true;
	}

	if (desiredFormatIndex > 0 && desiredFormatIndex < m_deviceInfo.deviceAvailableFormats.size())
	{
		IMFAttributes *pAttributes = NULL;
		IMFActivate * vd_pActivate= NULL;

		// Close the device if it's currently open
		if (getIsOpen())
		{
			close();
		}

		// Remember the last 
		this->m_deviceFormatIndex= desiredFormatIndex;

		hr = MFCreateAttributes(&pAttributes, 1);
   
		if (SUCCEEDED(hr))
		{
			hr = pAttributes->SetGUID(
				MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
				MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
		}	

		IMFActivate *deviceActivationInterface= nullptr;
		if (SUCCEEDED(hr))
		{
			IMFActivate **ppDevices = nullptr;
			UINT32 deviceCount;
			hr = MFEnumDeviceSources(pAttributes, &ppDevices, &deviceCount);

			if (m_deviceIndex >= 0 && m_deviceIndex < (int)deviceCount)
			{
				deviceActivationInterface= ppDevices[m_deviceIndex];
				deviceActivationInterface->AddRef();
			}

			for(UINT32 i = 0; i < deviceCount; i++)
			{
				Utility::SafeRelease(&ppDevices[i]);
			}

			Utility::SafeReleaseAllCount(ppDevices);
		}

		if (SUCCEEDED(hr))
		{
			hr = deviceActivationInterface->ActivateObject(
				__uuidof(IMFMediaSource),
				(void**)&m_mediaSource);
		}

		IMFPresentationDescriptor *pPD = nullptr;
		if (SUCCEEDED(hr))
			hr = m_mediaSource->CreatePresentationDescriptor(&pPD);

		BOOL fSelected;
		IMFStreamDescriptor *pSD = nullptr;
		if (SUCCEEDED(hr))
			hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);

		IMFMediaTypeHandler *pHandler = nullptr;
		if (SUCCEEDED(hr))
			hr = pSD->GetMediaTypeHandler(&pHandler);

		DWORD cTypes = 0;
		if (SUCCEEDED(hr))
			hr = pHandler->GetMediaTypeCount(&cTypes);

		IMFMediaType *pType = nullptr;
		if (SUCCEEDED(hr))
			hr = pHandler->GetMediaTypeByIndex((DWORD)m_deviceFormatIndex, &pType);

		if (SUCCEEDED(hr))
			hr = pHandler->SetCurrentMediaType(pType);

		if (SUCCEEDED(hr))
		{
			const WMFDeviceFormatInfo &deviceFormat= 
				m_deviceInfo.deviceAvailableFormats[desiredFormatIndex];

			m_videoFrameProcessor = new WMFVideoFrameProcessor(deviceFormat, trackerListener);
			hr= m_videoFrameProcessor->init(m_mediaSource);
		}

		if (SUCCEEDED(hr))
		{
			// Update the property constraints for the current video format
			for (int prop_index = 0; prop_index < PSVRVideoProperty_COUNT; ++prop_index)
			{
				getVideoPropertyConstraint((PSVRVideoPropertyType)prop_index, m_videoPropertyConstraints[prop_index]);
			}

			// Apply video property settings stored in config onto the camera
			for (int prop_index = 0; prop_index < PSVRVideoProperty_COUNT; ++prop_index)
			{
				const PSVRVideoPropertyType prop_type = (PSVRVideoPropertyType)prop_index;
				const PSVRVideoPropertyConstraint &constraint= m_videoPropertyConstraints[prop_index];

				if (constraint.is_supported)
				{
					// Use the properties from the config if we used this video mode previously
					if (desiredFormatIndex == cfg.wmf_video_format_index)
					{
						int currentValue= getVideoProperty(prop_type);
						int desiredValue= cfg.video_properties[prop_index];

						if (desiredValue != currentValue ||
							prop_type == PSVRVideoProperty_Focus) // always set focus to disable auto-focus
						{
							// Use the desired value if it is in-range
							if (desiredValue >= constraint.min_value &&
								desiredValue <= constraint.max_value)
							{
								setVideoProperty(prop_type, desiredValue);
							}
							// Otherwise update the config to use the current value
							else
							{
								cfg.video_properties[prop_index]= currentValue;
							}
						}
					}
					// Otherwise use the current value for the property
					// and update the config to match
					else
					{
						int currentValue= getVideoProperty(prop_type);

						if (currentValue >= constraint.min_value &&
							currentValue <= constraint.max_value)
						{
							cfg.video_properties[prop_index]= currentValue;
						}
						else
						{
							// If the current value is somehow out-of-range
							// fallback to the default value
							setVideoProperty(prop_type, constraint.default_value);
							cfg.video_properties[prop_index]= constraint.default_value;
						}
					}
				}
			}

			// Remember which video format index that was last successfully opened
			cfg.wmf_video_format_index= desiredFormatIndex;
		}

		if (SUCCEEDED(hr))
			hr= m_videoFrameProcessor->start();

		Utility::SafeReleaseAllCount(&pPD);
		Utility::SafeRelease(&pSD);
		Utility::SafeRelease(&pHandler);
		Utility::SafeRelease(&pType);
		Utility::SafeReleaseAllCount(&deviceActivationInterface);
		Utility::SafeReleaseAllCount(&pAttributes);

		if (!SUCCEEDED(hr))
		{
			close();
		}
	}
	else
	{
		hr= E_INVALIDARG;
	}

	return SUCCEEDED(hr);
}

void WMFVideoDevice::close()
{
	if (m_videoFrameProcessor != nullptr)
	{
		delete m_videoFrameProcessor;
		m_videoFrameProcessor= nullptr;
	}

	if (m_mediaSource != nullptr)
	{
		m_mediaSource->Stop();
		Utility::SafeRelease(&m_mediaSource);
	}
}

bool WMFVideoDevice::getIsOpen() const
{
	return m_mediaSource != nullptr;
}

/*
	See https://msdn.microsoft.com/en-us/library/windows/desktop/dd407328(v=vs.85).aspx
	VideoProcAmp_Brightness		[-10k, 10k]
	VideoProcAmp_Contrast			[0, 10k]
	VideoProcAmp_Hue				[-180k, 180k]  
	VideoProcAmp_Saturation		[0, 10k]
	VideoProcAmp_Sharpness		[0, 100]
	VideoProcAmp_Gamma			[1, 500]
	VideoProcAmp_ColorEnable		0=off, 1=on
	VideoProcAmp_WhiteBalance		device dependent
	VideoProcAmp_BacklightCompensation		0=off, 1=on
	VideoProcAmp_Gain				device dependent
*/
bool WMFVideoDevice::setProcAmpProperty(VideoProcAmpProperty propId, long value, bool bAuto)
{
	bool bSuccess= false;

	IAMVideoProcAmp *pProcAmp = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

	if (SUCCEEDED(hr))
	{
		hr = pProcAmp->Set(propId, value, bAuto ? VideoProcAmp_Flags_Auto : VideoProcAmp_Flags_Manual);
		pProcAmp->Release();
	}

	return SUCCEEDED(hr);
}

long WMFVideoDevice::getProcAmpProperty(VideoProcAmpProperty propId, bool *bIsAuto) const
{
	long intValue= 0;
	IAMVideoProcAmp *pProcAmp = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

	if (SUCCEEDED(hr))
	{
		long flags;
		hr = pProcAmp->Get(propId, &intValue, &flags);

		if (bIsAuto != nullptr)
		{
			*bIsAuto = flags == VideoProcAmp_Flags_Auto;
		}

		pProcAmp->Release();
	}

	return intValue;
}

bool WMFVideoDevice::getProcAmpRange(VideoProcAmpProperty propId, PSVRVideoPropertyConstraint &constraint) const
{
	IAMVideoProcAmp *pProcAmp = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

	memset(&constraint, 0, sizeof(PSVRVideoPropertyConstraint));

	if (SUCCEEDED(hr))
	{
		long minValue, maxValue, stepSize, defaultValue, flags;
		hr = pProcAmp->GetRange(propId, &minValue, &maxValue, &stepSize, &defaultValue, &flags);

		constraint.default_value= defaultValue;
		constraint.min_value= minValue;
		constraint.max_value= maxValue;
		constraint.stepping_delta= stepSize;
		constraint.is_supported= true;
		constraint.is_automatic= flags == VideoProcAmp_Flags_Auto;

		pProcAmp->Release();
	}

	return SUCCEEDED(hr);
}

bool WMFVideoDevice::setCameraControlProperty(CameraControlProperty propId, long value, bool bAuto)
{
	bool bSuccess= false;

	IAMCameraControl *pProcControl = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pProcControl));

	if (SUCCEEDED(hr))
	{
		hr = pProcControl->Set(propId, value, bAuto ? CameraControl_Flags_Auto : CameraControl_Flags_Manual);

		pProcControl->Release();
	}

	return SUCCEEDED(hr);
}

long WMFVideoDevice::getCameraControlProperty(CameraControlProperty propId, bool *bIsAuto) const
{
	long intValue= 0;
	IAMCameraControl *pCameraControl = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pCameraControl));

	if (SUCCEEDED(hr))
	{
		long flags;
		hr = pCameraControl->Get(propId, &intValue, &flags);

		if (bIsAuto != nullptr)
		{
			*bIsAuto = flags == CameraControl_Flags_Auto;
		}

		pCameraControl->Release();
	}

	return intValue;
}

bool WMFVideoDevice::getCameraControlRange(
	CameraControlProperty propId, PSVRVideoPropertyConstraint &constraint) const
{
	double unitValue= 0;
	IAMCameraControl *pCameraControl = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pCameraControl));

	memset(&constraint, 0, sizeof(PSVRVideoPropertyConstraint));

	if (SUCCEEDED(hr))
	{
		long minValue, maxValue, stepSize, defaultValue, flags;
		hr = pCameraControl->GetRange(propId, &minValue, &maxValue, &stepSize, &defaultValue, &flags);

		if (SUCCEEDED(hr))
		{
			constraint.default_value= defaultValue;
			constraint.min_value= minValue;
			constraint.max_value= maxValue;
			constraint.stepping_delta= stepSize;
			constraint.is_supported= true;
			constraint.is_automatic= flags == VideoProcAmp_Flags_Auto;
		}

		pCameraControl->Release();
	}

	return SUCCEEDED(hr);
}

bool WMFVideoDevice::getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const
{
	bool bSuccess= false;

	switch (property_type)
	{
    case PSVRVideoProperty_Brightness:
		bSuccess= getProcAmpRange(VideoProcAmp_Brightness, outConstraint);
		break;
	case PSVRVideoProperty_Contrast:
		bSuccess= getProcAmpRange(VideoProcAmp_Contrast, outConstraint);
		break;
	case PSVRVideoProperty_Hue:
		bSuccess= getProcAmpRange(VideoProcAmp_Hue, outConstraint);
		break;
	case PSVRVideoProperty_Saturation:
		bSuccess= getProcAmpRange(VideoProcAmp_Saturation, outConstraint);
		break;
	case PSVRVideoProperty_Sharpness:
		bSuccess= getProcAmpRange(VideoProcAmp_Sharpness, outConstraint);
		break;
	case PSVRVideoProperty_Gamma:
		bSuccess= getProcAmpRange(VideoProcAmp_Gamma, outConstraint);
		break;
	case PSVRVideoProperty_WhiteBalance:
		bSuccess= getProcAmpRange(VideoProcAmp_WhiteBalance, outConstraint);
		break;
	case PSVRVideoProperty_Gain:
		bSuccess= getProcAmpRange(VideoProcAmp_Gain, outConstraint);
		break;
	case PSVRVideoProperty_Pan:
		bSuccess= getCameraControlRange(CameraControl_Pan, outConstraint);
		break;
	case PSVRVideoProperty_Tilt:
		bSuccess= getCameraControlRange(CameraControl_Tilt, outConstraint);
		break;
	case PSVRVideoProperty_Roll:
		bSuccess= getCameraControlRange(CameraControl_Roll, outConstraint);
		break;
	case PSVRVideoProperty_Zoom:
		bSuccess= getCameraControlRange(CameraControl_Zoom, outConstraint);
		break;
	case PSVRVideoProperty_Exposure:
		bSuccess= getCameraControlRange(CameraControl_Exposure, outConstraint);
		break;
	case PSVRVideoProperty_Iris:
		bSuccess= getCameraControlRange(CameraControl_Iris, outConstraint);
		break;
	case PSVRVideoProperty_Focus:
		bSuccess= getCameraControlRange(CameraControl_Focus, outConstraint);
		break;
	}
	
	return bSuccess;
}

void WMFVideoDevice::setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value)
{
	switch (property_type)
	{
    case PSVRVideoProperty_Brightness:
		setProcAmpProperty(VideoProcAmp_Brightness, desired_value, false);
		break;
	case PSVRVideoProperty_Contrast:
		setProcAmpProperty(VideoProcAmp_Contrast, desired_value, false);
		break;
	case PSVRVideoProperty_Hue:
		setProcAmpProperty(VideoProcAmp_Hue, desired_value, false);
		break;
	case PSVRVideoProperty_Saturation:
		setProcAmpProperty(VideoProcAmp_Saturation, desired_value, false);
		break;
	case PSVRVideoProperty_Sharpness:
		setProcAmpProperty(VideoProcAmp_Sharpness, desired_value, false);
		break;
	case PSVRVideoProperty_Gamma:
		setProcAmpProperty(VideoProcAmp_Gamma, desired_value, false);
		break;
	case PSVRVideoProperty_WhiteBalance:
		setProcAmpProperty(VideoProcAmp_WhiteBalance, desired_value, false);
		break;
	case PSVRVideoProperty_Gain:
		setProcAmpProperty(VideoProcAmp_Gain, desired_value, false);
		break;
	case PSVRVideoProperty_Pan:
		setCameraControlProperty(CameraControl_Pan, desired_value, false);
		break;
	case PSVRVideoProperty_Tilt:
		setCameraControlProperty(CameraControl_Tilt, desired_value, false);
		break;
	case PSVRVideoProperty_Roll:
		setCameraControlProperty(CameraControl_Roll, desired_value, false);
		break;
	case PSVRVideoProperty_Zoom:
		setCameraControlProperty(CameraControl_Zoom, desired_value, false);
		break;
	case PSVRVideoProperty_Exposure:
		setCameraControlProperty(CameraControl_Exposure, desired_value, false);
		break;
	case PSVRVideoProperty_Iris:
		setCameraControlProperty(CameraControl_Iris, desired_value, false);
		break;
	case PSVRVideoProperty_Focus:
		setCameraControlProperty(CameraControl_Focus, desired_value, false);
		break;
	}
}

int WMFVideoDevice::getVideoProperty(const PSVRVideoPropertyType property_type) const
{
	int value= 0;

	switch (property_type)
	{
    case PSVRVideoProperty_Brightness:
		value= getProcAmpProperty(VideoProcAmp_Brightness);
		break;
	case PSVRVideoProperty_Contrast:
		value= getProcAmpProperty(VideoProcAmp_Contrast);
		break;
	case PSVRVideoProperty_Hue:
		value= getProcAmpProperty(VideoProcAmp_Hue);
		break;
	case PSVRVideoProperty_Saturation:
		value= getProcAmpProperty(VideoProcAmp_Saturation);
		break;
	case PSVRVideoProperty_Sharpness:
		value= getProcAmpProperty(VideoProcAmp_Sharpness);
		break;
	case PSVRVideoProperty_Gamma:
		value= getProcAmpProperty(VideoProcAmp_Gamma);
		break;
	case PSVRVideoProperty_WhiteBalance:
		value= getProcAmpProperty(VideoProcAmp_WhiteBalance);
		break;
	case PSVRVideoProperty_Gain:
		value= getProcAmpProperty(VideoProcAmp_Gain);
		break;
	case PSVRVideoProperty_Pan:
		value= getCameraControlProperty(CameraControl_Pan);
		break;
	case PSVRVideoProperty_Tilt:
		value= getCameraControlProperty(CameraControl_Tilt);
		break;
	case PSVRVideoProperty_Roll:
		value= getCameraControlProperty(CameraControl_Roll);
		break;
	case PSVRVideoProperty_Zoom:
		value= getCameraControlProperty(CameraControl_Zoom);
		break;
	case PSVRVideoProperty_Exposure:
		value= getCameraControlProperty(CameraControl_Exposure);
		break;
	case PSVRVideoProperty_Iris:
		value= getCameraControlProperty(CameraControl_Iris);
		break;
	case PSVRVideoProperty_Focus:
		value= getCameraControlProperty(CameraControl_Focus);
		break;
	}

	return value;
}

const WMFDeviceFormatInfo *WMFVideoDevice::getCurrentDeviceFormat() const
{
	return 
		(m_deviceFormatIndex != INVALID_DEVICE_FORMAT_INDEX)
		? &m_deviceInfo.deviceAvailableFormats[m_deviceFormatIndex]
		: nullptr;
}

// -- WMF Video Frame Processor -----
WMFVideoFrameProcessor::WMFVideoFrameProcessor(
	const WMFDeviceFormatInfo &deviceFormat, ITrackerListener *listener)
	: WorkerThread("WMFVideoFrameProcessor")
	, m_referenceCount(1)
	, m_pSession(nullptr)
	, m_pTopology(nullptr)
	, m_bIsRunning(false)
	, m_trackerListener(listener)
{
}

WMFVideoFrameProcessor::~WMFVideoFrameProcessor(void)
{
	dispose();
}

HRESULT WMFVideoFrameProcessor::init(IMFMediaSource *pSource)
{
	// Clean up previous session, if any.
    if (m_pSession)
    {
        m_pSession->Shutdown();
    }
    Utility::SafeReleaseAllCount(&m_pSession);
    Utility::SafeReleaseAllCount(&m_pTopology);

	// Configure the media type that the video frame processor will receive.
	// Setting the major and subtype is usually enough for the topology loader
	// to resolve the topology.
    IMFMediaType *pType = nullptr;
	HRESULT hr = MFCreateMediaType(&pType);

	if (SUCCEEDED(hr))
		hr = pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);

	if (SUCCEEDED(hr))
		hr = pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB24);

	// Create the sample grabber sink.
    IMFActivate *pSinkActivate = nullptr;
	if (SUCCEEDED(hr))
		hr = MFCreateSampleGrabberSinkActivate(pType, this, &pSinkActivate);

	// To run as fast as possible, set this attribute (requires Windows 7):
	if (SUCCEEDED(hr))
		hr = pSinkActivate->SetUINT32(MF_SAMPLEGRABBERSINK_IGNORE_CLOCK, TRUE);

	// Create the Media Session.
	if (SUCCEEDED(hr))
		hr = MFCreateMediaSession(NULL, &m_pSession);
	
	// Create the topology.
	if (SUCCEEDED(hr))
		hr = CreateTopology(pSource, pSinkActivate, &m_pTopology);

	
	// Clean up.
	if (FAILED(hr))
	{		
		if (m_pSession)
		{
			m_pSession->Shutdown();
		}

		Utility::SafeRelease(&m_pSession);
		Utility::SafeRelease(&m_pTopology);
	}

    Utility::SafeRelease(&pSinkActivate);
    Utility::SafeRelease(&pType);

	return hr;
}

void WMFVideoFrameProcessor::dispose()
{
	stop();

	if (m_pSession)
    {
		m_pSession->Shutdown();
    }
					
	Utility::SafeReleaseAllCount(&m_pSession);
	Utility::SafeReleaseAllCount(&m_pTopology);

	PSVR_LOG_INFO("WMFVideoFrameProcessor::dispose") << "Disposing video frame grabber for device: " << m_deviceIndex;
}

HRESULT WMFVideoFrameProcessor::start(void)
{
    HRESULT hr = m_pSession->SetTopology(0, m_pTopology);

	if (SUCCEEDED(hr))
	{
		PROPVARIANT var;
		PropVariantInit(&var);

		hr = m_pSession->Start(&GUID_NULL, &var);
	}

	if (SUCCEEDED(hr))
	{
		m_bIsRunning= true;
		WorkerThread::startThread();
	}

    return hr;
}

bool WMFVideoFrameProcessor::doWork()
{
	bool bKeepRunning= true;

	IMFMediaEvent *pEvent = nullptr;
	HRESULT hr = m_pSession->GetEvent(0, &pEvent);

	if (SUCCEEDED(hr))
	{
		HRESULT hrStatus;
		hr = pEvent->GetStatus(&hrStatus);

		if (!SUCCEEDED(hr) || !SUCCEEDED(hrStatus))
		{
			bKeepRunning= false;
			hr= E_FAIL;
		}

		MediaEventType met;
		if (SUCCEEDED(hr))
		{
			hr = pEvent->GetType(&met);
		}

		if(SUCCEEDED(hr))
		{
			switch (met)
			{
			case MESessionEnded:
				{
					PSVR_MT_LOG_INFO("WMFVideoFrameProcessor::doWork") << "MESessionEnded: " << m_deviceIndex;
					bKeepRunning= false;
				} break;

			case MESessionStopped:
				{
					PSVR_MT_LOG_INFO("WMFVideoFrameProcessor::doWork") << "MESessionStopped: " << m_deviceIndex;
					bKeepRunning= false;
				} break;

			case MEVideoCaptureDeviceRemoved:
				{
					PSVR_MT_LOG_INFO("WMFVideoFrameProcessor::doWork") << "MEVideoCaptureDeviceRemoved: " << m_deviceIndex;
					bKeepRunning= false;
				} break;
			}
		}
		else
		{
			bKeepRunning= false;
		}
	}
	else
	{
		bKeepRunning= false;
	}

	Utility::SafeRelease(&pEvent);

	return bKeepRunning;
}

void WMFVideoFrameProcessor::stop()
{
	PSVR_LOG_INFO("WMFVideoFrameProcessor::stop") << "Stopping video frame grabbing on device: " << m_deviceIndex;

	if (m_bIsRunning)
	{
		if (m_pSession != nullptr)
		{
			// This will send a MESessionStopped event to the worker thread
			m_pSession->Stop();
		}

		WorkerThread::stopThread();

		m_bIsRunning= false;
	}
}

HRESULT WMFVideoFrameProcessor::CreateTopology(
	IMFMediaSource *pSource, IMFActivate *pSinkActivate, IMFTopology **ppTopo)
{
    IMFTopology *pTopology = nullptr;
    HRESULT hr = MFCreateTopology(&pTopology);

    IMFPresentationDescriptor *pPD = nullptr;
	if (SUCCEEDED(hr))
		hr = pSource->CreatePresentationDescriptor(&pPD);

    DWORD cStreams = 0;
	if (SUCCEEDED(hr))
		hr = pPD->GetStreamDescriptorCount(&cStreams);
    
	if (SUCCEEDED(hr))
	{
		for (DWORD i = 0; i < cStreams; i++)
		{
			// Look for video streams and connect them to the sink
			BOOL fSelected = FALSE;
			GUID majorType;

		    IMFStreamDescriptor *pSD = nullptr;
			hr = pPD->GetStreamDescriptorByIndex(i, &fSelected, &pSD);

		    IMFMediaTypeHandler *pHandler = nullptr;
			if (SUCCEEDED(hr))
				hr = pSD->GetMediaTypeHandler(&pHandler);

			if (SUCCEEDED(hr))
				hr = pHandler->GetMajorType(&majorType);

			if (SUCCEEDED(hr))
			{
				if (majorType == MFMediaType_Video && fSelected)
				{
					IMFTopologyNode *pNode1 = nullptr;
					IMFTopologyNode *pNode2 = nullptr;

					hr = AddSourceNode(pTopology, pSource, pPD, pSD, &pNode1);

					if (SUCCEEDED(hr))
						hr = AddOutputNode(pTopology, pSinkActivate, 0, &pNode2);

					if (SUCCEEDED(hr))
						hr = pNode1->ConnectOutput(0, pNode2, 0);

					Utility::SafeRelease(&pNode1);
					Utility::SafeRelease(&pNode2);
					break;
				}
				else
				{
					hr = pPD->DeselectStream(i);
				}
			}

			Utility::SafeRelease(&pSD);
			Utility::SafeRelease(&pHandler);

			if (FAILED(hr))
				break;
		}
	}

	if (SUCCEEDED(hr))
	{
		*ppTopo = pTopology;
		(*ppTopo)->AddRef();
	}

    Utility::SafeRelease(&pTopology);
    Utility::SafeRelease(&pPD);

    return hr;
}

HRESULT WMFVideoFrameProcessor::AddSourceNode(
	IMFTopology *pTopology,           
	IMFMediaSource *pSource,          
	IMFPresentationDescriptor *pPD,   
	IMFStreamDescriptor *pSD,         
	IMFTopologyNode **ppNode)
{
    IMFTopologyNode *pNode = nullptr;
    HRESULT hr = MFCreateTopologyNode(MF_TOPOLOGY_SOURCESTREAM_NODE, &pNode);

	if (SUCCEEDED(hr))
		hr = pNode->SetUnknown(MF_TOPONODE_SOURCE, pSource);

	if (SUCCEEDED(hr))
		hr = pNode->SetUnknown(MF_TOPONODE_PRESENTATION_DESCRIPTOR, pPD);

	if (SUCCEEDED(hr))
		hr = pNode->SetUnknown(MF_TOPONODE_STREAM_DESCRIPTOR, pSD);

	if (SUCCEEDED(hr))
		hr = pTopology->AddNode(pNode);

    // Return the pointer to the caller.
	if (SUCCEEDED(hr))
	{
		*ppNode = pNode;
		(*ppNode)->AddRef();
	}

    Utility::SafeRelease(&pNode);

    return hr;
}

HRESULT WMFVideoFrameProcessor::AddOutputNode(
	IMFTopology *pTopology,
	IMFActivate *pActivate,
	DWORD dwId,
	IMFTopologyNode **ppNode)
{
    IMFTopologyNode *pNode = NULL;

    HRESULT hr = MFCreateTopologyNode(MF_TOPOLOGY_OUTPUT_NODE, &pNode);

	if (SUCCEEDED(hr))
		hr = pNode->SetObject(pActivate);

	if (SUCCEEDED(hr))
		hr = pNode->SetUINT32(MF_TOPONODE_STREAMID, dwId);

	if (SUCCEEDED(hr))
		hr = pNode->SetUINT32(MF_TOPONODE_NOSHUTDOWN_ON_REMOVE, FALSE);

	if (SUCCEEDED(hr))
		hr = pTopology->AddNode(pNode);

    // Return the pointer to the caller.
	if (SUCCEEDED(hr))
	{
		*ppNode = pNode;
		(*ppNode)->AddRef();
	}

    Utility::SafeRelease(&pNode);

    return hr;
}
	
STDMETHODIMP WMFVideoFrameProcessor::QueryInterface(REFIID riid, void** ppv)
{
	// Creation tab of shifting interfaces from start of this class
    static const QITAB qit[] = 
    {
        QITABENT(WMFVideoFrameProcessor, IMFSampleGrabberSinkCallback),
        QITABENT(WMFVideoFrameProcessor, IMFClockStateSink),
        { 0 }
    };
    return QISearch(this, qit, riid, ppv);
}

STDMETHODIMP_(ULONG) WMFVideoFrameProcessor::AddRef()
{
    return InterlockedIncrement(&m_referenceCount);
}

STDMETHODIMP_(ULONG) WMFVideoFrameProcessor::Release()
{
    ULONG cRef = InterlockedDecrement(&m_referenceCount);
    if (cRef == 0)
    {
        delete this;
    }
    return cRef;
}

STDMETHODIMP WMFVideoFrameProcessor::OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
	LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE * pSampleBuffer,
	DWORD dwSampleSize)
{
	if (m_trackerListener)
	{
		m_trackerListener->notifyVideoFrameReceived(static_cast<const unsigned char *>(pSampleBuffer));
	}

	return S_OK;
}
