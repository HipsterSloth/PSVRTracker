#ifndef WMF_COMMON_H
#define WMF_COMMON_H

// -- includes -----
#include "WMFCameraEnumerator.h"
#include "WMFConfig.h"
#include "WorkerThread.h"

#include <Mfidl.h>
#include <Mfapi.h>
#include <Mferror.h>
#include <Strmif.h>
#include <Shlwapi.h>

// -- definitions -----
class WMFVideoFrameProcessor : public IMFSampleGrabberSinkCallback, public WorkerThread
{
public:
	WMFVideoFrameProcessor(const WMFDeviceFormatInfo &deviceFormat, class ITrackerListener *listener);
	~WMFVideoFrameProcessor();

	HRESULT init(IMFMediaSource *pSource);
	void dispose();
	
	HRESULT start();
	void stop();

	inline bool getIsRunning() const { return m_bIsRunning; }

protected:
	virtual bool doWork() override;

	HRESULT CreateTopology(IMFMediaSource *pSource, IMFActivate *pSinkActivate, IMFTopology **ppTopo);
	HRESULT AddSourceNode(
		IMFTopology *pTopology,           
		IMFMediaSource *pSource,          
		IMFPresentationDescriptor *pPD,   
		IMFStreamDescriptor *pSD,         
		IMFTopologyNode **ppNode);
	HRESULT AddOutputNode(
		IMFTopology *pTopology,     
		IMFActivate *pActivate,     
		DWORD dwId,                 
		IMFTopologyNode **ppNode);
	
	// IUnknown methods
	STDMETHODIMP QueryInterface(REFIID iid, void** ppv);
    STDMETHODIMP_(ULONG) AddRef();
    STDMETHODIMP_(ULONG) Release();

    // IMFClockStateSink methods
	STDMETHODIMP OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset) { return S_OK; }
    STDMETHODIMP OnClockStop(MFTIME hnsSystemTime) { return S_OK; }
    STDMETHODIMP OnClockPause(MFTIME hnsSystemTime) { return S_OK; }
    STDMETHODIMP OnClockRestart(MFTIME hnsSystemTime) { return S_OK; }
    STDMETHODIMP OnClockSetRate(MFTIME hnsSystemTime, float flRate) { return S_OK; }

    // IMFSampleGrabberSinkCallback methods
    STDMETHODIMP OnSetPresentationClock(IMFPresentationClock* pClock)  { return S_OK; }
    STDMETHODIMP OnShutdown() { return S_OK; }
    STDMETHODIMP OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
        LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE * pSampleBuffer,
        DWORD dwSampleSize);

private:
	long m_referenceCount;

	unsigned int m_deviceIndex;
	
	IMFMediaSession *m_pSession;
	IMFTopology *m_pTopology;
	
	bool m_bIsRunning;

	class ITrackerListener *m_trackerListener;
};

class WMFVideoDevice
{
public:
    WMFVideoDevice(const int device_index, const WMFDeviceInfo &device_info);
	~WMFVideoDevice();

	bool open(int desiredFormatIndex, WMFCommonTrackerConfig &cfg, class ITrackerListener *trackerListener);
	void close();

	inline const PSVRVideoPropertyConstraint *getVideoPropertyConstraints() const { return m_videoPropertyConstraints; }
	inline const WMFVideoFrameProcessor *getVideoFrameProcessorConst() const { return m_videoFrameProcessor; }
	inline WMFVideoFrameProcessor *getVideoFrameProcessor() { return m_videoFrameProcessor; }
	const WMFDeviceFormatInfo *getCurrentDeviceFormat() const;
	bool getIsOpen() const;

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
	bool setProcAmpProperty(VideoProcAmpProperty propId, long desired_value, bool bAuto);
	long getProcAmpProperty(VideoProcAmpProperty propId, bool *bIsAuto = nullptr) const;
	bool getProcAmpRange(VideoProcAmpProperty propId, PSVRVideoPropertyConstraint &constraint) const;

	/*
		https://msdn.microsoft.com/en-us/library/windows/desktop/dd318253(v=vs.85).aspx
		CameraControl_Pan			[-180, 180]
		CameraControl_Tilt			[-180, 180]
		CameraControl_Roll			[-180, 180]
		CameraControl_Zoom			[10, 600]
		CameraControl_Exposure		1/2^n seconds (example n=-3 is 1/8th seconds)
		CameraControl_Iris			units of f_stop*10
		CameraControl_Focus			 optimally focused target, in millimeters
	*/
	bool setCameraControlProperty(CameraControlProperty propId, long desired_value, bool bAuto);
	long getCameraControlProperty(CameraControlProperty propId, bool *bIsAuto = nullptr) const;
	bool getCameraControlRange(CameraControlProperty propId, PSVRVideoPropertyConstraint &constraint) const;

	bool getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const;
	void setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value);
	int getVideoProperty(const PSVRVideoPropertyType property_type) const;

public:
	int m_deviceIndex;
	WMFDeviceInfo m_deviceInfo;
	int m_deviceFormatIndex;
	PSVRVideoPropertyConstraint m_videoPropertyConstraints[PSVRVideoProperty_COUNT];
	IMFMediaSource *m_mediaSource;
	WMFVideoFrameProcessor *m_videoFrameProcessor;
};

#endif // WMF_COMMON_H
