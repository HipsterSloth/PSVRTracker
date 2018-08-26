#ifndef PSVR_SERVICE_INTERFACE_H
#define PSVR_SERVICE_INTERFACE_H

//-- includes -----
#include "PSVRClient_CAPI.h"
#include <atomic>

//-- definitions -----
enum DeviceCategory
{
	DeviceCategory_CONTROLLER= 0,
	DeviceCategory_TRACKER= 1,
	DeviceCategory_HMD= 2
};

struct ControllerDataPacket
{
    PSVRControllerID controller_id;
    PSVRControllerType controller_type;
    union
    {
        PSVRPSMove  psmove_state;
        PSVRDualShock4 ds4_state;
    }               controller_state;
    bool            is_valid;
    int             output_sequence_num;
    bool            is_connected;
};	

struct TrackerDataPacket
{
	PSVRTrackerID tracker_id; 
	PSVRTrackerType tracker_type;
    int sequence_num;
	bool is_connected;
};
	
struct HMDDataPacket
{
    PSVRHmdID hmd_id;
    PSVRHmdType hmd_type;
    union
    {
        PSVRMorpheus  morpheus_state;
        PSVRVirtualHMD virtual_hmd_state;
    }               hmd_state;
    bool            is_valid;
    int             output_sequence_num;
    bool            is_connected;
};	
	
struct DeviceOutputDataFrame
{
	union{
		ControllerDataPacket controller_data_packet;
		TrackerDataPacket tracker_data_packet;
		HMDDataPacket hmd_data_packet;
	} device;
	DeviceCategory device_category;
};

class SharedVideoFrameBuffer
{
public:
    SharedVideoFrameBuffer();
    ~SharedVideoFrameBuffer();

    bool initialize(const char *buffer_name, int width, int height, int stride, int section_count);
    void dispose();
    void writeVideoFrame(PSVRVideoFrameSection section, const unsigned char *buffer);
	void finalizeVideoFrameWrite();
    inline int getSectionCount() const { return m_section_count; }
	inline int getFrameIndex() const { return m_frame_index; } 
    const unsigned char *getBuffer(PSVRVideoFrameSection section) const;
    unsigned char *getBufferMutable(PSVRVideoFrameSection section);
    static size_t computeVideoBufferSize(int section_count, int stride, int height);

private:
    std::string m_buffer_name;
	int m_width;
    int m_height;
    int m_stride;
    int m_section_count;
	unsigned char *m_buffer[2];
	std::atomic_int m_frame_index;
};

//-- interface -----
class INotificationListener
{
public:
    virtual void handle_notification(const PSVREventMessage &response) = 0;
};

class IDataFrameListener
{
public:
    virtual void handle_data_frame(const DeviceOutputDataFrame &data_frame) = 0;
};

#endif  // PSVRPROTOCOL_INTERFACE_H
