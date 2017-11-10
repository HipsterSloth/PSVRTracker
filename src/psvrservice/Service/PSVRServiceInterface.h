#ifndef PSVR_SERVICE_INTERFACE_H
#define PSVR_SERVICE_INTERFACE_H

//-- includes -----
#include "PSVRClient_CAPI.h"

//-- definitions -----
enum DeviceCategory
{
	DeviceCategory_TRACKER= 0;
	DeviceCategory_HMD= 1;
}

struct TrackerDataPacket
{
	PSMTrackerID tracker_id; 
	PSMTrackerType tracker_type;
    int sequence_num;
	bool is_connected;
};
	
struct HMDDataPacket
{
    PSMHmdID hmd_id;
    PSMHmdType hmd_type;
    union
    {
        PSMMorpheus  morpheus_state;
        PSMVirtualHMD virtual_hmd_state;
    }               hmd_state;
    bool            is_valid;
    int             output_sequence_num;
    bool            is_connected;
};	
	
struct DeviceOutputDataFrame
{
	union{
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

    bool initialize(const char *buffer_name, int width, int height, int stride);
    void dispose();
    void writeVideoFrame(const unsigned char *buffer);
    const unsigned char *getBuffer() const;
    unsigned char *getBufferMutable();
    static size_t computeVideoBufferSize(int stride, int height);

private:
    std::string m_shared_memory_name;
	int m_width;
    int m_height;
    int m_stride;
	unsigned char *m_buffer;
	int m_frame_index;	
};

//-- interface -----
class INotificationListener
{
public:
    virtual void handle_notification(const PSMEventMessage &response) = 0;
};

class IDataFrameListener
{
public:
    virtual void handle_data_frame(const DeviceOutputDataFrame &data_frame) = 0;
};

#endif  // PSMOVEPROTOCOL_INTERFACE_H
