#ifndef PSVR_SERVICE_INTERFACE_H
#define PSVR_SERVICE_INTERFACE_H

//-- includes -----
#include "PSVRClient_CAPI.h"
#include <atomic>
#include <mutex>

//-- definitions -----
enum DeviceCategory
{
	DeviceCategory_CONTROLLER= 0,
	DeviceCategory_TRACKER= 1,
	DeviceCategory_HMD= 2
};

struct ControllerOutputDataPacket
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

struct TrackerOutputDataPacket
{
	PSVRTrackerID tracker_id; 
	PSVRTrackerType tracker_type;
    int sequence_num;
	bool is_connected;
};
	
struct HMDOutputDataPacket
{
    PSVRHmdID hmd_id;
    PSVRHmdType hmd_type;
    union
    {
        PSVRMorpheus  morpheus_state;
    }               hmd_state;
    bool            is_valid;
    int             output_sequence_num;
    bool            is_connected;
};	
	
struct DeviceOutputDataFrame
{
	union{
		ControllerOutputDataPacket controller_data_packet;
		TrackerOutputDataPacket tracker_data_packet;
		HMDOutputDataPacket hmd_data_packet;
	} device;
	DeviceCategory device_category;
};

struct ControllerInputDataPacket
{
    PSVRControllerID controller_id;
    PSVRControllerType controller_type;
    int input_sequence_num;
    union
    {
        PSVRPSMoveInput      psmove_state;
        PSVRDualShock4Input  ds4_state;
    } controller_state;
};

struct DeviceInputDataFrame
{
	union{
		ControllerInputDataPacket controller_data_packet;
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
    void writeMonoVideoFrame(const unsigned char *buffer);
	void writeStereoVideoFrame(const unsigned char *left_buffer, const unsigned char *right_buffer);
    inline int getSectionCount() const { return m_section_count; }
	inline int getFrameIndex() const { return m_read_thread_frame_index; } 
    const unsigned char *fetchBufferSection(PSVRVideoFrameSection section);
    static size_t computeVideoBufferSize(int section_count, int stride, int height);

private:
    std::string m_buffer_name;
	int m_width;
    int m_height;
    int m_stride;
    int m_section_count;
	unsigned char *m_write_thread_buffer;
	unsigned char *m_read_thread_buffer;
	std::atomic_int m_write_thread_frame_index;
	int m_read_thread_frame_index;
	std::mutex m_buffer_mutex;
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
