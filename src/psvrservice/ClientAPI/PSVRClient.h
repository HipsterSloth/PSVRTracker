#ifndef PSVR_CLIENT_H
#define PSVR_CLIENT_H

//-- includes -----
#include "PSVRClient_CAPI.h"
#include "PSVRServiceInterface.h"
#include "Logger.h"
#include <deque>
#include <map>
#include <vector>

//-- typedefs -----
typedef std::deque<PSVRMessage> t_message_queue;
typedef std::vector<ResponsePtr> t_event_reference_cache;

//-- definitions -----
class PSVRClient : 
    public IDataFrameListener,
    public INotificationListener
{
public:
    PSVRClient(class ServiceRequestHandler * request_handler);
    virtual ~PSVRClient();

	// -- State Queries ----
	inline bool getIsConnected() const { return m_bIsConnected; }
	bool pollHasTrackerListChanged();
	bool pollHasHMDListChanged();

    // -- ClientPSVRAPI System -----
    bool startup(e_log_severity_level log_level);
    void update();
	void process_messages();
    bool poll_next_message(PSVREventMessage *message, size_t message_size);
    void shutdown();

    // -- ClientPSVRAPI Requests -----
    bool allocate_tracker_listener(const PSVRClientTrackerInfo &trackerInfo);
    void free_tracker_listener(PSVRTrackerID tracker_id);
    PSVRTracker* get_tracker_view(PSVRTrackerID tracker_id);
	bool open_video_stream(PSVRTrackerID tracker_id);
	void close_video_stream(PSVRTrackerID tracker_id);
	const unsigned char *get_video_frame_buffer(PSVRTrackerID tracker_id) const;

    bool allocate_hmd_listener(PSVRHmdID HmdID);
    void free_hmd_listener(PSVRHmdID HmdID);   
	PSVRHeadMountedDisplay* get_hmd_view(PSVRHmdID tracker_id);
    
protected:
    // IDataFrameListener
    virtual void handle_data_frame(const DeviceOutputDataFrame *data_frame) override;

    // INotificationListener
    virtual void handle_notification(ResponsePtr notification) override;

    // Message Helpers
    //-----------------
	void process_event_message(const PSVREventMessage *event_message);
    void enqueue_event_message(PSVREventMessage::eEventType event_type, ResponsePtr event);

private:
    //-- Request Handling -----
    class ServiceRequestHandler *m_requestManager;
    
    //-- Tracker Views -----
	PSVRTracker m_trackers[PSVRSERVICE_MAX_TRACKER_COUNT];
    
    //-- HMD Views -----
	PSVRHeadMountedDisplay m_HMDs[PSVRSERVICE_MAX_HMD_COUNT];

	bool m_bHasTrackerListChanged;
	bool m_bHasHMDListChanged;

    //-- Messages -----
    // Queue of message received from the most recent call to update()
    // This queue will be emptied automatically at the next call to update().
    t_message_queue m_message_queue;

    // These vectors are used solely to keep the ref counted pointers to the 
    // response and event parameter data valid until the next update call.
    // The message queue contains raw void pointers to the response and event data.
    t_event_reference_cache m_event_reference_cache;
};


#endif // PSVR_CLIENT_H