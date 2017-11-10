#ifndef PSVR_CLIENT_H
#define PSVR_CLIENT_H

//-- includes -----
#include "PSMoveClient_CAPI.h"
#include "PSVRServiceInterface.h"
#include "Logger.h"
#include <deque>
#include <map>
#include <vector>

//-- typedefs -----
typedef std::deque<PSMMessage> t_message_queue;
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

    // -- ClientPSMoveAPI System -----
    bool startup(e_log_severity_level log_level);
    void update();
	void process_messages();
    bool poll_next_message(PSMEventMessage *message, size_t message_size);
    void shutdown();

    // -- ClientPSMoveAPI Requests -----
    bool allocate_tracker_listener(const PSMClientTrackerInfo &trackerInfo);
    void free_tracker_listener(PSMTrackerID tracker_id);
    PSMTracker* get_tracker_view(PSMTrackerID tracker_id);
	bool open_video_stream(PSMTrackerID tracker_id);
	void close_video_stream(PSMTrackerID tracker_id);
	const unsigned char *get_video_frame_buffer(PSMTrackerID tracker_id) const;

    bool allocate_hmd_listener(PSMHmdID HmdID);
    void free_hmd_listener(PSMHmdID HmdID);   
	PSMHeadMountedDisplay* get_hmd_view(PSMHmdID tracker_id);
    
protected:
    // IDataFrameListener
    virtual void handle_data_frame(const DeviceOutputDataFrame *data_frame) override;

    // INotificationListener
    virtual void handle_notification(ResponsePtr notification) override;

    // Message Helpers
    //-----------------
	void process_event_message(const PSMEventMessage *event_message);
    void enqueue_event_message(PSMEventMessage::eEventType event_type, ResponsePtr event);

private:
    //-- Request Handling -----
    class ServiceRequestHandler *m_requestManager;
    
    //-- Tracker Views -----
	PSMTracker m_trackers[PSMOVESERVICE_MAX_TRACKER_COUNT];
    
    //-- HMD Views -----
	PSMHeadMountedDisplay m_HMDs[PSMOVESERVICE_MAX_HMD_COUNT];

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