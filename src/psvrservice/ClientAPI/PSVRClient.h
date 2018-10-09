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
typedef std::deque<PSVREventMessage> t_message_queue;

//-- definitions -----
class PSVRClient : 
    public IDataFrameListener,
    public INotificationListener
{
public:
    PSVRClient();
    virtual ~PSVRClient();

	// -- State Queries ----
	bool pollHasControllerListChanged();
	bool pollHasTrackerListChanged();
	bool pollHasHMDListChanged();

    // -- Client PSVR API System -----
    bool startup(PSVRLogSeverityLevel log_level, class ServiceRequestHandler * request_handler);
    void update();
	void process_messages();
    bool poll_next_message(PSVREventMessage *message, size_t message_size);
    void shutdown();

    // -- Client PSVR API Requests -----
    bool allocate_controller_listener(PSVRControllerID controller_id);
    void free_controller_listener(PSVRControllerID controller_id);   
    PSVRController* get_controller_view(PSVRControllerID controller_id);

    bool allocate_tracker_listener(const PSVRClientTrackerInfo &trackerInfo);
    void free_tracker_listener(PSVRTrackerID tracker_id);
    PSVRTracker* get_tracker_view(PSVRTrackerID tracker_id);
	bool open_video_stream(PSVRTrackerID tracker_id);
	void close_video_stream(PSVRTrackerID tracker_id);
    int get_video_frame_section_count(PSVRTrackerID tracker_id) const;
	const unsigned char *get_video_frame_buffer(PSVRTrackerID tracker_id, PSVRVideoFrameSection section) const;

    bool allocate_hmd_listener(PSVRHmdID HmdID);
    void free_hmd_listener(PSVRHmdID HmdID);   
	PSVRHeadMountedDisplay* get_hmd_view(PSVRHmdID tracker_id);
    
protected:
	void publish();

    // IDataFrameListener
    virtual void handle_data_frame(const DeviceOutputDataFrame &data_frame) override;

    // INotificationListener
    virtual void handle_notification(const PSVREventMessage &response) override;

    // Message Helpers
    //-----------------
	void process_event_message(const PSVREventMessage *event_message);

private:
    //-- Request Handling -----
    class ServiceRequestHandler *m_requestHandler;

    //-- Controller Views -----
	PSVRController m_controllers[PSVRSERVICE_MAX_CONTROLLER_COUNT];

    //-- Tracker Views -----
	PSVRTracker m_trackers[PSVRSERVICE_MAX_TRACKER_COUNT];
    
    //-- HMD Views -----
	PSVRHeadMountedDisplay m_HMDs[PSVRSERVICE_MAX_HMD_COUNT];

	bool m_bHasControllerListChanged;
	bool m_bHasTrackerListChanged;
	bool m_bHasHMDListChanged;

    //-- Messages -----
    // Queue of message received from the most recent call to update()
    // This queue will be emptied automatically at the next call to update().
    t_message_queue m_message_queue;
};


#endif // PSVR_CLIENT_H