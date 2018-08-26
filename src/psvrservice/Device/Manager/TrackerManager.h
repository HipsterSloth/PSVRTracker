#ifndef TRACKER_MANAGER_H
#define TRACKER_MANAGER_H

//-- includes -----
#include <memory>
#include <deque>
#include <vector>

#include "DeviceTypeManager.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "PSVRConfig.h"

//-- typedefs -----
class ServerTrackerView;
typedef std::shared_ptr<ServerTrackerView> ServerTrackerViewPtr;

//-- definitions -----
class TrackerManagerConfig : public PSVRConfig
{
public:
    static const int CONFIG_VERSION;

    TrackerManagerConfig(const std::string &fnamebase = "TrackerManagerConfig");

    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

	static PSVRTrackerDebugFlags debug_flags;
	static bool are_debug_flags_enabled(PSVRTrackerDebugFlags mask) 
	{ return (debug_flags & mask) > 0; }

    long version;
	int tracker_sleep_ms;
	bool use_bgr_to_hsv_lookup_table;
	float min_valid_projection_area;
	bool disable_roi;
	float global_forward_degrees;

	PSVRVector3f get_global_forward_axis() const;
	PSVRVector3f get_global_backward_axis() const;
	PSVRVector3f get_global_right_axis() const;
	PSVRVector3f get_global_left_axis() const;
	PSVRVector3f get_global_up_axis() const;
	PSVRVector3f get_global_down_axis() const;
};

class TrackerManager : public DeviceTypeManager
{
public:
    TrackerManager();

    bool startup() override;

	void pollUpdatedVideoFrames();
    void closeAllTrackers();

    static const int k_max_devices = PSVRSERVICE_MAX_TRACKER_COUNT;
    int getMaxDevices() const override
    {
        return TrackerManager::k_max_devices;
    }

    ServerTrackerViewPtr getTrackerViewPtr(int device_id) const;

    inline const TrackerManagerConfig& getConfig() const
    {
        return cfg;
    }

    PSVRTrackingColorType allocateTrackingColorID();
    bool claimTrackingColorID(const class ServerControllerView *controller_view, PSVRTrackingColorType color_id);
    bool claimTrackingColorID(const class ServerHMDView *hmd_view, PSVRTrackingColorType color_id);
    void freeTrackingColorID(PSVRTrackingColorType color_id);

protected:
    bool can_update_connected_devices() override;
    void mark_tracker_list_dirty();

    DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
	int getListUpdatedResponseType() override;

private:
	class TrackerCapabilitiesSet *m_supportedTrackers;
	std::deque<PSVRTrackingColorType> m_available_color_ids;
    TrackerManagerConfig cfg;
    bool m_tracker_list_dirty;
};

#endif // TRACKER_MANAGER_H