#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

//-- includes -----
#include <memory>
#include <string>
#include <vector>
#include "DeviceTypeManager.h"
#include "DeviceEnumerator.h"
#include "PSVRConfig.h"

//-- typedefs -----
class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;
class TrackerManager;

//-- definitions -----
class ControllerManagerConfig : public PSVRConfig
{
public:
    static const int CONFIG_VERSION;

    ControllerManagerConfig(const std::string &fnamebase = "ControllerManagerConfig");

    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

    int version;
};

class ControllerManager : public DeviceTypeManager
{
public:
    ControllerManager();

    virtual bool startup() override;
    virtual void shutdown() override;

    static const int k_max_devices = PSVRSERVICE_MAX_CONTROLLER_COUNT;
    int getMaxDevices() const override
    {
        return ControllerManager::k_max_devices;
    }

    ServerControllerViewPtr getControllerViewPtr(int device_id);

    inline const ControllerManagerConfig& getConfig() const
    {
        return cfg;
    }

	// Broadcasts new video frame from a single tracker on tracker thread
	void notifyVideoFrameReceived(class ServerTrackerView* tracker_view);

	// Update Pose Filter using update packets from the tracker and IMU threads
	void updatePoseFilters();

protected:
    bool can_update_connected_devices() override;
    class DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(class DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
    int getListUpdatedResponseType() override;

private:
	std::string m_bluetooth_host_address;
    ControllerManagerConfig cfg;
};

#endif // CONTROLLER_MANAGER_H
