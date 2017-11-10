#ifndef PSMOVE_SERVICE_H
#define PSMOVE_SERVICE_H

//-- includes -----
#include "ClientConstants.h"
#include "Logger.h"
#include <string>

//-- definitions -----
class PSVRService
{
public:
    PSVRService();
    virtual ~PSVRService();

    static PSVRService *getInstance()
    { return m_instance; }

	bool startup(PSVRLogSeverityLevel log_level);
	void update();
	void shutdown();
	
	inline bool getIsInitialized() const { return m_isInitialized; }
	inline class ServiceRequestHandler * getRequestHandler() const { return m_request_handler; }

private:
    // Singleton instance of the class
    static PSVRService *m_instance;
	
    // Manages all control and bulk transfer requests in another thread
    class USBDeviceManager *m_usb_device_manager;

    // Keep track of currently connected devices (cameras, HMDs)
    class DeviceManager *m_device_manager;

    // Generates responses from incoming requests sent from the client API
    class ServiceRequestHandler *m_request_handler;	
	
	bool m_isInitialized;
};

#endif // PSMOVE_SERVICE_H
