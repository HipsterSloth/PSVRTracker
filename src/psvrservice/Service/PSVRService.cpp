//-- includes -----
#include "PSVRService.h"
#include "ServerNetworkManager.h"
#include "ServerRequestHandler.h"
#include "DeviceManager.h"
#include "Version.h"
#include "ServerLog.h"
#include "SharedTrackerState.h"
#include "TrackerManager.h"
#include "USBDeviceManager.h"

#include <fstream>
#include <cstdio>
#include <string>
#include <chrono>

//-- statics -----
PSVRService *PSVRService::m_instance= nullptr;

//-- definitions -----
PSVRService::PSVRService()
	: m_usb_device_manager(nullptr)
	, m_device_manager(nullptr)
	, m_request_handler(nullptr)
{
	PSVRService::m_instance= this;
	
    // Manages all control and bulk transfer requests in another thread
    m_usb_device_manager= new USBDeviceManager;

    // Keep track of currently connected devices (PSVR controllers, cameras, HMDs)
    m_device_manager= new DeviceManager();

    // Generates responses from incoming requests sent to the network manager
    m_request_handler= new ServiceRequestHandler(m_device_manager);
	
}

PSVRService::~PSVRService()
{
    delete m_usb_device_manager;
    delete m_device_manager;
    delete m_request_handler;
	
	PSVRService::m_instance= nullptr;
}

bool PSVRService::startup(PSVRLogSeverityLevel log_level)
{
	bool success= true;
   
	// initialize logging system
	log_init(log_level, "PSVRSERVICE.log");

	// Start the service app
	SERVER_LOG_INFO("main") << "Starting PSVRService v" << PSVR_RELEASE_VERSION_STRING << " (protocol v" << PSVR_PROTOCOL_VERSION_STRING << ")";	   
   
	/** Setup the usb async transfer thread before we attempt to initialize the trackers */
	if (success)
	{
		if (!m_usb_device_manager->startup())
		{
			SERVER_LOG_FATAL("PSVRService") << "Failed to initialize the usb async request manager";
			success = false;
		}
	}

	/** Setup the device manager */
	if (success)
	{
		if (!m_device_manager->startup())
		{
			SERVER_LOG_FATAL("PSVRService") << "Failed to initialize the device manager";
			success= false;
		}
	}

	/** Setup the request handler */
	if (success)
	{
		if (!m_request_handler->startup())
		{
			SERVER_LOG_FATAL("PSVRService") << "Failed to initialize the service request handler";
			success= false;
		}
	}

	return success;
}

void PSVRService::update()
{
	// Update an async requests still waiting to complete
	m_request_handler->update();

	// Process any async results from the USB transfer thread
	m_usb_device_manager->update();

	// Update the list of active tracked devices
	// Send device updates to the client
	m_device_manager->update();
}

void PSVRService::shutdown()
{
	SERVER_LOG_INFO("main") << "Shutting down PSVRService";
	
	// Kill any pending request state
	m_request_handler->shutdown();

	// Disconnect any actively connected controllers
	m_device_manager->shutdown();

	// Shutdown the usb async request thread
	// Must be after device manager since devices can have an active usb connection
	m_usb_device_manager->shutdown();
	
	log_dispose();		
}