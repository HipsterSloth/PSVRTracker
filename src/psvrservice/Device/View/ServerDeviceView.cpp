//-- includes -----
#include "ServerDeviceView.h"
#include "Logger.h"

#include <chrono>

//-- private methods -----

//-- public implementation -----
ServerDeviceView::ServerDeviceView(
    const int device_id)
    : m_bHasUnpublishedState(false)
    , m_pollNoDataCount(0)
    , m_sequence_number(0)
    , m_deviceID(device_id)
{
}

ServerDeviceView::~ServerDeviceView()
{
}

bool
ServerDeviceView::open(const DeviceEnumerator *enumerator)
{
    // Attempt to allocate the device 
    bool bSuccess= allocate_device_interface(enumerator);
    
    // Attempt to open the device
    if (bSuccess)
    {
        bSuccess= getDevice()->open(enumerator);
    }
    
    if (bSuccess)
    {
        // Consider a successful opening as an update
        m_pollNoDataCount= 0;
    }

    return bSuccess;
}

bool
ServerDeviceView::getIsOpen() const
{
    IDeviceInterface* device= getDevice();

    return (device != nullptr) ? device->getIsOpen() : false;
}

void ServerDeviceView::publish()
{
    if (m_bHasUnpublishedState)
    {
        publish_device_data_frame();

        m_bHasUnpublishedState= false;
        m_sequence_number++;
    }
}

void
ServerDeviceView::close()
{
    if (getIsOpen())
    {
        getDevice()->close();
        free_device_interface();
    }
}

bool
ServerDeviceView::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    return getIsOpen() && getDevice()->matchesDeviceEnumerator(enumerator);
}