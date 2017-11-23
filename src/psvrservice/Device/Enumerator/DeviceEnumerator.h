#ifndef DEVICE_ENUMERATOR_H
#define DEVICE_ENUMERATOR_H

// -- includes -----
#include "DeviceInterface.h"
#include "stdlib.h" // size_t

// -- definitions -----
class DeviceEnumerator
{
public:
    DeviceEnumerator() 
		: m_deviceType(CommonSensorState::INVALID_DEVICE_TYPE)
		, m_deviceTypeFilter(CommonSensorState::INVALID_DEVICE_TYPE)
    { }
    DeviceEnumerator(CommonSensorState::eDeviceType deviceTypeFilter)
		: m_deviceType(CommonSensorState::INVALID_DEVICE_TYPE)
		, m_deviceTypeFilter(deviceTypeFilter)
    { }
    virtual ~DeviceEnumerator() {}

    virtual bool is_valid() const =0;
    virtual bool next()=0;
	virtual int get_vendor_id() const =0;
	virtual int get_product_id() const =0;
    virtual const char *get_path() const =0;
    
    inline CommonSensorState::eDeviceType get_device_type() const
    {
        return m_deviceType;
    }

    inline CommonSensorState::eDeviceType get_device_type_filter() const
    {
        return m_deviceTypeFilter;
    }

protected:
    CommonSensorState::eDeviceType m_deviceType;
	CommonSensorState::eDeviceType m_deviceTypeFilter;
};

#endif // DEVICE_ENUMERATOR_H