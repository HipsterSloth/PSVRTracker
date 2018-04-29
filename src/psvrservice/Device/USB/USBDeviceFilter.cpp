// -- include -----
#include "USBDeviceFilter.h"

bool USBDeviceFilterSet::addFilter(unsigned short vendor_id, unsigned short product_id)
{
	return addFilter(vendor_id, product_id, ANY_INTERFACE_MASK);
}

bool USBDeviceFilterSet::addFilter(unsigned short vendor_id, unsigned short product_id, unsigned int interface_mask)
{
	if (!containsFilter(vendor_id, product_id, interface_mask))
	{
		filters.push_back({vendor_id, product_id, interface_mask});
		return true;
	}
	else
	{
		return false;
	}
}

bool USBDeviceFilterSet::containsFilter(unsigned short vendor_id, unsigned short product_id, unsigned int interface_mask)
{
	bool bContains= false;

	for (const USBDeviceFilter &filter : filters)
	{
		if (filter.vendor_id == vendor_id && filter.product_id == product_id && filter.interface_mask == interface_mask)
		{
			bContains= true;
			break;
		}
	}

	return bContains;
}

bool USBDeviceFilterSet::passesFilter(unsigned short vendor_id, unsigned short product_id)
{
	return passesFilter(vendor_id, product_id, ANY_INTERFACE_MASK);
}

bool USBDeviceFilterSet::passesFilter(unsigned short vendor_id, unsigned short product_id, unsigned int interface_mask)
{
	bool bPasses= false;

	for (const USBDeviceFilter &filter : filters)
	{
		if (filter.vendor_id == vendor_id 
			&& filter.product_id == product_id 
			&& (filter.interface_mask & interface_mask) > 0)
		{
			bPasses= true;
			break;
		}
	}

	return bPasses;	
}
