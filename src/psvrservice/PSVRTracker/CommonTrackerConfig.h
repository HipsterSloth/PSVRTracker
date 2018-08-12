#ifndef COMMON_TRACKER_CONFIG_H
#define COMMON_TRACKER_CONFIG_H

// -- includes -----
#include "ClientColor_CAPI.h"
#include "PSVRConfig.h"

// -- definitions -----
class CommonTrackerConfig : public PSVRConfig
{
public:
    CommonTrackerConfig(const std::string &fnamebase = "CommonTrackerConfig");
    
    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

	const PSVR_HSVColorRangeTable *getColorRangeTable(const std::string &table_name) const;
	PSVR_HSVColorRangeTable *getOrAddColorRangeTable(const std::string &table_name);

    bool is_valid;
    long max_poll_failure_count;

	std::string current_mode;
	int video_properties[PSVRVideoProperty_COUNT];

    PSVRPosef pose;
	PSVR_HSVColorRangeTable SharedColorPresets;
	std::vector<PSVR_HSVColorRangeTable> DeviceColorPresets;
};

#endif // WMF_CONFIG_H
