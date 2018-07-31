#ifndef WMF_CONFIG_H
#define WMF_CONFIG_H

// -- includes -----
#include "ClientColor_CAPI.h"
#include "PSVRConfig.h"

// -- definitions -----
class WMFCommonTrackerConfig : public PSVRConfig
{
public:
    WMFCommonTrackerConfig(const std::string &fnamebase = "WMFCommonTrackerConfig");
    
    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

	const PSVR_HSVColorRangeTable *getColorRangeTable(const std::string &table_name) const;
	PSVR_HSVColorRangeTable *getOrAddColorRangeTable(const std::string &table_name);

    bool is_valid;
    long max_poll_failure_count;

	std::string current_mode;
	int wmf_video_format_index;
	int video_properties[PSVRVideoProperty_COUNT];

    PSVRPosef pose;
	PSVR_HSVColorRangeTable SharedColorPresets;
	std::vector<PSVR_HSVColorRangeTable> DeviceColorPresets;
};

#endif // WMF_CONFIG_H
