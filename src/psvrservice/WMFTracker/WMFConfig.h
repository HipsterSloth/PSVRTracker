#ifndef WMF_CONFIG_H
#define WMF_CONFIG_H

// -- includes -----
#include "ClientColor_CAPI.h"
#include "CommonTrackerConfig.h"

// -- definitions -----
class WMFCommonTrackerConfig : public CommonTrackerConfig
{
public:
    WMFCommonTrackerConfig(const std::string &fnamebase = "WMFCommonTrackerConfig");
    
    virtual const configuru::Config writeToJSON() override;
    virtual void readFromJSON(const configuru::Config &pt) override;

	int wmf_video_format_index;
};

#endif // WMF_CONFIG_H
