// -- includes -----
#include "WMFConfig.h"
#include "TrackerCapabilitiesConfig.h"
#include "PSVRClient_CAPI.h"

// -- WMF Stereo Tracker Config
WMFCommonTrackerConfig::WMFCommonTrackerConfig(const std::string &fnamebase)
    : CommonTrackerConfig(fnamebase)
	, wmf_video_format_index(-1)
{
};

const configuru::Config 
WMFCommonTrackerConfig::writeToJSON()
{
	configuru::Config pt = CommonTrackerConfig::writeToJSON();

	pt["wmf_video_format_index"]= wmf_video_format_index;

    return pt;
}

void 
WMFCommonTrackerConfig::readFromJSON(const configuru::Config &pt)
{
	CommonTrackerConfig::readFromJSON(pt);
	wmf_video_format_index= pt.get_or<int>("wmf_video_format_index", wmf_video_format_index);
}