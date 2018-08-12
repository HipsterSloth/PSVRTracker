#ifndef PS3EYE_CONFIG_H
#define PS3EYE_CONFIG_H

// -- includes -----
#include "ClientColor_CAPI.h"
#include "CommonTrackerConfig.h"

// -- definitions -----
class PS3EyeTrackerConfig : public CommonTrackerConfig
{
public:
    enum eFOVSetting
    {
        RedDot, // 56 degree FOV
        BlueDot, // 75 degree FOV
        
        MAX_FOV_SETTINGS
    };

    PS3EyeTrackerConfig(const std::string &fnamebase = "PS3EyeTrackerConfig");
    
    virtual const configuru::Config writeToJSON() override;
    virtual void readFromJSON(const configuru::Config &pt) override;

	inline int getExposure() const { return video_properties[PSVRVideoProperty_Exposure]; }
	inline int getGain() const { return video_properties[PSVRVideoProperty_Gain]; }

	inline void setExposure(int exposure) { video_properties[PSVRVideoProperty_Exposure]= exposure; }
	inline void setGain(int gain) { video_properties[PSVRVideoProperty_Gain]= gain; }

	int ps3eye_video_mode_index;
    eFOVSetting fovSetting;    
    PSVRMonoTrackerIntrinsics trackerIntrinsics;
};

#endif // PS3EYE_CONFIG_H
