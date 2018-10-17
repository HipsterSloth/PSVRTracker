#include "PS3EyeConfig.h"
#include "TrackerCapabilitiesConfig.h"
#include "PSVRClient_CAPI.h"

PS3EyeTrackerConfig::PS3EyeTrackerConfig(const std::string &fnamebase)
    : CommonTrackerConfig(fnamebase)
	, ps3eye_video_mode_index(-1)
    , fovSetting(BlueDot)
{
	CommonTrackerConfig::current_mode= "640x480(60FPS)";

	memset(video_properties, 0, sizeof(video_properties));
	video_properties[PSVRVideoProperty_Gain]= 20;
	video_properties[PSVRVideoProperty_Exposure]= 120;
	video_properties[PSVRVideoProperty_Sharpness]= 0;
	video_properties[PSVRVideoProperty_Hue]= 143;
	video_properties[PSVRVideoProperty_WhiteBalance]= 0;
	video_properties[PSVRVideoProperty_Brightness]= 20;
	video_properties[PSVRVideoProperty_Contrast]= 37;
	video_properties[PSVRVideoProperty_BlueBalance]= 128;
	video_properties[PSVRVideoProperty_RedBalance]= 128;
	video_properties[PSVRVideoProperty_GreenBalance]= 128;

	flip_horizontal= true;
	flip_vertical= false;

	trackerIntrinsics.pixel_width= 640.f;
	trackerIntrinsics.pixel_height= 480.f;
    trackerIntrinsics.camera_matrix= {
        554.2563, 0.0, 320.0, 
        0.0, 554.2563, 240.0,
        0.0, 0.0, 1.0}; // pixels
    trackerIntrinsics.hfov= 60.0; // degrees
    trackerIntrinsics.vfov= 45.0; // degrees
    trackerIntrinsics.znear= 10.0; // cm
    trackerIntrinsics.zfar= 200.0; // cm
    trackerIntrinsics.distortion_coefficients.k1= -0.10771770030260086;
    trackerIntrinsics.distortion_coefficients.k2= 0.1213262677192688;
    trackerIntrinsics.distortion_coefficients.k3= 0.04875476285815239;
    trackerIntrinsics.distortion_coefficients.p1= 0.00091733073350042105;
    trackerIntrinsics.distortion_coefficients.p2= 0.00010589254816295579;
};

const configuru::Config
PS3EyeTrackerConfig::writeToJSON()
{
    configuru::Config pt= CommonTrackerConfig::writeToJSON();

	pt["fovSetting"]= static_cast<int>(fovSetting);
	pt["ps3eye_video_mode_index"]= ps3eye_video_mode_index;
	pt["flip_horizontal"]= flip_horizontal;
	pt["flip_vertical"]= flip_vertical;

    writeMatrix3d(pt, "camera_matrix", trackerIntrinsics.camera_matrix);
    writeDistortionCoefficients(pt, "distortion", &trackerIntrinsics.distortion_coefficients);

    return pt;
}

void
PS3EyeTrackerConfig::readFromJSON(const configuru::Config &pt)
{
	CommonTrackerConfig::readFromJSON(pt);

    trackerIntrinsics.hfov = pt.get_or<float>("hfov", 60.0f);
    trackerIntrinsics.vfov = pt.get_or<float>("vfov", 45.0f);
    trackerIntrinsics.znear = pt.get_or<float>("zNear", 10.0f);
    trackerIntrinsics.zfar = pt.get_or<float>("zFar", 200.0f);
	trackerIntrinsics.pixel_width = pt.get_or<float>("frame_width", 640.f);
	trackerIntrinsics.pixel_height = pt.get_or<float>("frame_height", 480.f);

    readMatrix3d(pt, "camera_matrix", trackerIntrinsics.camera_matrix);
    readDistortionCoefficients(pt, "distortion", 
        &trackerIntrinsics.distortion_coefficients, 
        &trackerIntrinsics.distortion_coefficients);

	ps3eye_video_mode_index= pt.get_or<int>("ps3eye_video_mode_index", ps3eye_video_mode_index);
	flip_horizontal= pt.get_or<bool>("flip_horizontal", flip_horizontal);
	flip_vertical= pt.get_or<bool>("flip_vertical", flip_horizontal);
	fovSetting = 
		static_cast<PS3EyeTrackerConfig::eFOVSetting>(
			pt.get_or<int>("fovSetting", PS3EyeTrackerConfig::eFOVSetting::BlueDot));
}
