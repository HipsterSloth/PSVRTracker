#include "PSVRConfig.h"
#include "DeviceInterface.h"
#include "Logger.h"
#include "Utility.h"
#include <iostream>

// Suppress unhelpful configuru warnings
#ifdef _MSC_VER
    #pragma warning (push)
    #pragma warning (disable: 4996) // This function or variable may be unsafe
    #pragma warning (disable: 4244) // 'return': conversion from 'const int64_t' to 'float', possible loss of data
    #pragma warning (disable: 4715) // configuru::Config::operator[]': not all control paths return a value
#endif
#define CONFIGURU_IMPLEMENTATION 1
#include <configuru.hpp>
#ifdef _MSC_VER
    #pragma warning (pop)
#endif


// Format: {hue center, hue range}, {sat center, sat range}, {val center, val range}
// All hue angles are 60 degrees apart to maximize hue separation for 6 max tracked colors.
// Hue angle reference: http://i.imgur.com/PKjgfFXm.jpg 
// Hue angles divide by 2 for opencv which remaps hue range to [0,180]
const PSVR_HSVColorRange g_default_color_presets[] = {
    { { 300 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Magenta
    { { 180 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Cyan
    { { 60 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Yellow
    { { 0, 10 }, { 255, 32 }, { 255, 32 } }, // Red
    { { 120 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Green
    { { 240 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Blue
};
const PSVR_HSVColorRange *k_default_color_presets = g_default_color_presets;

PSVRConfig::PSVRConfig(const std::string &fnamebase)
: ConfigFileBase(fnamebase)
{
}

const std::string
PSVRConfig::getConfigPath()
{
    std::string home_dir= Utility::get_home_directory();  
    std::string config_path = home_dir + "/PSVRSERVICE";
    
    if (!Utility::create_directory(config_path))
    {
        PSVR_LOG_ERROR("PSVRConfig::getConfigPath") << "Failed to create config directory: " << config_path;
    }

    std::string config_filepath = config_path + "/" + ConfigFileBase + ".json";

    return config_filepath;
}

void
PSVRConfig::save()
{
    save(getConfigPath());
}

void 
PSVRConfig::save(const std::string &path)
{
	configuru::dump_file(path, writeToJSON(), configuru::JSON);
}

bool
PSVRConfig::load()
{
    return load(getConfigPath());
}

bool 
PSVRConfig::load(const std::string &path)
{
    bool bLoadedOk = false;

    if (Utility::file_exists( path ) )
    {
        configuru::Config cfg = configuru::parse_file(path, configuru::JSON);
        readFromJSON(cfg);
        bLoadedOk = true;
    }

    return bLoadedOk;
}


void PSVRConfig::writeMonoTrackerIntrinsics(
    configuru::Config &pt,
    const PSVRMonoTrackerIntrinsics &tracker_intrinsics)
{
    pt["frame_width"]= tracker_intrinsics.pixel_width;
    pt["frame_height"]= tracker_intrinsics.pixel_height;
    pt["hfov"]= tracker_intrinsics.hfov;
    pt["vfov"]= tracker_intrinsics.vfov;
    pt["zNear"]= tracker_intrinsics.znear;
    pt["zFar"]= tracker_intrinsics.zfar;

    writeMatrix3d(pt, "camera_matrix", tracker_intrinsics.camera_matrix);
    writeDistortionCoefficients(pt, "distortion_cofficients", &tracker_intrinsics.distortion_coefficients);
}

void PSVRConfig::readMonoTrackerIntrinsics(
	const configuru::Config &pt,
	PSVRMonoTrackerIntrinsics &tracker_intrinsics)
{

	tracker_intrinsics.pixel_width = pt.get_or<float>("frame_width", 640.f);
	tracker_intrinsics.pixel_height = pt.get_or<float>("frame_height", 480.f);
    tracker_intrinsics.hfov = pt.get_or<float>("hfov", 60.f);
    tracker_intrinsics.vfov = pt.get_or<float>("vfov", 45.f);
    tracker_intrinsics.znear = pt.get_or<float>("zNear", 10.f);
    tracker_intrinsics.zfar = pt.get_or<float>("zFar", 200.f);

    readMatrix3d(pt, "camera_matrix", tracker_intrinsics.camera_matrix);
    readDistortionCoefficients(pt, "distortion_cofficients", 
        &tracker_intrinsics.distortion_coefficients, 
        &tracker_intrinsics.distortion_coefficients);
}

void PSVRConfig::writeStereoTrackerIntrinsics(
    configuru::Config &pt,
    const PSVRStereoTrackerIntrinsics &tracker_intrinsics)
{
    pt["frame_width"]= tracker_intrinsics.pixel_width;
    pt["frame_height"]= tracker_intrinsics.pixel_height;
    pt["hfov"]= tracker_intrinsics.hfov;
    pt["vfov"]= tracker_intrinsics.vfov;
    pt["zNear"]= tracker_intrinsics.znear;
    pt["zFar"]= tracker_intrinsics.zfar;

    writeMatrix3d(pt, "left_camera_matrix", tracker_intrinsics.left_camera_matrix);
    writeMatrix3d(pt, "right_camera_matrix", tracker_intrinsics.right_camera_matrix);

    writeDistortionCoefficients(pt, "left_distortion_cofficients", &tracker_intrinsics.left_distortion_coefficients);
    writeDistortionCoefficients(pt, "right_distortion_cofficients", &tracker_intrinsics.right_distortion_coefficients);

    writeMatrix3d(pt, "left_rectification_rotation", tracker_intrinsics.left_rectification_rotation);
    writeMatrix3d(pt, "right_rectification_rotation", tracker_intrinsics.right_rectification_rotation);

    writeMatrix34d(pt, "left_rectification_projection", tracker_intrinsics.left_rectification_projection);
    writeMatrix34d(pt, "right_rectification_projection", tracker_intrinsics.right_rectification_projection);

    writeMatrix3d(pt, "rotation_between_cameras", tracker_intrinsics.rotation_between_cameras);
    writeVector3d(pt, "translation_between_cameras", tracker_intrinsics.translation_between_cameras);
    writeMatrix3d(pt, "essential_matrix", tracker_intrinsics.essential_matrix);
    writeMatrix3d(pt, "fundamental_matrix", tracker_intrinsics.fundamental_matrix);
    writeMatrix4d(pt, "reprojection_matrix", tracker_intrinsics.reprojection_matrix);
}

void PSVRConfig::readStereoTrackerIntrinsics(
	const configuru::Config &pt,
	PSVRStereoTrackerIntrinsics &tracker_intrinsics)
{
	tracker_intrinsics.pixel_width = pt.get_or<float>("frame_width", 640.f);
	tracker_intrinsics.pixel_height = pt.get_or<float>("frame_height", 480.f);
    tracker_intrinsics.hfov = pt.get_or<float>("hfov", 60.f);
    tracker_intrinsics.vfov = pt.get_or<float>("vfov", 45.f);
    tracker_intrinsics.znear = pt.get_or<float>("zNear", 10.f);
    tracker_intrinsics.zfar = pt.get_or<float>("zFar", 200.f);

    readMatrix3d(pt, "left_camera_matrix", tracker_intrinsics.left_camera_matrix);
    readMatrix3d(pt, "right_camera_matrix", tracker_intrinsics.right_camera_matrix);

    readDistortionCoefficients(pt, "left_distortion_cofficients", 
        &tracker_intrinsics.left_distortion_coefficients, 
        &tracker_intrinsics.left_distortion_coefficients);
    readDistortionCoefficients(pt, "right_distortion_cofficients", 
        &tracker_intrinsics.right_distortion_coefficients, 
        &tracker_intrinsics.right_distortion_coefficients);

    readMatrix3d(pt, "left_rectification_rotation", tracker_intrinsics.left_rectification_rotation);
    readMatrix3d(pt, "right_rectification_rotation", tracker_intrinsics.right_rectification_rotation);

    readMatrix34d(pt, "left_rectification_projection", tracker_intrinsics.left_rectification_projection);
    readMatrix34d(pt, "right_rectification_projection", tracker_intrinsics.right_rectification_projection);

    readMatrix3d(pt, "rotation_between_cameras", tracker_intrinsics.rotation_between_cameras);
    readVector3d(pt, "translation_between_cameras", tracker_intrinsics.translation_between_cameras);
    readMatrix3d(pt, "essential_matrix", tracker_intrinsics.essential_matrix);
    readMatrix3d(pt, "fundamental_matrix", tracker_intrinsics.fundamental_matrix);
    readMatrix4d(pt, "reprojection_matrix", tracker_intrinsics.reprojection_matrix);
}

void PSVRConfig::writeDistortionCoefficients(
    configuru::Config &pt,
    const char *coefficients_name,
    const PSVRDistortionCoefficients *coefficients)
{
    char full_property_name[256];

    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.k1", coefficients_name);
    pt[full_property_name]= coefficients->k1;
    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.k2", coefficients_name);
    pt[full_property_name]= coefficients->k2;
    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.k3", coefficients_name);
    pt[full_property_name]= coefficients->k3;
    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.p1", coefficients_name);
    pt[full_property_name]= coefficients->p1;
    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.p2", coefficients_name);
    pt[full_property_name]= coefficients->p2;
}

void PSVRConfig::readDistortionCoefficients(
    const configuru::Config &pt,
    const char *coefficients_name,
    PSVRDistortionCoefficients *outCoefficients,
    const PSVRDistortionCoefficients *defaultCoefficients)
{
    char full_property_name[256];

    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.k1", coefficients_name);
    outCoefficients->k1= pt.get_or<double>(full_property_name, defaultCoefficients->k1);
    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.k2", coefficients_name);
    outCoefficients->k2= pt.get_or<double>(full_property_name, defaultCoefficients->k2);
    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.k3", coefficients_name);
    outCoefficients->k3= pt.get_or<double>(full_property_name, defaultCoefficients->k3);
    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.p1", coefficients_name);
    outCoefficients->p1= pt.get_or<double>(full_property_name, defaultCoefficients->p1);
    Utility::format_string(full_property_name, sizeof(full_property_name), "%s.p2", coefficients_name);
    outCoefficients->p2= pt.get_or<double>(full_property_name, defaultCoefficients->p2);
}

void PSVRConfig::writeMatrix3d(
    configuru::Config &pt,
    const char *matrix_name,
    const PSVRMatrix3d &m)
{
    pt[matrix_name]= configuru::Config::array({
        m.m[0][0], m.m[0][1], m.m[0][2],
        m.m[1][0], m.m[1][1], m.m[1][2],
        m.m[2][0], m.m[2][1], m.m[2][2]});
}

void PSVRConfig::readMatrix3d(
    const configuru::Config &pt,
    const char *matrix_name,
    PSVRMatrix3d &outMatrix)
{
    if (pt[matrix_name].is_array())
    {
        int i= 0;
        int j= 0;
        for (const configuru::Config& element : pt[matrix_name].as_array()) 
        {
            outMatrix.m[j][i]= element.as_double();

            ++i;
            if (i >= 3)
            {
                i=0;
                ++j;
            }
            if (j >= 3)
                break;
        }
    }
}

void PSVRConfig::writeMatrix34d(
    configuru::Config &pt,
    const char *matrix_name,
    const PSVRMatrix34d &m)
{
    pt[matrix_name]= configuru::Config::array({
        m.m[0][0], m.m[0][1], m.m[0][2], m.m[0][3],
        m.m[1][0], m.m[1][1], m.m[1][2], m.m[1][3],
        m.m[2][0], m.m[2][1], m.m[2][2], m.m[2][3]});
}

void PSVRConfig::readMatrix34d(
    const configuru::Config &pt,
    const char *matrix_name,
    PSVRMatrix34d &outMatrix)
{
    if (pt[matrix_name].is_array())
    {
        int i= 0;
        int j= 0;
        for (const configuru::Config& element : pt[matrix_name].as_array()) 
        {
            outMatrix.m[j][i]= element.as_double();

            ++i;
            if (i >= 4)
            {
                i=0;
                ++j;
            }
            if (j >= 3)
                break;
        }
    }
}

void PSVRConfig::writeMatrix4d(
    configuru::Config &pt,
    const char *matrix_name,
    const PSVRMatrix4d &m)
{
    pt[matrix_name]= configuru::Config::array({
        m.m[0][0], m.m[0][1], m.m[0][2], m.m[0][3],
        m.m[1][0], m.m[1][1], m.m[1][2], m.m[1][3],
        m.m[2][0], m.m[2][1], m.m[2][2], m.m[2][3],
        m.m[3][0], m.m[3][1], m.m[3][2], m.m[3][3]});
}

void PSVRConfig::readMatrix4d(
    const configuru::Config &pt,
    const char *matrix_name,
    PSVRMatrix4d &outMatrix)
{
    if (pt[matrix_name].is_array())
    {
        int i= 0;
        int j= 0;
        for (const configuru::Config& element : pt[matrix_name].as_array()) 
        {
            outMatrix.m[j][i]= element.as_double();

            ++i;
            if (i >= 4)
            {
                i=0;
                ++j;
            }
            if (j >= 4)
                break;
        }
    }
}

void PSVRConfig::writeVector3d(
    configuru::Config &pt,
    const char *vector_name,
    const PSVRVector3d &v)
{
    pt[vector_name]= configuru::Config::array({v.x, v.y, v.z});
}

void PSVRConfig::readVector3d(
    const configuru::Config &pt,
    const char *vector_name,
    PSVRVector3d &outVector)
{
    if (pt[vector_name].is_array())
    {
        outVector.x= pt[vector_name][0].as_double();
        outVector.y= pt[vector_name][1].as_double();
        outVector.z= pt[vector_name][2].as_double();
    }
}

void
PSVRConfig::writeColorPropertyPresetTable(
	const PSVR_HSVColorRangeTable *table,
    configuru::Config &pt)
{
	const char *profile_name= table->table_name;

    writeColorPreset(pt, profile_name, "magenta", &table->color_presets[PSVRTrackingColorType_Magenta]);
    writeColorPreset(pt, profile_name, "cyan", &table->color_presets[PSVRTrackingColorType_Cyan]);
    writeColorPreset(pt, profile_name, "yellow", &table->color_presets[PSVRTrackingColorType_Yellow]);
    writeColorPreset(pt, profile_name, "red", &table->color_presets[PSVRTrackingColorType_Red]);
    writeColorPreset(pt, profile_name, "green", &table->color_presets[PSVRTrackingColorType_Green]);
    writeColorPreset(pt, profile_name, "blue", &table->color_presets[PSVRTrackingColorType_Blue]);
}

void
PSVRConfig::readColorPropertyPresetTable(
	const configuru::Config &pt,
	PSVR_HSVColorRangeTable *table)
{
	const char *profile_name= table->table_name;

    readColorPreset(pt, profile_name, "magenta", &table->color_presets[PSVRTrackingColorType_Magenta], &k_default_color_presets[PSVRTrackingColorType_Magenta]);
    readColorPreset(pt, profile_name, "cyan", &table->color_presets[PSVRTrackingColorType_Cyan], &k_default_color_presets[PSVRTrackingColorType_Cyan]);
    readColorPreset(pt, profile_name, "yellow", &table->color_presets[PSVRTrackingColorType_Yellow], &k_default_color_presets[PSVRTrackingColorType_Yellow]);
    readColorPreset(pt, profile_name, "red", &table->color_presets[PSVRTrackingColorType_Red], &k_default_color_presets[PSVRTrackingColorType_Red]);
    readColorPreset(pt, profile_name, "green", &table->color_presets[PSVRTrackingColorType_Green], &k_default_color_presets[PSVRTrackingColorType_Green]);
    readColorPreset(pt, profile_name, "blue", &table->color_presets[PSVRTrackingColorType_Blue], &k_default_color_presets[PSVRTrackingColorType_Blue]);
}

void
PSVRConfig::writeTrackingColor(
	configuru::Config &pt,
	int tracking_color_id)
{
	switch (tracking_color_id)
	{
	case PSVRTrackingColorType_INVALID:
		pt["tracking_color"]= "invalid";
		break;
	case PSVRTrackingColorType_Magenta:
		pt["tracking_color"]= "magenta";
		break;
	case PSVRTrackingColorType_Cyan:
		pt["tracking_color"]= "cyan";
		break;
	case PSVRTrackingColorType_Yellow:
		pt["tracking_color"]= "yellow";
		break;
	case PSVRTrackingColorType_Red:
		pt["tracking_color"]= "red";
		break;
	case PSVRTrackingColorType_Green:
		pt["tracking_color"]= "green";
		break;
	case PSVRTrackingColorType_Blue:
		pt["tracking_color"]= "blue";
		break;
	default:
		assert(false && "unreachable");
	}
}

int 
PSVRConfig::readTrackingColor(
	const configuru::Config &pt)
{
	std::string tracking_color_string = pt.get_or("tracking_color", "invalid");
	int tracking_color_id = PSVRTrackingColorType_INVALID;

	if (tracking_color_string == "magenta")
	{
		tracking_color_id = PSVRTrackingColorType_Magenta;
	}
	else if (tracking_color_string == "cyan")
	{
		tracking_color_id = PSVRTrackingColorType_Cyan;
	}
	else if (tracking_color_string == "yellow")
	{
		tracking_color_id = PSVRTrackingColorType_Yellow;
	}
	else if (tracking_color_string == "red")
	{
		tracking_color_id = PSVRTrackingColorType_Red;
	}
	else if (tracking_color_string == "green")
	{
		tracking_color_id = PSVRTrackingColorType_Green;
	}
	else if (tracking_color_string == "blue")
	{
		tracking_color_id = PSVRTrackingColorType_Blue;
	}

	return tracking_color_id;
}

void
PSVRConfig::writeColorPreset(
    configuru::Config &pt,
    const char *profile_name,
    const char *color_name,
    const PSVR_HSVColorRange *colorPreset)
{
    if (profile_name != nullptr && profile_name[0] != '\0')
    {
        pt.insert_or_assign(profile_name, {
            {"color_preset", {
                {color_name, {
                    {"hue_center", colorPreset->hue_range.center},
                    {"hue_range", colorPreset->hue_range.range},
                    {"saturation_center", colorPreset->saturation_range.center},
                    {"saturation_range", colorPreset->saturation_range.range},
                    {"value_center", colorPreset->value_range.center},
                    {"value_range", colorPreset->value_range.range}
                }},
            }},
        });
    }
    else
    {
        pt.insert_or_assign("color_preset", {
            {color_name, {
                    {"hue_center", colorPreset->hue_range.center},
                    {"hue_range", colorPreset->hue_range.range},
                    {"saturation_center", colorPreset->saturation_range.center},
                    {"saturation_range", colorPreset->saturation_range.range},
                    {"value_center", colorPreset->value_range.center},
                    {"value_range", colorPreset->value_range.range}
                },
            },
        });
    }
}

static void
readColorPropertyPreset(
    const configuru::Config &pt,
    const char *profile_name,
    const char *color_name,
    const char *property_name,
    float &out_value,
    const float default_value)
{
    if (profile_name != nullptr && profile_name[0] != '\0')
    {
        if (pt.has_key(profile_name))
        {
            const auto &profile= pt[profile_name];

            if (profile.has_key("color_preset"))
            {
                const auto &color_preset= profile["color_preset"];

                if (color_preset.has_key(color_name))
                {
                    const auto &color= color_preset[color_name];

                    if (color.has_key(property_name))
                    {
                        out_value= color.get_or<float>(property_name, default_value);
                        return;
                    }
                }
            }
        }
    }
    else
    {
        const auto &color_preset= pt["color_preset"];

        if (color_preset.has_key(color_name))
        {
            const auto &color= color_preset[color_name];

            if (color.has_key(property_name))
            {
                out_value= color.get_or<float>(property_name, default_value);
                return;
            }
        }
    }

    out_value= default_value;
    return;
}

void
PSVRConfig::readColorPreset(
    const configuru::Config &pt,
    const char *profile_name,
    const char *color_name,
    PSVR_HSVColorRange *outColorPreset,
    const PSVR_HSVColorRange *defaultPreset)
{
    readColorPropertyPreset(pt, profile_name, color_name, "hue_center", outColorPreset->hue_range.center, defaultPreset->hue_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "hue_range", outColorPreset->hue_range.range, defaultPreset->hue_range.range);
    readColorPropertyPreset(pt, profile_name, color_name, "saturation_center", outColorPreset->saturation_range.center, defaultPreset->saturation_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "saturation_range", outColorPreset->saturation_range.range, defaultPreset->saturation_range.range);
    readColorPropertyPreset(pt, profile_name, color_name, "value_center", outColorPreset->value_range.center, defaultPreset->value_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "value_range", outColorPreset->value_range.range, defaultPreset->value_range.range);
}