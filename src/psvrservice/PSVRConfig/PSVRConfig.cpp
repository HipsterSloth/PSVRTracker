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
    configuru::dump_file(getConfigPath(), writeToJSON(), configuru::JSON);
}

bool
PSVRConfig::load()
{
    bool bLoadedOk = false;
    std::string configPath = getConfigPath();

    if (Utility::file_exists( configPath ) )
    {
        configuru::Config cfg = configuru::parse_file("input.json", configuru::JSON);
        readFromJSON(cfg);
        bLoadedOk = true;
    }

    return bLoadedOk;
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
                {"hue_center", colorPreset->hue_range.center},
                {"hue_range", colorPreset->hue_range.range},
                {"saturation_center", colorPreset->saturation_range.center},
                {"saturation_range", colorPreset->saturation_range.range},
                {"value_center", colorPreset->value_range.center},
                {"value_range", colorPreset->value_range.range}
            }},
        });
    }
    else
    {
        pt.insert_or_assign("color_preset", {
            {
                {"hue_center", colorPreset->hue_range.center},
                {"hue_range", colorPreset->hue_range.range},
                {"saturation_center", colorPreset->saturation_range.center},
                {"saturation_range", colorPreset->saturation_range.range},
                {"value_center", colorPreset->value_range.center},
                {"value_range", colorPreset->value_range.range}
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
    configuru::Config profile= pt;

    if (profile_name != nullptr && profile_name[0] != '\0')
    {
        if (pt.has_key(profile_name))
        {
            profile= pt.get_or<configuru::Config>(profile_name);
        }
    }

    if (profile.has_key("color_preset"))
    {
        const configuru::Config color_preset= profile.get<configuru::Config>("color_preset");

        if (color_preset.has_key(color_name))
        {
            const configuru::Config color= color_preset.get<configuru::Config>(color_name);

            if (color.has_key(property_name))
            {
                out_value= color.get<float>(property_name);
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