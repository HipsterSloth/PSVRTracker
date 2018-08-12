// -- includes -----
#include "CommonTrackerConfig.h"
#include "TrackerCapabilitiesConfig.h"
#include "PSVRClient_CAPI.h"

// -- constants -----
// -- private definitions -----
// -- public methods

// -- WMF Stereo Tracker Config
CommonTrackerConfig::CommonTrackerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
    , is_valid(false)
    , max_poll_failure_count(100)
	, current_mode("")
{
    pose= *k_PSVR_pose_identity;

	SharedColorPresets.table_name[0]= 0;
    for (int preset_index = 0; preset_index < PSVRTrackingColorType_MaxColorTypes; ++preset_index)
    {
        SharedColorPresets.color_presets[preset_index] = k_default_color_presets[preset_index];
    }

	memset(video_properties, 0, sizeof(video_properties));
};

const configuru::Config 
CommonTrackerConfig::writeToJSON()
{
    configuru::Config pt{
        {"is_valid", is_valid},
        {"max_poll_failure_count", max_poll_failure_count},
		{"current_mode", current_mode},
        {"brightness", video_properties[PSVRVideoProperty_Brightness]},
		{"contrast", video_properties[PSVRVideoProperty_Contrast]},
		{"hue", video_properties[PSVRVideoProperty_Hue]},
		{"saturation", video_properties[PSVRVideoProperty_Saturation]},
		{"sharpness", video_properties[PSVRVideoProperty_Sharpness]},
		{"gamma", video_properties[PSVRVideoProperty_Gamma]},
		{"whitebalance", video_properties[PSVRVideoProperty_WhiteBalance]},
		{"redbalance", video_properties[PSVRVideoProperty_RedBalance]},
		{"greenbalance", video_properties[PSVRVideoProperty_GreenBalance]},
		{"bluebalance", video_properties[PSVRVideoProperty_BlueBalance]},
		{"gain", video_properties[PSVRVideoProperty_Gain]},
		{"pan", video_properties[PSVRVideoProperty_Pan]},
		{"tilt", video_properties[PSVRVideoProperty_Tilt]},
		{"roll", video_properties[PSVRVideoProperty_Roll]},
		{"zoom", video_properties[PSVRVideoProperty_Zoom]},
		{"exposure", video_properties[PSVRVideoProperty_Exposure]},
		{"iris", video_properties[PSVRVideoProperty_Iris]},
		{"focus", video_properties[PSVRVideoProperty_Focus]},
        {"pose.orientation.w", pose.Orientation.w},
        {"pose.orientation.x", pose.Orientation.x},
        {"pose.orientation.y", pose.Orientation.y},
        {"pose.orientation.z", pose.Orientation.z},
        {"pose.position.x", pose.Position.x},
        {"pose.position.y", pose.Position.y},
        {"pose.position.z", pose.Position.z},
    };

	writeColorPropertyPresetTable(&SharedColorPresets, pt);

	for (auto &controller_preset_table : DeviceColorPresets)
	{
		writeColorPropertyPresetTable(&controller_preset_table, pt);
	}

    return pt;
}

void 
CommonTrackerConfig::readFromJSON(const configuru::Config &pt)
{
    is_valid = pt.get_or<bool>("is_valid", false);
    max_poll_failure_count = pt.get_or<long>("max_poll_failure_count", 100);
	current_mode= pt.get_or<std::string>("current_mode_index", current_mode);

    video_properties[PSVRVideoProperty_Brightness]= (int)pt.get_or<float>("brightness", (float)video_properties[PSVRVideoProperty_Brightness]);
	video_properties[PSVRVideoProperty_Contrast]= (int)pt.get_or<float>("contrast", (float)video_properties[PSVRVideoProperty_Contrast]);
	video_properties[PSVRVideoProperty_Hue]= (int)pt.get_or<float>("hue", (float)video_properties[PSVRVideoProperty_Hue]);
	video_properties[PSVRVideoProperty_Saturation]= (int)pt.get_or<float>("saturation", (float)video_properties[PSVRVideoProperty_Saturation]);
	video_properties[PSVRVideoProperty_Sharpness]= (int)pt.get_or<float>("sharpness", (float)video_properties[PSVRVideoProperty_Sharpness]);
	video_properties[PSVRVideoProperty_Gamma]= (int)pt.get_or<float>("gamma", (float)video_properties[PSVRVideoProperty_Gamma]);
	video_properties[PSVRVideoProperty_WhiteBalance]= (int)pt.get_or<float>("whitebalance", (float)video_properties[PSVRVideoProperty_WhiteBalance]);
	video_properties[PSVRVideoProperty_RedBalance]= (int)pt.get_or<float>("redbalance", (float)video_properties[PSVRVideoProperty_RedBalance]);
	video_properties[PSVRVideoProperty_GreenBalance]= (int)pt.get_or<float>("greenbalance", (float)video_properties[PSVRVideoProperty_GreenBalance]);
	video_properties[PSVRVideoProperty_BlueBalance]= (int)pt.get_or<float>("bluebalance", (float)video_properties[PSVRVideoProperty_BlueBalance]);
	video_properties[PSVRVideoProperty_Gain]= (int)pt.get_or<float>("gain", (float)video_properties[PSVRVideoProperty_Gain]);
	video_properties[PSVRVideoProperty_Pan]= (int)pt.get_or<float>("pan", (float)video_properties[PSVRVideoProperty_Pan]);
	video_properties[PSVRVideoProperty_Tilt]= (int)pt.get_or<float>("tilt", (float)video_properties[PSVRVideoProperty_Tilt]);
	video_properties[PSVRVideoProperty_Roll]= (int)pt.get_or<float>("roll", (float)video_properties[PSVRVideoProperty_Roll]);
	video_properties[PSVRVideoProperty_Zoom]= (int)pt.get_or<float>("zoom", (float)video_properties[PSVRVideoProperty_Zoom]);
	video_properties[PSVRVideoProperty_Exposure]= (int)pt.get_or<float>("exposure", (float)video_properties[PSVRVideoProperty_Exposure]);
	video_properties[PSVRVideoProperty_Iris]= (int)pt.get_or<float>("iris", (float)video_properties[PSVRVideoProperty_Iris]);
	video_properties[PSVRVideoProperty_Focus]= (int)pt.get_or<float>("focus", (float)video_properties[PSVRVideoProperty_Focus]);

    pose.Orientation.w = pt.get_or<float>("pose.orientation.w", 1.0);
    pose.Orientation.x = pt.get_or<float>("pose.orientation.x", 0.0);
    pose.Orientation.y = pt.get_or<float>("pose.orientation.y", 0.0);
    pose.Orientation.z = pt.get_or<float>("pose.orientation.z", 0.0);
    pose.Position.x = pt.get_or<float>("pose.position.x", 0.0);
    pose.Position.y = pt.get_or<float>("pose.position.y", 0.0);
    pose.Position.z = pt.get_or<float>("pose.position.z", 0.0);

	// Read the default preset table
	readColorPropertyPresetTable(pt, &SharedColorPresets);

	// Read all of the controller preset tables
	const std::string controller_prefix("controller_");
	const std::string hmd_prefix("hmd_");
    for (auto& iter : pt.as_object())
	{
		const std::string &entry_name= iter.key();
			
		if (entry_name.compare(0, controller_prefix.length(), controller_prefix) == 0 ||
			entry_name.compare(0, hmd_prefix.length(), hmd_prefix) == 0)
		{
			PSVR_HSVColorRangeTable table;

            strncpy(table.table_name, entry_name.c_str(), sizeof(table.table_name));
			for (int preset_index = 0; preset_index < PSVRTrackingColorType_MaxColorTypes; ++preset_index)
			{
				table.color_presets[preset_index] = k_default_color_presets[preset_index];
			}

			readColorPropertyPresetTable(pt, &table);

			DeviceColorPresets.push_back(table);
		}
	}
}

const PSVR_HSVColorRangeTable *
CommonTrackerConfig::getColorRangeTable(const std::string &table_name) const
{
	const PSVR_HSVColorRangeTable *table= &SharedColorPresets;	

	if (table_name.length() > 0)
	{
		for (auto &entry : DeviceColorPresets)
		{
			if (entry.table_name == table_name)
			{
				table= &entry;
			}
		}
	}

	return table;
}

PSVR_HSVColorRangeTable *
CommonTrackerConfig::getOrAddColorRangeTable(const std::string &table_name)
{
	PSVR_HSVColorRangeTable *table= nullptr;	

	if (table_name.length() > 0)
	{
		for (auto &entry : DeviceColorPresets)
		{
			if (entry.table_name == table_name)
			{
				table= &entry;
			}
		}

		if (table == nullptr)
		{
			PSVR_HSVColorRangeTable Table;

			strncpy(Table.table_name, table_name.c_str(), sizeof(Table.table_name));
			for (int preset_index = 0; preset_index < PSVRTrackingColorType_MaxColorTypes; ++preset_index)
			{
				Table.color_presets[preset_index] = k_default_color_presets[preset_index];
			}

			DeviceColorPresets.push_back(Table);
			table= &DeviceColorPresets[DeviceColorPresets.size() - 1];
		}
	}
	else
	{
		table= &SharedColorPresets;
	}

	return table;
}