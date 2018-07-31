// -- includes -----
#include "TrackerCapabilitiesConfig.h"
#include "Utility.h"

const configuru::Config TrackerFrameSectionInfo::writeToJSON() const
{
	configuru::Config pt{
        {"x", x},
		{"y", y},
    };

	return pt;
}

void TrackerFrameSectionInfo::readFromJSON(const configuru::Config &pt)
{
    x = pt.get_or<int>("x", x);
    y = pt.get_or<int>("y", y);
}

const configuru::Config TrackerModeConfig::writeToJSON() const
{
    configuru::Config pt{
        {"mode_name", modeName},
		{"frame_rate", frameRate},
		{"is_frame_mirrored", isFrameMirrored},
		{"is_buffer_mirrored", isBufferMirrored},
		{"buffer_pixel_width", bufferPixelWidth},
		{"buffer_pixel_height", bufferPixelHeight},
		{"buffer_format", bufferFormat}
    };

	switch (intrinsics.intrinsics_type)
	{
	case PSVR_MONO_TRACKER_INTRINSICS:
		pt["intrinsics_type"]= std::string("mono");
		PSVRConfig::writeMonoTrackerIntrinsics(pt, intrinsics.intrinsics.mono);
		break;
	case PSVR_STEREO_TRACKER_INTRINSICS:
		pt["intrinsics_type"]= std::string("stereo");
		PSVRConfig::writeStereoTrackerIntrinsics(pt, intrinsics.intrinsics.stereo);
		break;
	}

	std::vector<configuru::Config> frame_section_configs;
	for (const TrackerFrameSectionInfo &section_bounds : frameSections)
	{
		frame_section_configs.push_back(section_bounds.writeToJSON());
	}
	pt.insert_or_assign(std::string("frame_sections"), frame_section_configs);

	return pt;
}

void TrackerModeConfig::readFromJSON(const configuru::Config &pt)
{
    modeName = pt.get_or<std::string>("mode_name", modeName);
    frameRate = pt.get_or<float>("frame_rate", frameRate);
	isFrameMirrored = pt.get_or<bool>("is_frame_mirrored", isFrameMirrored);
	isBufferMirrored = pt.get_or<bool>("is_buffer_mirrored", isBufferMirrored);
	bufferFormat = pt.get_or<std::string>("buffer_format", CAMERA_BUFFER_FORMAT_MJPG);

	std::string intrinsics_type= pt.get_or<std::string>("intrinsics_type", "");
	if (intrinsics_type == "mono")
	{
		PSVRConfig::readMonoTrackerIntrinsics(pt, intrinsics.intrinsics.mono);
		intrinsics.intrinsics_type= PSVR_MONO_TRACKER_INTRINSICS;

		bufferPixelWidth= pt.get_or<int>("buffer_pixel_width", (int)intrinsics.intrinsics.mono.pixel_width);
		bufferPixelHeight= pt.get_or<int>("buffer_pixel_height", (int)intrinsics.intrinsics.mono.pixel_height);
	}
	else if (intrinsics_type == "stereo")
	{
		PSVRConfig::readStereoTrackerIntrinsics(pt, intrinsics.intrinsics.stereo);
		intrinsics.intrinsics_type= PSVR_STEREO_TRACKER_INTRINSICS;

		bufferPixelWidth= pt.get_or<int>("buffer_pixel_width", (int)intrinsics.intrinsics.stereo.pixel_width);
		bufferPixelHeight= pt.get_or<int>("buffer_pixel_height", (int)intrinsics.intrinsics.stereo.pixel_height);
	}

	if (pt.has_key("frame_sections"))
	{
		for (const configuru::Config& element : pt["frame_sections"].as_array())
		{
			TrackerFrameSectionInfo section_bounds;
			section_bounds.readFromJSON(element);
			frameSections.push_back(section_bounds);
		}
	}
}

TrackerCapabilitiesConfig::TrackerCapabilitiesConfig(const std::string &fnamebase)
	: PSVRConfig(fnamebase)
	, friendlyName()
	, usbProductId(0x0000)
	, usbVendorId(0x0000)
{
}

const configuru::Config TrackerCapabilitiesConfig::writeToJSON()
{
    configuru::Config pt{
        {"friendly_name", friendlyName},
		{"usb_product_id", usbProductId},
		{"usb_vendor_id", usbVendorId},
    };

	std::vector<configuru::Config> modes;
	for (const TrackerModeConfig &mode : supportedModes)
	{
		modes.push_back(mode.writeToJSON());
	}
	pt.insert_or_assign(std::string("supported_modes"), modes);

	return pt;
}

void TrackerCapabilitiesConfig::readFromJSON(const configuru::Config &pt)
{
	friendlyName= pt.get_or<std::string>("friendly_name", friendlyName);
	usbProductId= pt.get_or<int>("usb_product_id", usbProductId);
	usbVendorId= pt.get_or<int>("usb_vendor_id", usbVendorId);

	for (const configuru::Config& element : pt["supported_modes"].as_array())
	{
		TrackerModeConfig mode;
		mode.readFromJSON(element);
		supportedModes.push_back(mode);
	}

	if (usbVendorId == 0x1415 && usbProductId == 0x2000)
	{
		deviceType= CommonSensorState::PS3EYE;
	}
	else
	{
		if (supportedModes[0].intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS)
		{
			deviceType= CommonSensorState::WMFStereoCamera;
		}
		else
		{
			deviceType= CommonSensorState::WMFMonoCamera;
		}
	}
}

const TrackerModeConfig *TrackerCapabilitiesConfig::findCameraMode(const std::string &mode_name) const
{
	for (const TrackerModeConfig &mode_config : supportedModes)
	{
		if (mode_config.modeName == mode_name)
		{
			return &mode_config;
		}
	}

	return nullptr;
}

void TrackerCapabilitiesConfig::getAvailableTrackerModes(std::vector<std::string> &out_mode_names) const
{
	out_mode_names.clear();
	for (const TrackerModeConfig &mode_config : supportedModes)
	{
		out_mode_names.push_back(mode_config.modeName);
	}
}

void TrackerCapabilitiesSet::reloadSupportedTrackerCapabilities()
{
	std::string capability_directory= Utility::get_resource_directory() + std::string("\\supported_trackers\\");
	std::string search_path= capability_directory + std::string("*.json");

	std::vector<std::string> filenames;
	Utility::fetch_filenames_in_directory(search_path, filenames);

	m_supportedTrackers.clear();
	for (std::string filename : filenames)
	{
		std::string filepath= capability_directory+filename;
		TrackerCapabilitiesConfig config(filename);

		if (config.load(filepath))
		{
			m_supportedTrackers.push_back(config);
		}
	}
}

bool TrackerCapabilitiesSet::supportsTracker(
	unsigned short vendor_id, unsigned short product_id) const
{
	return getTrackerCapabilities(vendor_id, product_id) != nullptr;
}

CommonSensorState::eDeviceType TrackerCapabilitiesSet::findTrackerType(
	unsigned short vendor_id, unsigned short product_id) const
{
	const TrackerCapabilitiesConfig *config= getTrackerCapabilities(vendor_id, product_id);

	return (config != nullptr) ? config->deviceType : CommonSensorState::INVALID_DEVICE_TYPE;
}

const TrackerCapabilitiesConfig *TrackerCapabilitiesSet::getTrackerCapabilities(
	unsigned short vendor_id, unsigned short product_id) const
{
	for (const TrackerCapabilitiesConfig &filter : m_supportedTrackers)
	{
		if (filter.usbVendorId == vendor_id && filter.usbProductId == product_id)
		{
			return &filter;
		}
	}

	return nullptr;
}