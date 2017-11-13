//-- includes -----
#include "VirtualHMD.h"
#include "DeviceInterface.h"
#include "DeviceManager.h"
#include "HMDDeviceEnumerator.h"
#include "VirtualHMDDeviceEnumerator.h"
#include "MathUtility.h"
#include "Logger.h"
#include "Utility.h"
#include <vector>
#include <cstdlib>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

// -- constants -----
#define VIRTUAL_HMD_STATE_BUFFER_MAX 4

// -- private methods

// -- public interface
// -- Morpheus HMD Config
const int VirtualHMDConfig::CONFIG_VERSION = 1;

const configuru::Config
VirtualHMDConfig::writeToJSON()
{
    configuru::Config pt{
        {"is_valid", is_valid},
        {"version", VirtualHMDConfig::CONFIG_VERSION},
        {"Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a},
        {"Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b},
        {"Calibration.Time.MeanUpdateTime", mean_update_time_delta},
        {"PositionFilter.FilterType", position_filter_type},
        {"PositionFilter.MaxVelocity", max_velocity},
        {"prediction_time", prediction_time}
    };

    switch (trackingShape.shape_type)
    {
    case PSVRTrackingShape_Sphere:
        pt["tracking_shape"] = "sphere";
        pt["bulb.radius"] = trackingShape.shape.sphere.radius;
        break;
    case PSVRTrackingShape_LightBar:
        pt["tracking_shape"]= "light_bar";
        pt["lightbar.quad.v0.x"]= trackingShape.shape.lightbar.quad[0].x;
        pt["lightbar.quad.v0.y"]= trackingShape.shape.lightbar.quad[0].y;
        pt["lightbar.quad.v0.z"]= trackingShape.shape.lightbar.quad[0].z;
        pt["lightbar.quad.v1.x"]= trackingShape.shape.lightbar.quad[1].x;
        pt["lightbar.quad.v1.y"]= trackingShape.shape.lightbar.quad[1].y;
        pt["lightbar.quad.v1.z"]= trackingShape.shape.lightbar.quad[1].z;
        pt["lightbar.quad.v2.x"]= trackingShape.shape.lightbar.quad[2].x;
        pt["lightbar.quad.v2.y"]= trackingShape.shape.lightbar.quad[2].y;
        pt["lightbar.quad.v2.z"]= trackingShape.shape.lightbar.quad[2].z;
        pt["lightbar.quad.v3.x"]= trackingShape.shape.lightbar.quad[3].x;
        pt["lightbar.quad.v3.y"]= trackingShape.shape.lightbar.quad[3].y;
        pt["lightbar.quad.v3.z"]= trackingShape.shape.lightbar.quad[3].z;
        pt["lightbar.triangle.v0.x"]= trackingShape.shape.lightbar.triangle[0].x;
        pt["lightbar.triangle.v0.y"]= trackingShape.shape.lightbar.triangle[0].y;
        pt["lightbar.triangle.v0.z"]= trackingShape.shape.lightbar.triangle[0].z;
        pt["lightbar.triangle.v1.x"]= trackingShape.shape.lightbar.triangle[1].x;
        pt["lightbar.triangle.v1.y"]= trackingShape.shape.lightbar.triangle[1].y;
        pt["lightbar.triangle.v1.z"]= trackingShape.shape.lightbar.triangle[1].z;
        pt["lightbar.triangle.v2.x"]= trackingShape.shape.lightbar.triangle[2].x;
        pt["lightbar.triangle.v2.y"]= trackingShape.shape.lightbar.triangle[2].y;
        pt["lightbar.triangle.v2.z"]= trackingShape.shape.lightbar.triangle[2].z;
        break;
    case PSVRTrackingShape_PointCloud:
        pt["tracking_shape"]= "point_cloud";
        pt["points.count"]= trackingShape.shape.pointcloud.point_count;
        for (int point_index= 0; point_index < trackingShape.shape.pointcloud.point_count; ++point_index)
        {
            const char axis_label[3]= {'x', 'y', 'z'};
            const float* axis_values= (const float *)&trackingShape.shape.pointcloud.points[point_index];

            for (int axis_index = 0; axis_index < 3; ++axis_index)
            {
                char key[64];

                Utility::format_string(key, sizeof(key), "points.v%d.%c", point_index, axis_label[axis_index]);
                pt[key]= axis_values[axis_index];
            }
        }
        break;
    }

    writeTrackingColor(pt, tracking_color_id);

    return pt;
}

void
VirtualHMDConfig::readFromJSON(const configuru::Config &pt)
{
    version = pt.get_or<int>("version", 0);

    if (version == VirtualHMDConfig::CONFIG_VERSION)
    {
        is_valid = pt.get_or<bool>("is_valid", false);

        prediction_time = pt.get_or<float>("prediction_time", 0.f);

        position_variance_exp_fit_a = pt.get_or<float>("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
        position_variance_exp_fit_b = pt.get_or<float>("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

        mean_update_time_delta = pt.get_or<float>("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

        position_filter_type = pt.get_or<std::string>("PositionFilter.FilterType", position_filter_type);
        max_velocity = pt.get_or<float>("PositionFilter.MaxVelocity", max_velocity);

        // Read the tracking color
        tracking_color_id = static_cast<PSVRTrackingColorType>(readTrackingColor(pt));

        std::string shape_type= pt.get_or<std::string>("tracking_shape", "sphere");
        if (shape_type == "sphere")
            trackingShape.shape_type= PSVRTrackingShape_Sphere;
        else if (shape_type == "light_bar")
            trackingShape.shape_type= PSVRTrackingShape_LightBar;
        else if (shape_type == "point_cloud")
            trackingShape.shape_type= PSVRTrackingShape_PointCloud;

        switch (trackingShape.shape_type)
        {
        case PSVRTrackingShape_Sphere:
            trackingShape.shape.sphere.radius= pt.get_or<float>("bulb.radius", 2.25f);
            break;
        case PSVRTrackingShape_LightBar:
            trackingShape.shape.lightbar.quad[0].x= pt.get_or<float>("lightbar.quad.v0.x", 0.0f);
            trackingShape.shape.lightbar.quad[0].y= pt.get_or<float>("lightbar.quad.v0.y", 0.0f);
            trackingShape.shape.lightbar.quad[0].z= pt.get_or<float>("lightbar.quad.v0.z", 0.0f);
            trackingShape.shape.lightbar.quad[1].x= pt.get_or<float>("lightbar.quad.v1.x", 0.0f);
            trackingShape.shape.lightbar.quad[1].y= pt.get_or<float>("lightbar.quad.v1.y", 0.0f);
            trackingShape.shape.lightbar.quad[1].z= pt.get_or<float>("lightbar.quad.v1.z", 0.0f);
            trackingShape.shape.lightbar.quad[2].x= pt.get_or<float>("lightbar.quad.v2.x", 0.0f);
            trackingShape.shape.lightbar.quad[2].y= pt.get_or<float>("lightbar.quad.v2.y", 0.0f);
            trackingShape.shape.lightbar.quad[2].z= pt.get_or<float>("lightbar.quad.v2.z", 0.0f);
            trackingShape.shape.lightbar.quad[3].x= pt.get_or<float>("lightbar.quad.v3.x", 0.0f);
            trackingShape.shape.lightbar.quad[3].y= pt.get_or<float>("lightbar.quad.v3.y", 0.0f);
            trackingShape.shape.lightbar.quad[3].z= pt.get_or<float>("lightbar.quad.v3.z", 0.0f);
            trackingShape.shape.lightbar.triangle[0].x= pt.get_or<float>("lightbar.triangle.v0.x", 0.0f);
            trackingShape.shape.lightbar.triangle[0].y= pt.get_or<float>("lightbar.triangle.v0.y", 0.0f);
            trackingShape.shape.lightbar.triangle[0].z= pt.get_or<float>("lightbar.triangle.v0.z", 0.0f);
            trackingShape.shape.lightbar.triangle[1].x= pt.get_or<float>("lightbar.triangle.v1.x", 0.0f);
            trackingShape.shape.lightbar.triangle[1].y= pt.get_or<float>("lightbar.triangle.v1.y", 0.0f);
            trackingShape.shape.lightbar.triangle[1].z= pt.get_or<float>("lightbar.triangle.v1.z", 0.0f);
            trackingShape.shape.lightbar.triangle[2].x= pt.get_or<float>("lightbar.triangle.v2.x", 0.0f);
            trackingShape.shape.lightbar.triangle[2].y= pt.get_or<float>("lightbar.triangle.v2.y", 0.0f);
            trackingShape.shape.lightbar.triangle[2].z= pt.get_or<float>("lightbar.triangle.v2.z", 0.0f);
            break;
        case PSVRTrackingShape_PointCloud:
            trackingShape.shape.pointcloud.point_count= std::min(pt.get_or<int>("points.count", 0), (int)MAX_POINT_CLOUD_POINT_COUNT);
            for (int point_index= 0; point_index < trackingShape.shape.pointcloud.point_count; ++point_index)
            {
                const char axis_label[3]= {'x', 'y', 'z'};
                float* axis_values= (float *)&trackingShape.shape.pointcloud.points[point_index];

                for (int axis_index = 0; axis_index < 3; ++axis_index)
                {
                    char key[64];

                    Utility::format_string(key, sizeof(key), "points.v%d.%c", point_index, axis_label[axis_index]);
                    axis_values[axis_index]= pt.get_or<float>(key, 0.f);
                }
            }
            break;
        }
    }
    else
    {
        PSVR_LOG_WARNING("VirtualHMDConfig") <<
            "Config version " << version << " does not match expected version " <<
            VirtualHMDConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- Virtual HMD -----
VirtualHMD::VirtualHMD()
    : cfg()
    , NextPollSequenceNumber(0)
    , bIsOpen(false)
    , HMDStates()
    , bIsTracking(false)
{
    HMDStates.clear();
}

VirtualHMD::~VirtualHMD()
{
    if (getIsOpen())
    {
        PSVR_LOG_ERROR("~VirtualHMD") << "HMD deleted without calling close() first!";
    }
}

bool VirtualHMD::open()
{
    HMDDeviceEnumerator enumerator(HMDDeviceEnumerator::CommunicationType_VIRTUAL);
    bool success = false;

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

bool VirtualHMD::open(
    const DeviceEnumerator *enumerator)
{
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    const char *cur_dev_path = pEnum->get_path();
    bool success = false;

    if (getIsOpen())
    {
        PSVR_LOG_WARNING("VirtualHMD::open") << "VirtualHMD(" << cur_dev_path << ") already open. Ignoring request.";
        success = true;
    }
    else
    {
        PSVR_LOG_INFO("VirtualHMD::open") << "Opening VirtualHMD(" << cur_dev_path << ").";

        device_identifier = cur_dev_path;
        bIsOpen= true;

        // Load the config file
        cfg = VirtualHMDConfig(pEnum->get_path());
        cfg.load();

        // Save it back out again in case any defaults changed
        cfg.save();

        // Reset the polling sequence counter
        NextPollSequenceNumber = 0;

        success = true;
    }

    return success;
}

void VirtualHMD::close()
{
    if (bIsOpen)
    {
        device_identifier= "";
        bIsOpen= true;
    }
    else
    {
        PSVR_LOG_INFO("VirtualHMD::close") << "MorpheusHMD already closed. Ignoring request.";
    }
}

// Getters
bool
VirtualHMD::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == getDeviceType())
    {
        const char *enumerator_path = pEnum->get_path();
        const char *dev_path = device_identifier.c_str();

#ifdef _WIN32
        matches = _stricmp(dev_path, enumerator_path) == 0;
#else
        matches = strcmp(dev_path, enumerator_path) == 0;
#endif
    }

    return matches;
}

bool
VirtualHMD::getIsReadyToPoll() const
{
    return (getIsOpen());
}

std::string
VirtualHMD::getUSBDevicePath() const
{
    return device_identifier;
}

bool
VirtualHMD::getIsOpen() const
{
    return bIsOpen;
}

IControllerInterface::ePollResult
VirtualHMD::poll()
{
    IHMDInterface::ePollResult result = IHMDInterface::_PollResultFailure;

    if (getIsOpen())
    {
        VirtualHMDState newState;

        // New data available. Keep iterating.
        result = IHMDInterface::_PollResultSuccessNewData;

        // Increment the sequence for every new polling packet
        newState.PollSequenceNumber = NextPollSequenceNumber;
        ++NextPollSequenceNumber;

        // Make room for new entry if at the max queue size
        if (HMDStates.size() >= VIRTUAL_HMD_STATE_BUFFER_MAX)
        {
            HMDStates.erase(HMDStates.begin(), HMDStates.begin() + HMDStates.size() - VIRTUAL_HMD_STATE_BUFFER_MAX);
        }

        HMDStates.push_back(newState);
    }

    return result;
}

void
VirtualHMD::getTrackingShape(PSVRTrackingShape &outTrackingShape) const
{
    outTrackingShape= cfg.trackingShape;
}

bool 
VirtualHMD::setTrackingColorID(const PSVRTrackingColorType tracking_color_id)
{
    bool bSuccess = false;

    if (getIsOpen())
    {
        cfg.tracking_color_id = tracking_color_id;
        cfg.save();
        bSuccess = true;
    }

    return bSuccess;
}

bool 
VirtualHMD::getTrackingColorID(PSVRTrackingColorType &out_tracking_color_id) const
{
    out_tracking_color_id = cfg.tracking_color_id;
    return true;
}

float 
VirtualHMD::getPredictionTime() const
{
    return getConfig()->prediction_time;
}

const CommonDeviceState *
VirtualHMD::getState(
    int lookBack) const
{
    const int queueSize = static_cast<int>(HMDStates.size());
    const CommonDeviceState * result =
        (lookBack < queueSize) ? &HMDStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

long VirtualHMD::getMaxPollFailureCount() const
{
    return 1;
}

void VirtualHMD::setTrackingEnabled(bool bEnable)
{
    if (!bIsTracking && bEnable)
    {
        bIsTracking = true;
    }
    else if (bIsTracking && !bEnable)
    {
        bIsTracking = false;
    }
}