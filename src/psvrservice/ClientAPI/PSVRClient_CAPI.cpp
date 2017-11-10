// -- includes -----
#include "PSVRClient_CAPI.h"
#include "PSVRClient.h"
#include "PSVRService.h"
#include "ServiceRequestHandler.h"
#include "Logger.h"
#include "MathUtility.h"

#include <assert.h>

#ifdef _MSC_VER
	#pragma warning(disable:4996)  // ignore strncpy warning
#endif

// -- macros -----
#define IS_VALID_TRACKER_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_TRACKER_COUNT)
#define IS_VALID_HMD_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_HMD_COUNT)

// -- constants ----
const PSMVector3f k_identity_gravity_calibration_direction= {0.f, 1.f, 0.f};

// -- private data ---
PSVRService *g_psvr_service= nullptr;
PSVRClient *g_psvr_client= nullptr;

// -- public interface -----
const char* PSM_GetClientVersionString()
{
    const char *version_string= PSM_PROTOCOL_VERSION_STRING;

    return version_string;
}

bool PSM_GetIsInitialized()
{
	return g_psvr_client != nullptr && g_psvr_service != nullptr;
}

bool PSM_HasTrackerListChanged()
{
	return g_psvr_client != nullptr && g_psvr_client->pollHasTrackerListChanged();
}

bool PSM_HasHMDListChanged()
{
	return g_psvr_client != nullptr && g_psvr_client->pollHasHMDListChanged();
}

PSMResult PSM_Initialize(PSVRLogSeverityLevel log_level)
{
	PSMResult result= PSMResult_Success;

	if (g_psvr_service == nullptr || !g_psvr_service->getIsInitialized())
	{
		if (g_psvr_service == nullptr)
		{
			g_psvr_service= new PSVRService();
		}

		if (!g_psvr_service->startup(log_level))
		{
			delete g_psvr_service;
			g_psvr_service= nullptr;
			result= PSMResult_Error;
		}
	}
	
	if (result == PSMResult_Success)
	{
		if (g_psvr_client == nullptr || !g_psvr_client->getIsInitialized())
		{
			if (g_psvr_client == nullptr)
			{
				g_psvr_client= new PSVRClient();
			}

			if (!g_psvr_client->startup(log_level))
			{
				delete g_psvr_client;
				g_psvr_client= nullptr;
				result= PSMResult_Error;
			}
		}
	}

    return result;
}

PSMResult PSM_GetServiceVersionString(char *out_version_string, size_t max_version_string)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_client != nullptr)
    {
        result = 
			g_psvr_service->getRequestHandler()->get_service_version(
				out_version_string, max_version_string);
    }

    return result;
}

PSMResult PSM_Shutdown()
{
	PSMResult result= PSMResult_Error;

	if (g_psvr_client != nullptr)
	{
		g_psvr_client->shutdown();

		delete g_psvr_client;
		g_psvr_client= nullptr;

		result= PSMResult_Success;
	}	
	
	if (g_psvr_service != nullptr)
	{
		g_psvr_service->shutdown();

		delete g_psvr_service;
		g_psvr_service= nullptr;

		result= PSMResult_Success;
	}	

    return result;
}

PSMResult PSM_Update()
{
    PSMResult result = PSMResult_Error;

    if (PSM_UpdateNoPollMessages() == PSMResult_Success)
    {
		// Process all events and responses
		// Any incoming events become status flags we can poll (ex: pollHasConnectionStatusChanged)
		g_psvr_client->process_messages();

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_UpdateNoPollMessages()
{
    PSMResult result= PSMResult_Error;

	if (g_psvr_service != nullptr)
	{
		g_psvr_service->update();

		if (g_psvr_client != nullptr)
		{
			g_psvr_client->update();

			result= PSMResult_Success;
		}
	}	

    return result;
}

/// Tracker Pool
PSMTracker *PSM_GetTracker(PSMTrackerID tracker_id)
{
    if (g_psvr_client != nullptr)
        return g_psvr_client->get_tracker_view(tracker_id);
    else
        return nullptr;
}

PSMResult PSM_AllocateTrackerListener(PSMTrackerID tracker_id, const PSMClientTrackerInfo *tracker_info)
{
    if (g_psvr_client != nullptr)
	    return g_psvr_client->allocate_tracker_listener(*tracker_info) ? PSMResult_Success : PSMResult_Error;
    else
        return PSMResult_Error;
}

PSMResult PSM_FreeTrackerListener(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		g_psvr_client->free_tracker_listener(tracker_id);
        result= PSMResult_Success;
    }

    return result;
}

/// Tracker State Methods
PSMResult PSM_GetTrackerScreenSize(PSMTrackerID tracker_id, PSMVector2f *out_screen_size)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        PSMTracker *tracker= g_psm_client->get_tracker_view(tracker_id);

        switch (tracker->tracker_info.tracker_intrinsics.intrinsics_type)
        {
        case PSMTrackerIntrinsics::PSM_STEREO_TRACKER_INTRINSICS:
            out_screen_size->x= tracker->tracker_info.tracker_intrinsics.intrinsics.stereo.pixel_width;
            out_screen_size->y= tracker->tracker_info.tracker_intrinsics.intrinsics.stereo.pixel_height;
            break;
        case PSMTrackerIntrinsics::PSM_MONO_TRACKER_INTRINSICS:
            out_screen_size->x= tracker->tracker_info.tracker_intrinsics.intrinsics.stereo.pixel_width;
            out_screen_size->y= tracker->tracker_info.tracker_intrinsics.intrinsics.stereo.pixel_height;
            break;
        }
        
        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetTrackerIntrinsics(PSMTrackerID tracker_id, PSMTrackerIntrinsics *out_intrinsics)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		PSMTracker *tracker= g_psm_client->get_tracker_view(tracker_id);

        *out_intrinsics= tracker->tracker_info.tracker_intrinsics;
		result= PSMResult_Success;
	}

	return result;
}

/// Tracker Requests
PSMResult PSM_GetTrackerList(PSMTrackerList *out_tracker_list)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psvr_service != nullptr)
    {
		result_code= g_psvr_service->get_tracker_list(out_tracker_list);
    }
    
    return result_code;
}

PSMResult PSM_StartTrackerDataStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		result_code= g_psvr_service->start_tracker_data_stream(tracker_id);
    }

    return result;
}

PSMResult PSM_StopTrackerDataStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		result_code= g_psvr_service->start_tracker_data_stream(tracker_id);
    }

    return result;
}

PSMResult PSM_GetTrackingSpaceSettings(PSMTrackingSpace *out_tracking_space)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psvr_service != nullptr)
    {
        result_code= g_psvr_service->get_tracking_space_settings(out_tracking_space);
    }
    
    return result_code;
}

PSMResult PSM_OpenTrackerVideoStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        result= g_psvr_client->open_video_stream(tracker_id) ? PSMResult_Success : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_CloseTrackerVideoStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		g_psvr_client->close_video_stream(tracker_id);
		result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetTrackerVideoFrameSectionCount(PSMTrackerID tracker_id, int *out_section_count)
{
    PSMResult result= PSMResult_Error;
	assert(out_section_count != nullptr);

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        *out_section_count= g_psm_client->get_video_frame_section_count(tracker_id);
		result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetTrackerVideoFrameBuffer(PSMTrackerID tracker_id, PSMVideoFrameSection section_index, const unsigned char **out_buffer)
{
    PSMResult result= PSMResult_Error;
	assert(out_buffer != nullptr);

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        const unsigned char *buffer= g_psvr_client->get_video_frame_buffer(tracker_id);
		if (buffer != nullptr)
		{
			*out_buffer= buffer;
			result= PSMResult_Success;
		}
    }

    return result;
}

PSMResult PSM_GetTrackerFrustum(PSMTrackerID tracker_id, PSMFrustum *out_frustum)
{
    PSMResult result= PSMResult_Error;
	assert(out_frustum != nullptr);

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		const PSMTracker *tracker= g_psvr_client->get_tracker_view(tracker_id);
		const PSMClientTrackerInfo *tracker_info= &tracker->tracker_info;
		PSM_FrustumSetPose(out_frustum, &tracker_info->tracker_pose);

		// Convert the FOV angles to radians for rendering purposes
        const PSMMonoTrackerIntrinsics &mono_intrinsics= tracker_info->tracker_intrinsics.intrinsics.mono;
		out_frustum->HFOV = mono_intrinsics.hfov * k_degrees_to_radians;
		out_frustum->VFOV = mono_intrinsics.vfov * k_degrees_to_radians;
		out_frustum->zNear = mono_intrinsics.znear;
		out_frustum->zFar = mono_intrinsics.zfar;

		result= PSMResult_Success;
	}

    return result;
}

/// HMD Pool
PSMHeadMountedDisplay *PSM_GetHmd(PSMHmdID hmd_id)
{
    if (g_psvr_client != nullptr)
	    return g_psvr_client->get_hmd_view(hmd_id);
    else
        return nullptr;
}

PSMResult PSM_AllocateHmdListener(PSMHmdID hmd_id)
{
    if (g_psvr_client != nullptr)
	    return g_psvr_client->allocate_hmd_listener(hmd_id) ? PSMResult_Success : PSMResult_Error;
    else
        return PSMResult_Error;
}

PSMResult PSM_FreeHmdListener(PSMHmdID hmd_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		g_psvr_client->free_hmd_listener(hmd_id);

        result= PSMResult_Success;
    }

    return result;

}

/// HMD State Methods
PSMResult PSM_GetHmdOrientation(PSMHmdID hmd_id, PSMQuatf *out_orientation)
{
    PSMResult result= PSMResult_Error;
	assert(out_orientation);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {		
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_orientation = State.Pose.Orientation;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMHmd_Virtual:
            {
				result= PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPosition(PSMHmdID hmd_id, PSMVector3f *out_position)
{
    PSMResult result= PSMResult_Error;
	assert(out_position);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMHmd_Virtual:
            {
				PSMVirtualHMD State= hmd->HmdState.VirtualHMDState;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPose(PSMHmdID hmd_id, PSMPosef *out_pose)
{
    PSMResult result= PSMResult_Error;
	assert(out_pose);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_pose = State.Pose;

				result= (State.bIsOrientationValid && State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMHmd_Virtual:
            {
				PSMVirtualHMD State= hmd->HmdState.VirtualHMDState;
				*out_pose = State.Pose;

				result= (State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetIsHmdStable(PSMHmdID hmd_id, bool *out_is_stable)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_stable);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				const float k_cosine_20_degrees = 0.9396926f;

				// Get the direction the gravity vector should be pointing 
				// while the controller is in cradle pose.
				const PSMVector3f acceleration_direction = hmd->HmdState.MorpheusState.CalibratedSensorData.Accelerometer;
				float acceleration_magnitude;
				PSM_Vector3fNormalizeWithDefaultGetLength(&acceleration_direction, k_psm_float_vector3_zero, &acceleration_magnitude);

				*out_is_stable =
					is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
					PSM_Vector3fDot(&k_identity_gravity_calibration_direction, &acceleration_direction) >= k_cosine_20_degrees;

				result= PSMResult_Success;
            } break;
        case PSMHmd_Virtual:
            {
                // Virtual HMD can never be stable
				*out_is_stable = false;

				result= PSMResult_Success;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetIsHmdTracking(PSMHmdID hmd_id, bool *out_is_tracking)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_tracking);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				*out_is_tracking = hmd->HmdState.MorpheusState.bIsCurrentlyTracking;
				result= PSMResult_Success;
            } break;
        case PSMHmd_Virtual:
            {
				*out_is_tracking = hmd->HmdState.VirtualHMDState.bIsCurrentlyTracking;
				result= PSMResult_Success;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPixelLocationOnTracker(PSMHmdID hmd_id, PSMTrackingProjectionCount projection_index, PSMTrackerID *outTrackerId, PSMVector2f *outLocation)
{
	PSMResult result= PSMResult_Error;
	
    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSMHmd_Virtual:
            {
				trackerData= &hmd->HmdState.VirtualHMDState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
            if (outTrackerId)
                *outTrackerId = trackerData->TrackerID;
            if (outLocation)            
                *outLocation = trackerData->ScreenLocations[projection_index];
			result= PSMResult_Success;
		}
	}

    return result;
}

PSMResult PSM_GetHmdPositionOnTracker(PSMHmdID hmd_id, PSMTrackerID *outTrackerId, PSMVector3f *outPosition)
{
	assert(outPosition);
    assert(outTrackerId);
	PSMResult result= PSMResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSMHmd_Virtual:
            {
				trackerData= &hmd->HmdState.VirtualHMDState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outPosition = trackerData->RelativePositionCm;
			result= PSMResult_Success;
        }
	}

    return result;
}

PSMResult PSM_GetHmdOrientationOnTracker(PSMHmdID hmd_id, PSMTrackerID *outTrackerId, PSMQuatf *outOrientation)
{
	assert(outOrientation);
    assert(outTrackerId);
	PSMResult result= PSMResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSMHmd_Virtual:
            {
				trackerData= nullptr;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outOrientation = trackerData->RelativeOrientation;
			result= PSMResult_Success;
        }
	}

    return result;
}

PSMResult PSM_GetHmdProjectionOnTracker(PSMHmdID hmd_id, PSMTrackerID *outTrackerId, PSMTrackingProjection *outProjection)
{
	assert(outProjection);
    assert(outTrackerId);
	PSMResult result= PSMResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSMHmd_Virtual:
            {
				trackerData= &hmd->HmdState.VirtualHMDState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId= trackerData->TrackerID;
			*outProjection = trackerData->TrackingProjection;
			result= PSMResult_Success;
        }
	}

    return result;
}

PSMResult PSM_GetHmdTrackingShape(PSMHmdID hmd_id, PSMTrackingShape *out_shape)
{
	PSMResult result= PSMResult_Error;
	
    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
	    result= g_psvr_service->get_hmd_tracking_shape(hmd_id, out_shape);
	}

    return result;
}

/// HMD Requests
PSMResult PSM_GetHmdList(PSMHmdList *out_hmd_list)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_service != nullptr)
    {
	    result= g_psvr_service->get_hmd_list(out_hmd_list);
    }
    
    return result;
}

PSMResult PSM_StartHmdDataStream(PSMHmdID hmd_id, unsigned int data_stream_flags)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->start_hmd_data_stream(hmd_id, data_stream_flags);
    }

    return result;
}

PSMResult PSM_StopHmdDataStream(PSMHmdID hmd_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->stop_hmd_data_stream(hmd_id);
    }

    return result;
}

PSMResult PSM_SetHmdDataStreamTrackerIndex(PSMHmdID hmd_id, PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psvr_service != nullptr && 
        IS_VALID_HMD_INDEX(hmd_id) &&
        IS_VALID_TRACKER_INDEX(tracker_id))
    {
		result= g_psvr_service->set_hmd_data_stream_tracker_index(hmd_id, tracker_id);
    }

    return result;
}