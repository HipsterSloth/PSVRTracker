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
#define IS_VALID_TRACKER_INDEX(x) ((x) >= 0 && (x) < PSVRSERVICE_MAX_TRACKER_COUNT)
#define IS_VALID_HMD_INDEX(x) ((x) >= 0 && (x) < PSVRSERVICE_MAX_HMD_COUNT)

// -- constants ----
const PSVRVector3f k_identity_gravity_calibration_direction= {0.f, 1.f, 0.f};

// -- private data ---
PSVRService *g_psvr_service= nullptr;
PSVRClient *g_psvr_client= nullptr;

// -- public interface -----
bool PSVR_GetIsInitialized()
{
	return g_psvr_client != nullptr && g_psvr_service != nullptr;
}

bool PSVR_HasTrackerListChanged()
{
	return g_psvr_client != nullptr && g_psvr_client->pollHasTrackerListChanged();
}

bool PSVR_HasHMDListChanged()
{
	return g_psvr_client != nullptr && g_psvr_client->pollHasHMDListChanged();
}

PSVRResult PSVR_Initialize(PSVRLogSeverityLevel log_level)
{
	PSVRResult result= PSVRResult_Success;

	if (g_psvr_service == nullptr)
	{
		g_psvr_service= new PSVRService();
	}

	if (g_psvr_client == nullptr)
	{
		g_psvr_client= new PSVRClient();
	}

	if (!g_psvr_service->getIsInitialized())
	{
		if (!g_psvr_service->startup(log_level, g_psvr_client, g_psvr_client))
		{
			delete g_psvr_service;
			g_psvr_service= nullptr;
			result= PSVRResult_Error;
		}
	}
	
	if (result == PSVRResult_Success)
	{
		if (!g_psvr_client->startup(log_level, g_psvr_service->getRequestHandler()))
		{
			delete g_psvr_client;
			g_psvr_client= nullptr;
			result= PSVRResult_Error;
		}
	}

    return result;
}

PSVRResult PSVR_GetVersionString(char *out_version_string, size_t max_version_string)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr)
    {
        result = 
			g_psvr_service->getRequestHandler()->get_service_version(
				out_version_string, max_version_string);
    }

    return result;
}

PSVRResult PSVR_Shutdown()
{
	PSVRResult result= PSVRResult_Error;

	if (g_psvr_client != nullptr)
	{
		g_psvr_client->shutdown();

		delete g_psvr_client;
		g_psvr_client= nullptr;

		result= PSVRResult_Success;
	}	
	
	if (g_psvr_service != nullptr)
	{
		g_psvr_service->shutdown();

		delete g_psvr_service;
		g_psvr_service= nullptr;

		result= PSVRResult_Success;
	}	

    return result;
}

PSVRResult PSVR_Update()
{
    PSVRResult result = PSVRResult_Error;

    if (PSVR_UpdateNoPollEvents() == PSVRResult_Success)
    {
		// Process all events and responses
		// Any incoming events become status flags we can poll (ex: pollHasConnectionStatusChanged)
		g_psvr_client->process_messages();

        result= PSVRResult_Success;
    }

    return result;
}

PSVRResult PSVR_UpdateNoPollEvents()
{
    PSVRResult result= PSVRResult_Error;

	if (g_psvr_service != nullptr)
	{
		g_psvr_service->update();

		if (g_psvr_client != nullptr)
		{
			g_psvr_client->update();

			result= PSVRResult_Success;
		}
	}	

    return result;
}

PSVRResult PSVR_PollNextMessage(PSVREventMessage *message, size_t message_size)
{
    // Poll events queued up by the call to g_psm_client->update()
    if (g_psvr_client != nullptr)
        return g_psvr_client->poll_next_message(message, message_size) ? PSVRResult_Success : PSVRResult_Error;
    else
        return PSVRResult_Error;
}

/// Tracker Pool
PSVRTracker *PSVR_GetTracker(PSVRTrackerID tracker_id)
{
    if (g_psvr_client != nullptr)
        return g_psvr_client->get_tracker_view(tracker_id);
    else
        return nullptr;
}

PSVRResult PSVR_AllocateTrackerListener(PSVRTrackerID tracker_id, const PSVRClientTrackerInfo *tracker_info)
{
    if (g_psvr_client != nullptr)
	    return g_psvr_client->allocate_tracker_listener(*tracker_info) ? PSVRResult_Success : PSVRResult_Error;
    else
        return PSVRResult_Error;
}

PSVRResult PSVR_FreeTrackerListener(PSVRTrackerID tracker_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		g_psvr_client->free_tracker_listener(tracker_id);
        result= PSVRResult_Success;
    }

    return result;
}

/// Tracker State Methods
PSVRResult PSVR_GetTrackerScreenSize(PSVRTrackerID tracker_id, PSVRVector2f *out_screen_size)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        PSVRTracker *tracker= g_psvr_client->get_tracker_view(tracker_id);

        switch (tracker->tracker_info.tracker_intrinsics.intrinsics_type)
        {
        case PSVR_STEREO_TRACKER_INTRINSICS:
            out_screen_size->x= tracker->tracker_info.tracker_intrinsics.intrinsics.stereo.pixel_width;
            out_screen_size->y= tracker->tracker_info.tracker_intrinsics.intrinsics.stereo.pixel_height;
            break;
        case PSVR_MONO_TRACKER_INTRINSICS:
            out_screen_size->x= tracker->tracker_info.tracker_intrinsics.intrinsics.stereo.pixel_width;
            out_screen_size->y= tracker->tracker_info.tracker_intrinsics.intrinsics.stereo.pixel_height;
            break;
        }
        
        result= PSVRResult_Success;
    }

    return result;
}

PSVRResult PSVR_GetTrackerIntrinsics(PSVRTrackerID tracker_id, PSVRTrackerIntrinsics *out_intrinsics)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		PSVRTracker *tracker= g_psvr_client->get_tracker_view(tracker_id);

        *out_intrinsics= tracker->tracker_info.tracker_intrinsics;
		result= PSVRResult_Success;
	}

	return result;
}

PSVRResult PSVR_SetTrackerIntrinsics(PSVRTrackerID tracker_id, PSVRTrackerIntrinsics *intrinsics)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && 
        g_psvr_service != nullptr &&
        IS_VALID_TRACKER_INDEX(tracker_id))
    {
        if (g_psvr_service->getRequestHandler()->set_tracker_intrinsics(tracker_id, intrinsics) == PSVRResult_Success)
        {
    		PSVRTracker *tracker= g_psvr_client->get_tracker_view(tracker_id);

            tracker->tracker_info.tracker_intrinsics= *intrinsics;
		    result= PSVRResult_Success;
        }
	}

	return result;
}

PSVRResult PSVR_GetTrackerSettings(PSVRTrackerID tracker_id, PSVRHmdID hmd_id, PSVRClientTrackerSettings *out_settings)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && 
        IS_VALID_TRACKER_INDEX(tracker_id) && 
        IS_VALID_HMD_INDEX(hmd_id))
    {
        result= g_psvr_service->getRequestHandler()->get_tracker_settings(tracker_id, hmd_id, out_settings);
    }

    return result;
}

PSVRResult PSVR_ReloadTrackerSettings(PSVRTrackerID tracker_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        result= g_psvr_service->getRequestHandler()->reload_tracker_settings(tracker_id);
    }

    return result;
}

/// Tracker Requests
PSVRResult PSVR_GetTrackerList(PSVRTrackerList *out_tracker_list)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr)
    {
		result= g_psvr_service->getRequestHandler()->get_tracker_list(out_tracker_list);
    }
    
    return result;
}

PSVRResult PSVR_StartTrackerDataStream(PSVRTrackerID tracker_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		result= g_psvr_service->getRequestHandler()->start_tracker_data_stream(tracker_id);
    }

    return result;
}

PSVRResult PSVR_StopTrackerDataStream(PSVRTrackerID tracker_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		result= g_psvr_service->getRequestHandler()->start_tracker_data_stream(tracker_id);
    }

    return result;
}

PSVRResult PSVR_GetTrackingSpaceSettings(PSVRTrackingSpace *out_tracking_space)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr)
    {
        result= g_psvr_service->getRequestHandler()->get_tracking_space_settings(out_tracking_space);
    }
    
    return result;
}

PSVRResult PSVR_OpenTrackerVideoStream(PSVRTrackerID tracker_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        result= g_psvr_client->open_video_stream(tracker_id) ? PSVRResult_Success : PSVRResult_Error;
    }

    return result;
}

PSVRResult PSVR_CloseTrackerVideoStream(PSVRTrackerID tracker_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		g_psvr_client->close_video_stream(tracker_id);
		result= PSVRResult_Success;
    }

    return result;
}

PSVRResult PSVR_GetTrackerVideoFrameSectionCount(PSVRTrackerID tracker_id, int *out_section_count)
{
    PSVRResult result= PSVRResult_Error;
	assert(out_section_count != nullptr);

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        *out_section_count= g_psvr_client->get_video_frame_section_count(tracker_id);
		result= PSVRResult_Success;
    }

    return result;
}

PSVRResult PSVR_SetTrackerFrameRate(PSVRTrackerID tracker_id, float desired_frame_rate, bool save_setting, float *out_frame_rate)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        result= g_psvr_service->getRequestHandler()->set_tracker_frame_rate(
            tracker_id, desired_frame_rate, save_setting, out_frame_rate);
    }

    return result;
}

PSVRResult PSVR_SetTrackerExposure(PSVRTrackerID tracker_id, float desired_exposure, bool save_setting, float *out_exposure)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        result= g_psvr_service->getRequestHandler()->set_tracker_exposure(
            tracker_id, desired_exposure, save_setting, out_exposure);
    }

    return result;
}

PSVRResult PSVR_SetTrackerGain(PSVRTrackerID tracker_id, float desired_gain, bool save_setting, float *out_gain)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        result= g_psvr_service->getRequestHandler()->set_tracker_exposure(
            tracker_id, desired_gain, save_setting, out_gain);
    }

    return result;
}

PSVRResult PSVR_GetTrackerVideoFrameBuffer(PSVRTrackerID tracker_id, PSVRVideoFrameSection section_index, const unsigned char **out_buffer)
{
    PSVRResult result= PSVRResult_Error;
	assert(out_buffer != nullptr);

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        const unsigned char *buffer= g_psvr_client->get_video_frame_buffer(tracker_id, section_index);
		if (buffer != nullptr)
		{
			*out_buffer= buffer;
			result= PSVRResult_Success;
		}
    }

    return result;
}

PSVRResult PSVR_GetTrackerFrustum(PSVRTrackerID tracker_id, PSVRFrustum *out_frustum)
{
    PSVRResult result= PSVRResult_Error;
	assert(out_frustum != nullptr);

    if (g_psvr_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		const PSVRTracker *tracker= g_psvr_client->get_tracker_view(tracker_id);
		const PSVRClientTrackerInfo *tracker_info= &tracker->tracker_info;
		PSVR_FrustumSetPose(out_frustum, &tracker_info->tracker_pose);

		// Convert the FOV angles to radians for rendering purposes
        const PSVRMonoTrackerIntrinsics &mono_intrinsics= tracker_info->tracker_intrinsics.intrinsics.mono;
		out_frustum->HFOV = mono_intrinsics.hfov * k_degrees_to_radians;
		out_frustum->VFOV = mono_intrinsics.vfov * k_degrees_to_radians;
		out_frustum->zNear = mono_intrinsics.znear;
		out_frustum->zFar = mono_intrinsics.zfar;

		result= PSVRResult_Success;
	}

    return result;
}

/// HMD Pool
PSVRHeadMountedDisplay *PSVR_GetHmd(PSVRHmdID hmd_id)
{
    if (g_psvr_client != nullptr)
	    return g_psvr_client->get_hmd_view(hmd_id);
    else
        return nullptr;
}

PSVRResult PSVR_AllocateHmdListener(PSVRHmdID hmd_id)
{
    if (g_psvr_client != nullptr)
	    return g_psvr_client->allocate_hmd_listener(hmd_id) ? PSVRResult_Success : PSVRResult_Error;
    else
        return PSVRResult_Error;
}

PSVRResult PSVR_FreeHmdListener(PSVRHmdID hmd_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		g_psvr_client->free_hmd_listener(hmd_id);

        result= PSVRResult_Success;
    }

    return result;

}

/// HMD State Methods
PSVRResult PSVR_GetHmdOrientation(PSVRHmdID hmd_id, PSVRQuatf *out_orientation)
{
    PSVRResult result= PSVRResult_Error;
	assert(out_orientation);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {		
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				PSVRMorpheus State= hmd->HmdState.MorpheusState;
				*out_orientation = State.Pose.Orientation;

				result= State.bIsOrientationValid ? PSVRResult_Success : PSVRResult_Error;
            } break;
        case PSVRHmd_Virtual:
            {
				result= PSVRResult_Error;
            } break;
        }
    }

    return result;
}

PSVRResult PSVR_GetHmdPosition(PSVRHmdID hmd_id, PSVRVector3f *out_position)
{
    PSVRResult result= PSVRResult_Error;
	assert(out_position);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				PSVRMorpheus State= hmd->HmdState.MorpheusState;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSVRResult_Success : PSVRResult_Error;
            } break;
        case PSVRHmd_Virtual:
            {
				PSVRVirtualHMD State= hmd->HmdState.VirtualHMDState;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSVRResult_Success : PSVRResult_Error;
            } break;
        }
    }

    return result;
}

PSVRResult PSVR_GetHmdPose(PSVRHmdID hmd_id, PSVRPosef *out_pose)
{
    PSVRResult result= PSVRResult_Error;
	assert(out_pose);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				PSVRMorpheus State= hmd->HmdState.MorpheusState;
				*out_pose = State.Pose;

				result= (State.bIsOrientationValid && State.bIsPositionValid) ? PSVRResult_Success : PSVRResult_Error;
            } break;
        case PSVRHmd_Virtual:
            {
				PSVRVirtualHMD State= hmd->HmdState.VirtualHMDState;
				*out_pose = State.Pose;

				result= (State.bIsPositionValid) ? PSVRResult_Success : PSVRResult_Error;
            } break;
        }
    }

    return result;
}

PSVRResult PSVR_GetIsHmdStable(PSVRHmdID hmd_id, bool *out_is_stable)
{
    PSVRResult result= PSVRResult_Error;
	assert(out_is_stable);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				const float k_cosine_20_degrees = 0.9396926f;

				// Get the direction the gravity vector should be pointing 
				// while the controller is in cradle pose.
				const PSVRVector3f acceleration_direction = hmd->HmdState.MorpheusState.CalibratedSensorData.Accelerometer;
				float acceleration_magnitude;
				PSVR_Vector3fNormalizeWithDefaultGetLength(&acceleration_direction, k_PSVR_float_vector3_zero, &acceleration_magnitude);

				*out_is_stable =
					is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
					PSVR_Vector3fDot(&k_identity_gravity_calibration_direction, &acceleration_direction) >= k_cosine_20_degrees;

				result= PSVRResult_Success;
            } break;
        case PSVRHmd_Virtual:
            {
                // Virtual HMD can never be stable
				*out_is_stable = false;

				result= PSVRResult_Success;
            } break;
        }
    }

    return result;
}

PSVRResult PSVR_GetIsHmdTracking(PSVRHmdID hmd_id, bool *out_is_tracking)
{
    PSVRResult result= PSVRResult_Error;
	assert(out_is_tracking);

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				*out_is_tracking = hmd->HmdState.MorpheusState.bIsCurrentlyTracking;
				result= PSVRResult_Success;
            } break;
        case PSVRHmd_Virtual:
            {
				*out_is_tracking = hmd->HmdState.VirtualHMDState.bIsCurrentlyTracking;
				result= PSVRResult_Success;
            } break;
        }
    }

    return result;
}

PSVRResult PSVR_GetHmdPixelLocationOnTracker(PSVRHmdID hmd_id, PSVRTrackingProjectionCount projection_index, PSVRTrackerID *outTrackerId, PSVRVector2f *outLocation)
{
	PSVRResult result= PSVRResult_Error;
	
    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
		PSVRRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSVRHmd_Virtual:
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
			result= PSVRResult_Success;
		}
	}

    return result;
}

PSVRResult PSVR_GetHmdPositionOnTracker(PSVRHmdID hmd_id, PSVRTrackerID *outTrackerId, PSVRVector3f *outPosition)
{
	assert(outPosition);
    assert(outTrackerId);
	PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
		PSVRRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSVRHmd_Virtual:
            {
				trackerData= &hmd->HmdState.VirtualHMDState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outPosition = trackerData->RelativePositionCm;
			result= PSVRResult_Success;
        }
	}

    return result;
}

PSVRResult PSVR_GetHmdOrientationOnTracker(PSVRHmdID hmd_id, PSVRTrackerID *outTrackerId, PSVRQuatf *outOrientation)
{
	assert(outOrientation);
    assert(outTrackerId);
	PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
		PSVRRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSVRHmd_Virtual:
            {
				trackerData= nullptr;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outOrientation = trackerData->RelativeOrientation;
			result= PSVRResult_Success;
        }
	}

    return result;
}

PSVRResult PSVR_GetHmdProjectionOnTracker(PSVRHmdID hmd_id, PSVRTrackerID *outTrackerId, PSVRTrackingProjection *outProjection)
{
	assert(outProjection);
    assert(outTrackerId);
	PSVRResult result= PSVRResult_Error;

    if (g_psvr_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= g_psvr_client->get_hmd_view(hmd_id);
		PSVRRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSVRHmd_Virtual:
            {
				trackerData= &hmd->HmdState.VirtualHMDState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId= trackerData->TrackerID;
			*outProjection = trackerData->TrackingProjection;
			result= PSVRResult_Success;
        }
	}

    return result;
}

PSVRResult PSVR_GetHmdTrackingShape(PSVRHmdID hmd_id, PSVRTrackingShape *out_shape)
{
	PSVRResult result= PSVRResult_Error;
	
    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
	    result= g_psvr_service->getRequestHandler()->get_hmd_tracking_shape(hmd_id, out_shape);
	}

    return result;
}

/// HMD Requests
PSVRResult PSVR_GetHmdList(PSVRHmdList *out_hmd_list)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr)
    {
	    result= g_psvr_service->getRequestHandler()->get_hmd_list(out_hmd_list);
    }
    
    return result;
}

PSVRResult PSVR_StartHmdDataStream(PSVRHmdID hmd_id, unsigned int data_stream_flags)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->getRequestHandler()->start_hmd_data_stream(hmd_id, data_stream_flags);
    }

    return result;
}

PSVRResult PSVR_StopHmdDataStream(PSVRHmdID hmd_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->getRequestHandler()->stop_hmd_data_stream(hmd_id);
    }

    return result;
}

PSVRResult PSVR_SetHmdDataStreamTrackerIndex(PSVRHmdID hmd_id, PSVRTrackerID tracker_id)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && 
        IS_VALID_HMD_INDEX(hmd_id) &&
        IS_VALID_TRACKER_INDEX(tracker_id))
    {
		result= g_psvr_service->getRequestHandler()->set_hmd_data_stream_tracker_index(hmd_id, tracker_id);
    }

    return result;
}

PSVRResult PSVR_SetHmdPositionFilter(PSVRHmdID hmd_id, const char *position_filter)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->getRequestHandler()->set_hmd_position_filter(hmd_id, std::string(position_filter));
    }

    return result;
}

PSVRResult PSVR_SetHmdOrientationFilter(PSVRHmdID hmd_id, const char *orientation_filter)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->getRequestHandler()->set_hmd_orientation_filter(hmd_id, std::string(orientation_filter));
    }

    return result;
}

PSVRResult PSVR_SetHmdPredictionTime(PSVRHmdID hmd_id, float prediction_time)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->getRequestHandler()->set_hmd_prediction_time(hmd_id, prediction_time);
    }

    return result;
}

PSVRResult PSVR_SetHmdTrackingColorID(PSVRHmdID hmd_id, PSVRTrackingColorType tracking_color_type)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->getRequestHandler()->set_hmd_led_tracking_color(hmd_id, tracking_color_type);
    }

    return result;
}

PSVRResult PSVR_SetTrackerColorFilter(
    PSVRTrackerID tracker_id, PSVRHmdID hmd_id, PSVRTrackingColorType tracking_color_type,
    PSVR_HSVColorRange *desired_color_filter, PSVR_HSVColorRange *out_color_filter)
{
    PSVRResult result= PSVRResult_Error;

    if (g_psvr_service != nullptr && 
        IS_VALID_TRACKER_INDEX(tracker_id) &&
        IS_VALID_HMD_INDEX(hmd_id))
    {
		result= g_psvr_service->getRequestHandler()->set_tracker_color_preset(
            tracker_id, hmd_id, tracking_color_type,
            *desired_color_filter, *out_color_filter);
    }

    return result;
}