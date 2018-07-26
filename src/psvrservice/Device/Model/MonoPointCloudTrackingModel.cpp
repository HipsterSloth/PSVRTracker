// -- include -----
#include "MonoPointCloudTrackingModel.h"
#include "MathAlignment.h"
#include "MathEigen.h"
#include "MathTypeConversion.h"
#include "ServerTrackerView.h"
#include "StackContainer.h"
#include "TrackingModelMath.h"
#include "TrackerMath.h"
#include "TrackerManager.h"
#include "Utility.h"

#include <array>
#include <bitset>
#include <vector>

//-- typedefs ----
using t_high_resolution_timepoint= std::chrono::time_point<std::chrono::high_resolution_clock>;
using t_high_resolution_duration= t_high_resolution_timepoint::duration;
using t_double_seconds = std::chrono::duration<double>;

//-- constants ---
static const int k_max_pose_history= 5;

static const float k_reprojection_correspondance_tolerance_px = 15.0f;
static const float k_reprojection_correspondance_tolerance_px_sqrd = 
	k_reprojection_correspondance_tolerance_px*k_reprojection_correspondance_tolerance_px;

//-- private structures ----
struct MonoPointCorrespondence
{
	int proj_point_index; // index into PSVRTrackingProjectionData::shape.pointcloud.points
	int model_point_index; // index in PSVRTrackingShape::shape.pointcloud.points
    PSVRVector2f pixel_projection;

	void clear()
	{
		proj_point_index= -1;
		model_point_index= -1;
		pixel_projection= *k_PSVR_float_vector2_zero;
	}
};

struct PoseHistory
{
	std::deque<ShapeTimestampedPose> poseHistory;

	void clear()
	{
		poseHistory.clear();
	}

	void addTimestampedPose(const ShapeTimestampedPose &new_pose)
	{
		// Throw out old poses
		while (poseHistory.size() >= k_max_pose_history)
		{
			// old poses get popped off the front
			poseHistory.pop_front();
		}

		// Add the new pose 
		if (poseHistory.size() == 0 ||
			new_pose.timestamp > poseHistory.back().timestamp)
		{
			// New poses go on the back
			poseHistory.push_back(new_pose);
		}
	}

	bool estimateLinearVelocity(Eigen::Vector3d &out_velocity)
	{
		if (poseHistory.size() > 1)
		{
			const std::chrono::time_point<std::chrono::high_resolution_clock> old_time= poseHistory.front().timestamp;
			const std::chrono::time_point<std::chrono::high_resolution_clock> new_time= poseHistory.back().timestamp;
			assert(new_time > old_time);
			const double dT= std::chrono::duration_cast<t_double_seconds>(new_time - old_time).count();

			if (dT > k_real64_epsilon)
			{
				const Eigen::Vector3d old_location= 
					PSVR_vector3f_to_eigen_vector3(poseHistory.front().pose_cm.Position).cast<double>();
				const Eigen::Vector3d new_location=
					PSVR_vector3f_to_eigen_vector3(poseHistory.back().pose_cm.Position).cast<double>();

				out_velocity= (new_location - old_location) / dT;

				return true;
			}
		}

		return false;
	}

	bool estimateQuaternionDerivative(Eigen::Quaterniond &out_q_dot)
	{
		if (poseHistory.size() > 1)
		{
			const std::chrono::time_point<std::chrono::high_resolution_clock> old_time= poseHistory.front().timestamp;
			const std::chrono::time_point<std::chrono::high_resolution_clock> new_time= poseHistory.back().timestamp;
			assert(new_time > old_time);
			const double dT= std::chrono::duration_cast<t_double_seconds>(new_time - old_time).count();

			if (dT > k_real64_epsilon)
			{
				const Eigen::Quaterniond old_quat= 
					PSVR_quatf_to_eigen_quaternionf(poseHistory.front().pose_cm.Orientation).cast<double>();
				const Eigen::Quaterniond new_quat= 
					PSVR_quatf_to_eigen_quaternionf(poseHistory.back().pose_cm.Orientation).cast<double>();

				out_q_dot= Eigen::Quaterniond((new_quat.coeffs() - old_quat.coeffs()) / dT);

				return true;
			}
		}

		return false;
	}

	bool estimateNewTransform(const t_high_resolution_timepoint &new_time, Eigen::Affine3d &outNewTransform)
	{
		Eigen::Vector3d linear_velocity;
		Eigen::Quaterniond q_dot;

		if (estimateLinearVelocity(linear_velocity) && estimateQuaternionDerivative(q_dot))
		{
			const ShapeTimestampedPose &latest_pose= poseHistory.back();
			const Eigen::Vector3d p_latest= 
				PSVR_vector3f_to_eigen_vector3(latest_pose.pose_cm.Position).cast<double>();
			const Eigen::Quaterniond q_latest= 
				PSVR_quatf_to_eigen_quaternionf(latest_pose.pose_cm.Orientation).cast<double>();

			double dT= std::chrono::duration_cast<t_double_seconds>(new_time - latest_pose.timestamp).count();

			if (dT > k_real64_epsilon)
			{
				const Eigen::Vector3d p_estimated= p_latest + linear_velocity*dT;
				const Eigen::Quaterniond q_estimated = 
					Eigen::Quaterniond(q_latest.coeffs() + q_dot.coeffs()*dT).normalized();

				outNewTransform.linear().noalias() = q_estimated.toRotationMatrix();
				outNewTransform.translation().noalias() = p_estimated;
			}
			else
			{
				outNewTransform.linear().noalias() = q_latest.toRotationMatrix();
				outNewTransform.translation().noalias() = p_latest;
			}

			return true;
		}

		return false;
	}
};

struct MonoPointCloudTrackingModelState
{
    std::vector<Eigen::Vector3f> modelVertices;
	std::vector<Eigen::Vector3f> modelNormals;
	std::vector<t_tri_index_tuple> modelTriIndexPermutations;

	PoseHistory poseHistory;

    Eigen::Affine3d currentOpticalTransform; // cm
	t_high_resolution_timepoint currentTransformTimestamp;
    bool bIsCurrentOpticalTransformValid;

    std::vector<MonoPointCorrespondence> currentPointCorrespondences;

	void clearHistory()
	{
		poseHistory.clear();
		currentPointCorrespondences.clear();
		currentOpticalTransform= Eigen::Affine3d::Identity();
		bIsCurrentOpticalTransformValid= false;
	}
};

//-- private methods -----
static bool compute_transform_using_best_fit_correspondence(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<Eigen::Vector3f> &model_normals,
	const std::vector<t_tri_index_tuple> &model_tri_permutations,
	const Eigen::Affine3d *predicted_model_transform_ptr,
	std::vector<MonoPointCorrespondence> &out_point_correspondences,
	Eigen::Affine3d &out_transform);
static bool compute_predicted_point_correspondences(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<Eigen::Vector3f> &model_normals,
	const Eigen::Affine3d &predicted_model_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences);
static bool compute_brute_force_image_ray_correspondences(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
    const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<Eigen::Vector3f> &model_normals,
	const std::vector<t_tri_index_tuple> &model_tri_permutations,
	const Eigen::Affine3d *predicted_model_transform,
	Eigen::Affine3d &out_model_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences);

//-- public implementation -----
MonoPointCloudTrackingModel::MonoPointCloudTrackingModel() :
    m_state(new MonoPointCloudTrackingModelState)
{
    m_state->currentOpticalTransform= Eigen::Affine3d::Identity();
    m_state->bIsCurrentOpticalTransformValid= false;
}

MonoPointCloudTrackingModel::~MonoPointCloudTrackingModel()
{
    delete m_state;
}

bool MonoPointCloudTrackingModel::init(PSVRTrackingShape *tracking_shape)
{
    bool bSuccess= false;

    if (tracking_shape->shape_type == PSVRTrackingShape_PointCloud &&
        tracking_shape->shape.pointcloud.point_count >= 3)
    {
        const int model_point_count= tracking_shape->shape.pointcloud.point_count;

        m_state->currentOpticalTransform = Eigen::Affine3d::Identity();

        m_state->modelVertices.resize(model_point_count);
		m_state->modelNormals.resize(model_point_count);
        for (int source_index = 0; source_index < model_point_count; ++source_index)
        {
            const PSVRVector3f &point= tracking_shape->shape.pointcloud.points[source_index];
			const PSVRVector3f &normal= tracking_shape->shape.pointcloud.normals[source_index];

            m_state->modelVertices[source_index]= Eigen::Vector3f(point.x, point.y, point.z);
			m_state->modelNormals[source_index]= Eigen::Vector3f(normal.x, normal.y, normal.z);
        }

		// Generate a list of all visible permutations of triangles in the model points
        compute_all_visible_tri_index_permutations(
            m_state->modelVertices,
            m_state->modelNormals,
            m_state->modelTriIndexPermutations);

        bSuccess= true;
    }

    return bSuccess;
}

bool MonoPointCloudTrackingModel::applyShapeProjectionFromTracker(
	const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
    const class ServerTrackerView *tracker_view,
	const ShapeTimestampedPose *last_filtered_pose,
    const PSVRTrackingProjection &projection)
{
    bool bSuccess= false;

    assert(projection.projection_count == MONO_PROJECTION_COUNT);

	// Make the latest filtered pose tracker relative
	// and add it to filtered pose historyu
	if (last_filtered_pose->bIsValid)
	{
		ShapeTimestampedPose trackerRelativePose;

		trackerRelativePose.pose_cm.Orientation= 
			tracker_view->computeTrackerOrientation(&last_filtered_pose->pose_cm.Orientation);
		trackerRelativePose.pose_cm.Position=
			tracker_view->computeTrackerPosition(&last_filtered_pose->pose_cm.Position);
		trackerRelativePose.timestamp= last_filtered_pose->timestamp;
		trackerRelativePose.bIsValid= true;

		m_state->poseHistory.addTimestampedPose(trackerRelativePose);
	}	

	// Use a linear velocity model to predict a new transform for the point cloud
	Eigen::Affine3d predicted_transform;
	Eigen::Affine3d *predicted_transform_ptr= nullptr;
	if (m_state->poseHistory.estimateNewTransform(now, predicted_transform))
	{
		predicted_transform_ptr= &predicted_transform;
	}

	// Make a local copy of the model vertices and normals
	std::vector<Eigen::Vector3f> model_vertices= m_state->modelVertices;
	std::vector<Eigen::Vector3f> model_normals= m_state->modelNormals;

	// Try to find the optical transform of the lights using best fit correspondence
	Eigen::Affine3d new_transform;
	std::vector<MonoPointCorrespondence> new_correspondences;
	if (compute_transform_using_best_fit_correspondence(
			tracker_view, projection, 
			model_vertices, model_normals, m_state->modelTriIndexPermutations,
			predicted_transform_ptr,
			new_correspondences, new_transform))
	{
		m_state->currentOpticalTransform= new_transform;
		m_state->currentTransformTimestamp= now;
		m_state->bIsCurrentOpticalTransformValid= true;

		m_state->currentPointCorrespondences= new_correspondences;
		bSuccess= true;
	}
	else
	{
		m_state->clearHistory();
	}

    return bSuccess;
}

bool MonoPointCloudTrackingModel::getShapeOrientation(PSVRQuatf &out_orientation) const
{
    if (m_state->bIsCurrentOpticalTransformValid)
    {
        Eigen::Quaternionf q= Eigen::Quaternionf(
            m_state->currentOpticalTransform.linear().cast<float>());

		assert_eigen_quaternion_is_normalized(q);
        out_orientation= eigen_quaternionf_to_PSVR_quatf(q);
    }

    return m_state->bIsCurrentOpticalTransformValid;
}

bool MonoPointCloudTrackingModel::getShapePosition(PSVRVector3f &out_position) const
{
    if (m_state->bIsCurrentOpticalTransformValid)
    {
        out_position=
            eigen_vector3f_to_PSVR_vector3f(
                m_state->currentOpticalTransform.translation().cast<float>());
    }

    return m_state->bIsCurrentOpticalTransformValid;
}

bool MonoPointCloudTrackingModel::getShape(PSVRTrackingShape &out_shape) const
{
    if (m_state->bIsCurrentOpticalTransformValid)
    {
        const int point_count= static_cast<int>(m_state->modelVertices.size());

        out_shape.shape_type= PSVRTrackingShape_PointCloud;
        out_shape.shape.pointcloud.point_count= point_count;
        for (int point_index = 0; point_index < point_count; ++point_index)
        {
            const Eigen::Vector3d model_vertex= m_state->modelVertices[point_index].cast<double>();
            const Eigen::Vector3f world_vertex= (m_state->currentOpticalTransform * model_vertex).cast<float>();

            out_shape.shape.pointcloud.points[point_index]= eigen_vector3f_to_PSVR_vector3f(world_vertex);
        }
    }

    return m_state->bIsCurrentOpticalTransformValid;
}

bool MonoPointCloudTrackingModel::getPointCloudProjectionShapeCorrelation(PSVRTrackingProjection &projection) const
{
	assert(projection.shape_type == PSVRShape_PointCloud);

	if (m_state->bIsCurrentOpticalTransformValid)
    {
		for (const MonoPointCorrespondence &pixel_data : m_state->currentPointCorrespondences)
		{
			if (pixel_data.model_point_index != -1 &&
				pixel_data.proj_point_index != -1)
			{
				projection.projections[PRIMARY_PROJECTION_INDEX].shape.pointcloud.shape_point_index[pixel_data.proj_point_index]= 
					pixel_data.model_point_index;
			}
		}
    }

    return m_state->bIsCurrentOpticalTransformValid;
}

//-- private implementation -----
static void compute_image_point_camera_rays(
	const cv::Matx33f &intrinsic_matrix,
	const cv::Matx<float, 5, 1> &dist_coeffs,
	const PSVRTrackingProjection &projection,
	std::vector<Eigen::Vector3f> &out_image_point_rays)
{
	// Extract the focal lengths and principal point from the intrinsic matrix
	float fx, fy, cx, cy;
	extractCameraIntrinsicMatrixParameters(intrinsic_matrix, fx, fy, cx, cy);

	// Convert the PSVRVector2f points in the projection into a cv::Point2f list
	StackVector<cv::Point2f, MAX_POINT_CLOUD_POINT_COUNT> cv_image_points;
	const int image_point_count= projection.projections[0].shape.pointcloud.point_count;
	for (int image_point_index = 0; image_point_index < image_point_count; ++image_point_index)
	{
		const PSVRVector2f &image_point= projection.projections[0].shape.pointcloud.points[image_point_index];

		cv_image_points->push_back({image_point.x, image_point.y});
	}

	// Reverse the effects of the camera lens distortion
    StackVector<cv::Point2f, MAX_POINT_CLOUD_POINT_COUNT> undistorted_image_points;
	undistorted_image_points->resize(image_point_count);
    cv::undistortPoints(
		cv::_InputArray(cv_image_points->data(), image_point_count), 
		cv::_OutputArray(undistorted_image_points->data(), image_point_count), 
		intrinsic_matrix, dist_coeffs);

	// Compute a normalized ray for each projection point
	// See: http://answers.opencv.org/question/4862/how-can-i-do-back-projection/
	out_image_point_rays.clear();
	std::for_each(cv_image_points->begin(), cv_image_points->end(),
		[&out_image_point_rays, fx, fy, cx, cy](const cv::Point2f& image_point) {
			// NOTE!!!
			// Y-Axis is flipped since OpenCV has +Y pointing down
			// and our tracking model assumes that +Y is pointing up
			const Eigen::Vector3f ray((image_point.x - cx)/fx, (cy - image_point.y)/fy, 1.f);
			const Eigen::Vector3f unit_ray= ray.normalized();

			out_image_point_rays.push_back(unit_ray);
		});
}

static cv::Point3f eigen_opengl_vector3f_to_cv_point3f(const Eigen::Vector3f &in)
{
	cv::Mat tvec(3, 1, cv::DataType<double>::type);
	tvec.at<double>(0)= -in.x();
	tvec.at<double>(1)= -in.y();
	tvec.at<double>(2)= in.z();

	return tvec;
}

static bool compute_transform_using_best_fit_correspondence(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<Eigen::Vector3f> &model_normals,
	const std::vector<t_tri_index_tuple> &model_tri_permutations,
	const Eigen::Affine3d *predicted_model_transform_ptr,
	std::vector<MonoPointCorrespondence> &out_point_correspondences,
	Eigen::Affine3d &out_transform)
{
	Eigen::Affine3d solvepnp_guess_transform;
	bool bFoundCorrespondences= false;

	// cheap
	if (predicted_model_transform_ptr != nullptr)
	{
		bFoundCorrespondences=
			compute_predicted_point_correspondences(
					tracker_view, projection, model_vertices, model_normals, *predicted_model_transform_ptr,
					out_point_correspondences);
		solvepnp_guess_transform= *predicted_model_transform_ptr;
	}
	
	// expensive
	if (!bFoundCorrespondences)
	{
		bFoundCorrespondences=
			compute_brute_force_image_ray_correspondences(
				tracker_view, projection, model_vertices, model_normals, model_tri_permutations,
				predicted_model_transform_ptr, solvepnp_guess_transform, out_point_correspondences);
	}

	if (bFoundCorrespondences)
	{
		cv::Matx33f cameraMatrix;
		cv::Matx<float, 5, 1> distCoeffs;
		computeOpenCVCameraIntrinsicMatrix(
			tracker_view->getTrackerDevice(),
			PSVRVideoFrameSection_Primary, 
			cameraMatrix, distCoeffs);

		std::vector<cv::Point2f> cvImagePoints;
		std::vector<cv::Point3f> cvObjectPoints;

		for (const MonoPointCorrespondence &correspondence : out_point_correspondences)
		{
			const int proj_point_index= correspondence.proj_point_index;
			const PSVRVector2f &proj_point = projection.projections[0].shape.pointcloud.points[proj_point_index];

			const int model_point_index= correspondence.model_point_index;
			const Eigen::Vector3f &model_point= model_vertices[model_point_index];

			cvImagePoints.push_back(cv::Point2f(proj_point.x, proj_point.y));
			cvObjectPoints.push_back(eigen_opengl_vector3f_to_cv_point3f(model_point));
		}

        // Fill out the initial guess in OpenCV format for the contour pose
        // if a guess pose was provided
		cv::Mat cvCameraRVec(3, 1, cv::DataType<double>::type);
		cv::Mat cvCameraTVec(3, 1, cv::DataType<double>::type);

		// Use the current transform as an initial guess
		eigen_affine3d_to_cv_rvec_tvec(solvepnp_guess_transform, cvCameraRVec, cvCameraTVec);

		// Use a full solvePnP on the given point to model correspondences to compute a transform
		if (cv::solvePnP(
				cvObjectPoints, cvImagePoints,
				cameraMatrix, distCoeffs,
				cvCameraRVec, cvCameraTVec,
				true, cv::SOLVEPNP_ITERATIVE))
		{
			out_transform= cv_rvec_tvec_to_eigen_affine3d(cvCameraRVec, cvCameraTVec);

			return true;
		}

		return true;
	}

	return false;
}

static bool compute_predicted_point_correspondences(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<Eigen::Vector3f> &model_normals,
	const Eigen::Affine3d &predicted_model_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences)
{
	cv::Mat *drawingBuffer= tracker_view->getDebugDrawingBuffer(PSVRVideoFrameSection_Primary);

	// Get the camera intrinsic matrix and distortion coefficients from the given tracker
	// so that we can project estimated 3d points back onto the camera plane for correlation testing
	cv::Matx33f intrinsic_matrix;
	cv::Matx<float, 5, 1> dist_coeffs;
	computeOpenCVCameraIntrinsicMatrix(
		tracker_view->getTrackerDevice(),
		PSVRVideoFrameSection_Primary, 
		intrinsic_matrix, dist_coeffs);

    // Fill out the initial guess in OpenCV format for the contour pose
    // if a guess pose was provided
	cv::Mat cv_model_rvec(3, 1, cv::DataType<double>::type);
	cv::Mat cv_model_tvec(3, 1, cv::DataType<double>::type);
	eigen_affine3d_to_cv_rvec_tvec(predicted_model_transform, cv_model_rvec, cv_model_tvec);

	// Project the source point on to the solved camera plane
	// model pose = (rvec, tvec)
	std::vector<cv::Point3f> cv_model_points;
	for (const Eigen::Vector3f &opengl_vertex : model_vertices)
	{
		cv_model_points.push_back(eigen_opengl_vector3f_to_cv_point3f(opengl_vertex));
	}

	// Apply the model transform to the model normals
	const Eigen::Affine3f model_pose= predicted_model_transform.cast<float>();
	std::vector<Eigen::Vector3f> transformed_model_normals;
	std::for_each(model_normals.begin(), model_normals.end(),
		[&transformed_model_normals, model_pose](const Eigen::Vector3f& v) {
			const Eigen::Vector3f transformed_v= model_pose.linear() * v;
			transformed_model_normals.push_back(transformed_v);
		});

	std::vector<cv::Point2f> cv_projected_points;
	cv::projectPoints(
		cv_model_points, 
		cv_model_rvec,
		cv_model_tvec,
		intrinsic_matrix, dist_coeffs, cv_projected_points);

	// Find the best correspondences based on the predicted projection
	const size_t proj_point_count= projection.projections[0].shape.pointcloud.point_count;
	const size_t model_vertex_count= model_vertices.size();

	std::vector<bool> proj_point_used(proj_point_count, false);
	out_point_correspondences.clear();

	for (int model_point_index = 0; model_point_index < model_vertex_count; ++model_point_index)
	{
		const Eigen::Vector3f &model_normal= model_normals[model_point_index];

		// Only consider model points facing the camera
		if (model_normal.z() >= 0)
			continue;

		MonoPointCorrespondence correspondence;
		correspondence.clear();
		correspondence.model_point_index= model_point_index;

		float best_correspondence_dist= k_real_max;
		for (int proj_point_index = 0; proj_point_index < proj_point_count; ++proj_point_index)
		{
			if (proj_point_used[proj_point_index])
				continue;

			const PSVRVector2f &proj_point = projection.projections[0].shape.pointcloud.points[proj_point_index];
			const PSVRVector2f model_proj_point= 
				{cv_projected_points[model_point_index].x, cv_projected_points[model_point_index].y};
			const float proj_dist_sqrd= PSVR_Vector2fDistanceSquared(&model_proj_point, &proj_point);
			const bool bIsInRange= 
				proj_dist_sqrd <= k_reprojection_correspondance_tolerance_px_sqrd && 
				proj_dist_sqrd < best_correspondence_dist;

			if (bIsInRange)
			{
				correspondence.proj_point_index= proj_point_index;
				best_correspondence_dist= proj_dist_sqrd;
			}

			if (TrackerManagerConfig::are_debug_flags_enabled(PSMTrackerDebugFlags_trackingModel))
			{
				cv::drawMarker(
					*drawingBuffer, 
					cv::Point((int)model_proj_point.x, (int)model_proj_point.y), 
					bIsInRange ? cv::Scalar(0, 128, 0) : cv::Scalar(128, 0, 0), 
					bIsInRange ? cv::MarkerTypes::MARKER_DIAMOND : cv::MarkerTypes::MARKER_SQUARE,
					(int)k_reprojection_correspondance_tolerance_px);
			}
		}

		if (correspondence.proj_point_index != -1)
		{
			// Mark this projected point as used so that we don't consider that again
			proj_point_used[correspondence.proj_point_index]= true;

			// Store the projection in the correspondence (used to compute estimated_point_cm)
			correspondence.pixel_projection= 
				projection.projections[0].shape.pointcloud.points[correspondence.proj_point_index];

			// Add to the list of valid proj_point -> model_point correspondences
			out_point_correspondences.push_back(correspondence);

			if (TrackerManagerConfig::are_debug_flags_enabled(PSMTrackerDebugFlags_trackingModel))
			{
				char point_label[32];
				Utility::format_string(point_label, sizeof(point_label), "%d", model_point_index);
				cv::putText(*drawingBuffer, 
							point_label,
							cv::Point((int)correspondence.pixel_projection.x, (int)correspondence.pixel_projection.y),
							cv::FONT_HERSHEY_COMPLEX_SMALL,
							1.0, // Scale. 2.0 = 2x bigger
							cv::Scalar(0,255,255), // Color
							1, // Thickness
							CV_AA); // Anti-alias
			}
		}
	}

	// Need at least 4 point correspondences to use SolvePnP in the next step
	return out_point_correspondences.size() >= 4;
}

static bool compute_image_point_ray_correspondences(
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &image_point_rays,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<Eigen::Vector3f> &model_normals,
	const Eigen::Affine3d &model_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences,
	float &out_correspondence_score)
{
	const int model_point_count= (int)model_vertices.size();
	const int image_point_count= projection.projections[0].shape.pointcloud.point_count;
	assert(image_point_count == image_point_rays.size());

	out_correspondence_score= 0.f;
	out_point_correspondences.clear();

	if (image_point_count < 3 || model_vertices.size() < 3)
		return false;

	// Apply the model transform to the model vertices
	StackVector<Eigen::Vector3f, MAX_POINT_CLOUD_POINT_COUNT> transformed_model_vertices;
	std::for_each(model_vertices.begin(), model_vertices.end(),
		[&transformed_model_vertices, model_transform](const Eigen::Vector3f& v) {
			const Eigen::Vector3f transformed_v= model_transform.cast<float>() * v;
			transformed_model_vertices->push_back(transformed_v);
		});

	// Apply the model transform to the model normals
	StackVector<Eigen::Vector3f, MAX_POINT_CLOUD_POINT_COUNT> transformed_model_normals;
	std::for_each(model_normals.begin(), model_normals.end(),
		[&transformed_model_normals, model_transform](const Eigen::Vector3f& v) {
			const Eigen::Vector3f transformed_v= model_transform.linear().cast<float>() * v;
			transformed_model_normals->push_back(transformed_v);
		});


	std::bitset<MAX_POINT_CLOUD_POINT_COUNT> model_point_used;
	for (int image_point_index = 0; image_point_index < image_point_count; ++image_point_index)
	{
		const PSVRVector2f &image_point= projection.projections[0].shape.pointcloud.points[image_point_index];
		const Eigen::ParametrizedLine<float, 3> image_point_ray(Eigen::Vector3f::Zero(), image_point_rays[image_point_index]);

		int best_model_point_index= -1;
		float best_model_point_distance= 0.f;
		for (int model_point_index = 0; model_point_index < model_point_count; ++model_point_index)
		{
			if (model_point_used.test(model_point_index))
				continue;

			const Eigen::Vector3f &model_normal= transformed_model_normals[model_point_index];

			// Only consider model points facing the camera
			if (model_normal.dot(image_point_ray.direction()) >= 0)
			{
				const Eigen::Vector3f &model_point= transformed_model_vertices[model_point_index];
				const float distance_to_ray= image_point_ray.squaredDistance(model_point);

				if (best_model_point_index == -1 || distance_to_ray < best_model_point_distance)
				{
					best_model_point_distance= distance_to_ray;
					best_model_point_index= model_point_index;
				}
			}
		}

		if (best_model_point_index != -1)
		{
			// Record the correspondence
			MonoPointCorrespondence correspondence;

			correspondence.clear();
			correspondence.model_point_index= best_model_point_index;
			correspondence.proj_point_index= image_point_index;
			correspondence.pixel_projection= {image_point.x, image_point.y};

			out_point_correspondences.push_back(correspondence);

			// Mark this model point as used in a correspondence
			model_point_used.set(best_model_point_index, true);

			// Add the best distance to the correspondence score
			out_correspondence_score+= best_model_point_distance;
		}
	}

	return out_point_correspondences.size() >= 3;
}

static bool compute_brute_force_image_ray_correspondences(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
    const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<Eigen::Vector3f> &model_normals,
	const std::vector<t_tri_index_tuple> &model_tri_permutations,
	const Eigen::Affine3d *predicted_model_transform,
	Eigen::Affine3d &out_model_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences)
{
	const size_t proj_point_count= projection.projections[0].shape.pointcloud.point_count;

	out_model_transform= Eigen::Affine3d::Identity();

	out_point_correspondences.clear();
	out_point_correspondences.reserve(MAX_POINT_CLOUD_POINT_COUNT);

	if (proj_point_count < 4)
		return false;

	// Get the camera intrinsic matrix and distortion coefficients from the given tracker
	// so that we can project estimated 3d points back onto the camera plane for correlation testing
	cv::Matx33f intrinsic_matrix;
	cv::Matx<float, 5, 1> dist_coeffs;
	computeOpenCVCameraIntrinsicMatrix(
		tracker_view->getTrackerDevice(),
		PSVRVideoFrameSection_Primary, 
		intrinsic_matrix, dist_coeffs);

	// Compute a ray vector for each image point
	std::vector<Eigen::Vector3f> image_point_rays;
	compute_image_point_camera_rays(
		intrinsic_matrix, dist_coeffs, projection, 
		image_point_rays);

	// Generate a list of all unique combinations of triangles from the "detections" in the projection
	std::vector<t_tri_index_tuple> proj_tri_combinations; // D = Detections
	compute_all_possible_tri_index_combinations(proj_point_count, proj_tri_combinations);

	// Brute-Force test 3-point correspondences from projections points to model points
	std::vector<cv::Mat> cv_model_rvecs;
	std::vector<cv::Mat> cv_model_tvecs;
	std::vector<cv::Point2f> cv_image_points;
	std::vector<cv::Point3f> cv_object_points;
	std::vector<MonoPointCorrespondence> point_correspondences;
	cv_model_rvecs.reserve(4);
	cv_model_tvecs.reserve(4);
	cv_image_points.reserve(MAX_POINT_CLOUD_POINT_COUNT);
	cv_image_points.reserve(MAX_POINT_CLOUD_POINT_COUNT);
	point_correspondences.reserve(MAX_POINT_CLOUD_POINT_COUNT);

	float best_correspondence_score= 0.f;
	for (const t_tri_index_tuple &D : proj_tri_combinations)
	{
		for (const t_tri_index_tuple &L : model_tri_permutations)
		{
			// Use a test projection -> model triangle correspondence 
			// to compute a possible camera pose from solveP3P (up to 4 solutions)
			cv_model_rvecs.clear();
			cv_model_tvecs.clear();

			int solution_count= 0;
			{
				cv_image_points.clear();
				cv_object_points.clear();
				for (int tuple_index = 0; tuple_index < 3; ++tuple_index)
				{
					const int proj_point_index= D[tuple_index];
					const PSVRVector2f &proj_point = projection.projections[0].shape.pointcloud.points[proj_point_index];

					const int model_point_index= L[tuple_index];
					const Eigen::Vector3f &model_point= model_vertices[model_point_index];

					cv_image_points.push_back(cv::Point2f(proj_point.x, proj_point.y));
					cv_object_points.push_back(eigen_opengl_vector3f_to_cv_point3f(model_point));
				}

				solution_count= 
					cv::solveP3P(
						cv_object_points, cv_image_points, 
						intrinsic_matrix, dist_coeffs, 
						cv_model_rvecs, cv_model_tvecs, 
						cv::SOLVEPNP_P3P);
			}

			// Using each estimated model transform returned from solveP3P:
			// * Compute a ray through each projection point
			// * Find the closest transformed(rvec|tvec) model point to each ray
			// * Compute a correspondence score (summed squared error)
			// * Use this correspondence if it's better than any found so far
			for (int solution_index = 0; solution_index < solution_count; ++solution_index)
			{
				// Convert the rvec|tvec pose returned from solveP3P to an Eigen::Affine3f
				const cv::Mat rvec= cv_model_rvecs[solution_index];
				const cv::Mat tvec= cv_model_tvecs[solution_index];
				const Eigen::Affine3d model_transform= cv_rvec_tvec_to_eigen_affine3d(rvec, tvec);

				// Use the predicted transform to throw out solutions with the Y-axis pointing the opposite direction.
				// The guess transform comes from the last filtered pose which incorporates IMU data.
				// The IMU data combined with a known tracker pose tells if we expect to see
				// the model projected upside down or not
				if (predicted_model_transform != nullptr)
				{
					const Eigen::Vector3d model_basis_y= model_transform.linear().col(1);
					const Eigen::Vector3d guess_basis_y= predicted_model_transform->linear().col(1);

					if (guess_basis_y.dot(model_basis_y) < 0.0)
					{
						continue;
					}
				}

				float correspondence_score;
				if (compute_image_point_ray_correspondences(
						projection, image_point_rays, 
						model_vertices, model_normals, model_transform,
						point_correspondences, correspondence_score))
				{
					if (out_point_correspondences.size() == 0 || correspondence_score < best_correspondence_score)
					{
						out_point_correspondences= point_correspondences;
						out_model_transform= model_transform;
						best_correspondence_score= correspondence_score;
					}
				}
			}
		}
	}

	// Need at least 4 point correspondences to use SolvePnP in the next step
	return out_point_correspondences.size() >= 4;
}