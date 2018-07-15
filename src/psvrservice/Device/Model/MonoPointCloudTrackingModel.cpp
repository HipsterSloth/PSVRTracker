// -- include -----
#include "MonoPointCloudTrackingModel.h"
#include "MathAlignment.h"
#include "MathTypeConversion.h"
#include "ServerTrackerView.h"
#include "TrackingModelMath.h"
#include "TrackerMath.h"
#include "TrackerManager.h"

#include <array>
#include <vector>

//-- typedefs ----
using t_high_resolution_timepoint= std::chrono::time_point<std::chrono::high_resolution_clock>;
using t_high_resolution_duration= t_high_resolution_timepoint::duration;
using t_double_seconds = std::chrono::duration<double>;

//-- constants ---
static const float k_reprojection_correspondance_tolerance_px = 5.0f;
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

struct MonoPointCloudTrackingModelState
{
    std::vector<Eigen::Vector3f> modelVertices;
	std::vector<t_tri_index_tuple> modelTriIndexPermutations;

    Eigen::Affine3d prevTransform; // cm
	t_high_resolution_timepoint prevTransformTimestamp; 
    bool bIsPrevTransformValid;

    Eigen::Affine3d currentTransform; // cm
	t_high_resolution_timepoint currentTransformTimestamp;
    bool bIsCurrentTransformValid;

    std::vector<MonoPointCorrespondence> currentPointCorrespondences;

	void clearHistory()
	{
		currentPointCorrespondences.clear();
		prevTransform= Eigen::Affine3d::Identity();
		currentTransform= Eigen::Affine3d::Identity();
		bIsPrevTransformValid= false;
		bIsCurrentTransformValid= false;
	}

	bool estimateLinearVelocity(Eigen::Vector3d &out_velocity)
	{
		if (bIsPrevTransformValid && bIsCurrentTransformValid)
		{
			double dT= std::chrono::duration_cast<t_double_seconds>(currentTransformTimestamp - prevTransformTimestamp).count();

			if (dT > k_real64_epsilon)
			{
				const Eigen::Vector3d prev_location= prevTransform.translation();
				const Eigen::Vector3d curr_location= currentTransform.translation();

				out_velocity= (curr_location - prev_location) / dT;

				return true;
			}
		}

		return false;
	}

	bool estimateQuaternionDerivative(Eigen::Quaterniond &out_q_dot)
	{
		if (bIsPrevTransformValid && bIsCurrentTransformValid)
		{
			double dT= std::chrono::duration_cast<t_double_seconds>(currentTransformTimestamp - prevTransformTimestamp).count();

			if (dT > k_real64_epsilon)
			{
				const Eigen::Quaterniond prev_quat= Eigen::Quaterniond(prevTransform.linear());
				const Eigen::Quaterniond curr_quat= Eigen::Quaterniond(currentTransform.linear());

				out_q_dot= Eigen::Quaterniond((curr_quat.coeffs() - prev_quat.coeffs()) / dT);

				return true;
			}
		}

		return false;
	}

	bool estimateNewTransform(const t_high_resolution_timepoint &newTime, Eigen::Affine3d &outNewTransform)
	{
		Eigen::Vector3d linear_velocity;
		Eigen::Quaterniond q_dot;

		if (estimateLinearVelocity(linear_velocity) && estimateQuaternionDerivative(q_dot))
		{
			double dT= std::chrono::duration_cast<t_double_seconds>(newTime - currentTransformTimestamp).count();

			if (dT > k_real64_epsilon)
			{
				const Eigen::Vector3d curr_location= currentTransform.translation();
				const Eigen::Vector3d new_location= curr_location + linear_velocity*dT;
		
				// Compute the quaternion derivative of the current state
				// q_new= q + q_dot*dT
				const Eigen::Quaterniond curr_quat= Eigen::Quaterniond(currentTransform.linear());
				const Eigen::Quaterniond q_new = Eigen::Quaterniond(curr_quat.coeffs() + q_dot.coeffs()*dT);

				outNewTransform.linear().noalias() = q_new.toRotationMatrix();
				outNewTransform.translation().noalias() = new_location;

				return true;
			}			
		}

		return false;
	}
};

//-- private methods -----
static bool compute_transform_using_best_fit_correspondence(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<t_tri_index_tuple> &model_tri_permutations,
	const Eigen::Affine3d *guess_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences,
	Eigen::Affine3d &out_transform);
static bool compute_predicted_point_correspondences(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const Eigen::Affine3d *guess_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences);
static bool compute_brute_force_point_correspondences(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
    const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<t_tri_index_tuple> &model_tri_permutations,
	std::vector<MonoPointCorrespondence> &out_point_correspondences);

//-- public implementation -----
MonoPointCloudTrackingModel::MonoPointCloudTrackingModel() :
    m_state(new MonoPointCloudTrackingModelState)
{
    m_state->currentTransform= Eigen::Affine3d::Identity();
    m_state->bIsCurrentTransformValid= false;
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

        m_state->currentTransform = Eigen::Affine3d::Identity();

        m_state->modelVertices.resize(model_point_count);
        for (int source_index = 0; source_index < model_point_count; ++source_index)
        {
            const PSVRVector3f &point= tracking_shape->shape.pointcloud.points[source_index];

            m_state->modelVertices[source_index]= Eigen::Vector3f(point.x, point.y, point.z);
        }

		// Generate a list of all visible permutations of triangles in the model points
        compute_all_visible_tri_index_permutations(
            m_state->modelVertices,
            Eigen::Vector3f(0.f, 0.f, 12.f), //###HipsterSloth $HACK specific to PSVR headset
            m_state->modelTriIndexPermutations);

        bSuccess= true;
    }

    return bSuccess;
}

bool MonoPointCloudTrackingModel::applyShapeProjectionFromTracker(
	const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
    const class ServerTrackerView *tracker_view,
    const PSVRTrackingProjection &projection)
{
    bool bSuccess= false;

    assert(projection.projection_count == MONO_PROJECTION_COUNT);

	// Use a linear velocity model to predict a new transform for the point cloud
	Eigen::Affine3d predicted_transform;
	Eigen::Affine3d *predicted_transform_ptr= nullptr;
	if (m_state->estimateNewTransform(now, predicted_transform))
	{
		predicted_transform_ptr= &predicted_transform;
	}

	// Try to find the optical transform of the lights using best fit correspondence
	Eigen::Affine3d new_transform;
	std::vector<MonoPointCorrespondence> new_correspondences;
	if (compute_transform_using_best_fit_correspondence(
			tracker_view, projection, m_state->modelVertices, m_state->modelTriIndexPermutations,
			predicted_transform_ptr,
			new_correspondences, new_transform))
	{
		if (m_state->bIsCurrentTransformValid)
		{
			m_state->prevTransform= m_state->currentTransform;
			m_state->prevTransformTimestamp= m_state->currentTransformTimestamp;
			m_state->bIsPrevTransformValid= true;
		}

		m_state->currentTransform= new_transform;
		m_state->currentTransformTimestamp= now;
		m_state->bIsCurrentTransformValid= true;

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
    if (m_state->bIsCurrentTransformValid)
    {
        Eigen::Quaternionf q= Eigen::Quaternionf(
            m_state->currentTransform.linear().cast<float>());

        out_orientation= eigen_quaternionf_to_PSVR_quatf(q);
    }

    return m_state->bIsCurrentTransformValid;
}

bool MonoPointCloudTrackingModel::getShapePosition(PSVRVector3f &out_position) const
{
    if (m_state->bIsCurrentTransformValid)
    {
        out_position=
            eigen_vector3f_to_PSVR_vector3f(
                m_state->currentTransform.translation().cast<float>());
    }

    return m_state->bIsCurrentTransformValid;
}

bool MonoPointCloudTrackingModel::getShape(PSVRTrackingShape &out_shape) const
{
    if (m_state->bIsCurrentTransformValid)
    {
        const int point_count= static_cast<int>(m_state->modelVertices.size());

        out_shape.shape_type= PSVRTrackingShape_PointCloud;
        out_shape.shape.pointcloud.point_count= point_count;
        for (int point_index = 0; point_index < point_count; ++point_index)
        {
            const Eigen::Vector3d model_vertex= m_state->modelVertices[point_index].cast<double>();
            const Eigen::Vector3f world_vertex= (m_state->currentTransform * model_vertex).cast<float>();

            out_shape.shape.pointcloud.points[point_index]= eigen_vector3f_to_PSVR_vector3f(world_vertex);
        }
    }

    return m_state->bIsCurrentTransformValid;
}

bool MonoPointCloudTrackingModel::getPointCloudProjectionShapeCorrelation(PSVRTrackingProjection &projection) const
{
	assert(projection.shape_type == PSVRShape_PointCloud);

	if (m_state->bIsCurrentTransformValid)
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

    return m_state->bIsCurrentTransformValid;
}

//-- private implementation -----
static bool compute_transform_using_best_fit_correspondence(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<t_tri_index_tuple> &model_tri_permutations,
	const Eigen::Affine3d *guess_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences,
	Eigen::Affine3d &out_transform)
{
	if (
		//compute_predicted_point_correspondences(  // cheap
		//	tracker_view, projection, model_vertices, guess_transform,
		//	out_point_correspondences) ||
		compute_brute_force_point_correspondences( // expensive
			tracker_view, projection, model_vertices, model_tri_permutations,
			out_point_correspondences))
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
			cvObjectPoints.push_back(cv::Point3f(model_point.x(), model_point.y(), model_point.z()));
		}

        // Fill out the initial guess in OpenCV format for the contour pose
        // if a guess pose was provided
		cv::Mat cvCameraRVec(3, 1, cv::DataType<double>::type);
		cv::Mat cvCameraTVec(3, 1, cv::DataType<double>::type);

		// Use the current transform as an initial guess
        bool bUseExtrinsicGuess= false;
        if (guess_transform != nullptr)
        {
            // solvePnP expects a rotation as a Rodrigues (AngleAxis) vector
			Eigen::Quaterniond orientation_estimate= Eigen::Quaterniond(guess_transform->linear());
            cvCameraRVec= eigen_quatd_to_cv_rodrigues_vector(orientation_estimate);

			const Eigen::Vector3d position_estimate= guess_transform->translation();
			cvCameraTVec= eigen_vector3d_to_cv_vector3d(position_estimate);

            bUseExtrinsicGuess= true;
        }

		// Use a full solvePnP on the given point to model correspondences to compute a transform
		if (cv::solvePnP(
				cvObjectPoints, cvImagePoints,
				cameraMatrix, distCoeffs,
				cvCameraRVec, cvCameraTVec,
				bUseExtrinsicGuess, cv::SOLVEPNP_ITERATIVE))
		{
			Eigen::Quaterniond orientation_estimate= cv_rodrigues_vector_to_eigen_quatd(cvCameraRVec);
			Eigen::Vector3d position_estimate= cv_vector3d_to_eigen_vector3d(cvCameraTVec);

			out_transform.linear().noalias() = orientation_estimate.toRotationMatrix();
			out_transform.translation().noalias() = position_estimate;

			return true;
		}
	}

	return false;
}

static bool compute_predicted_point_correspondences(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
	const std::vector<Eigen::Vector3f> &model_vertices,
	const Eigen::Affine3d *guess_transform,
	std::vector<MonoPointCorrespondence> &out_point_correspondences)
{
	if (guess_transform == nullptr)
		return false;

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

    // solvePnP expects a rotation as a Rodrigues (AngleAxis) vector
	Eigen::Quaterniond orientation_estimate= Eigen::Quaterniond(guess_transform->linear());
    cv_model_rvec= eigen_quatd_to_cv_rodrigues_vector(orientation_estimate);

	const Eigen::Vector3d position_estimate= guess_transform->translation();
	cv_model_tvec= eigen_vector3d_to_cv_vector3d(position_estimate);

	const size_t proj_point_count= projection.projections[0].shape.pointcloud.point_count;
	const size_t model_vertex_count= model_vertices.size();

	// Project the source point on to the solved camera plane
	// model pose = (rvec, tvec)
	const Eigen::Affine3f model_pose= guess_transform->cast<float>();
	std::vector<cv::Point3f> cv_model_points;
	for (const Eigen::Vector3f &vertex : model_vertices)
	{
		const Eigen::Vector3f camera_relative_point= model_pose * vertex;

		cv_model_points.push_back(cv::Point3d(camera_relative_point.x(), camera_relative_point.y(), camera_relative_point.z()));
	}

	std::vector<cv::Point2f> cv_projected_points;
	cv::projectPoints(cv_model_points, cv_model_rvec, cv_model_tvec, intrinsic_matrix, dist_coeffs, cv_projected_points);

	// Find the best correspondences based on the predicted projection
	std::vector<bool> proj_point_used(proj_point_count, false);
	out_point_correspondences.clear();
	for (int model_point_index = 0; model_point_index < model_vertex_count; ++model_point_index)
	{
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
		}
	}

	// Need at least 4 point correspondences to use SolvePnP in the next step
	return out_point_correspondences.size() >= 4;
}

// Algorithm originally from "A Monocular Pose Estimation System based on Infrared LEDs"
// by Matthias Faessler, Elias Mueggler, Karl Schwabe and Davide Scaramuzza
// See "Algorithm 1 - Correspondence Search" on page 4
// http://rpg.ifi.uzh.ch/docs/ICRA14_Faessler.pdf
static bool compute_brute_force_point_correspondences(
	const ServerTrackerView *tracker_view,
	const PSVRTrackingProjection &projection,
    const std::vector<Eigen::Vector3f> &model_vertices,
	const std::vector<t_tri_index_tuple> &model_tri_permutations,
	std::vector<MonoPointCorrespondence> &out_point_correspondences)
{
	cv::Mat *drawingBuffer= tracker_view->getDebugDrawingBuffer(PSVRVideoFrameSection_Primary);

	const size_t proj_point_count= projection.projections[0].shape.pointcloud.point_count;
	const size_t model_vertex_count= model_vertices.size();

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

	// Generate a list of all unique combinations of triangles from the "detections" in the projection
	std::vector<t_tri_index_tuple> proj_tri_combinations; // D = Detections
	compute_all_possible_tri_index_combinations(proj_point_count, proj_tri_combinations);

	PSVRVector3d min_pos, max_pos;
	int sample_count= 0;

	// Brute-Force build a correspondence histogram from projections points to model points
	cv::Mat_<int> histogram((int)proj_point_count, (int)model_vertex_count, 0);
	for (const t_tri_index_tuple &D : proj_tri_combinations)
	{
		for (const t_tri_index_tuple &L : model_tri_permutations)
		{
			// Use a test projection -> model triangle correspondence 
			// to compute a possible camera pose from solveP3P
			std::vector<cv::Mat> cv_model_rvecs;
			std::vector<cv::Mat> cv_model_tvecs;
			int solution_count= 0;
			{
				std::vector<cv::Point2f> cv_image_points;
				std::vector<cv::Point3f> cv_object_points;

				for (int tuple_index = 0; tuple_index < 3; ++tuple_index)
				{
					const int proj_point_index= D[tuple_index];
					const PSVRVector2f &proj_point = projection.projections[0].shape.pointcloud.points[proj_point_index];

					const int model_point_index= L[tuple_index];
					const Eigen::Vector3f &model_point= model_vertices[model_point_index];

					cv_image_points.push_back(cv::Point2f(proj_point.x, proj_point.y));
					cv_object_points.push_back(cv::Point3f(model_point.x(), model_point.y(), model_point.z()));
				}

				solution_count= 
					cv::solveP3P(
						cv_object_points, cv_image_points, 
						intrinsic_matrix, dist_coeffs, 
						cv_model_rvecs, cv_model_tvecs, 
						cv::SOLVEPNP_P3P);
			}

			// Tally up every model projection that gets close to a detection point in the histogram
			// using all of the camera pose solutions returned from solveP3P
			for (int solution_index = 0; solution_index < solution_count; ++solution_index)
			{
				bool bFound= false;

				cv::Mat rvec= cv_model_rvecs[solution_index];
				cv::Mat tvec= cv_model_tvecs[solution_index];

				PSVRVector3d sample= cv_vec3d_to_PSVR_vector3d(tvec);
				if (sample_count > 0)
				{
					min_pos.x= fmin(min_pos.x, sample.x);
					min_pos.y= fmin(min_pos.y, sample.y);
					min_pos.z= fmin(min_pos.z, sample.z);
					max_pos.x= fmax(max_pos.x, sample.x);
					max_pos.y= fmax(max_pos.y, sample.y);
					max_pos.z= fmax(max_pos.z, sample.z);
				}
				else
				{
					min_pos= max_pos= sample;
				}
				sample_count++;

				for (int model_point_index = 0; model_point_index < model_vertex_count; ++model_point_index)
				{
					// Only consider source points NOT being used in the P3P solve
					if (model_point_index == L[0] || model_point_index == L[1] || model_point_index == L[2])
						continue;

					// Project the source point on to the solved camera plane
					// camera pose = (rvec, tvec)
					const Eigen::Vector3f &model_point= model_vertices[model_point_index];
					std::vector<cv::Point3f> cv_model_points;
					cv_model_points.push_back(cv::Point3f(model_point.x(), model_point.y(), model_point.z()));

					std::vector<cv::Point2f> cv_projected_points;
					cv::projectPoints(cv_model_points, rvec, tvec, intrinsic_matrix, dist_coeffs, cv_projected_points);
					const PSVRVector2f model_proj_point= {cv_projected_points[0].x, cv_projected_points[0].y};

					// See if the projected source point is near any of the measured projected points
					for (int tuple_index = 0; tuple_index < 3; ++tuple_index)
					{
						const int proj_point_index= D[tuple_index];
						const PSVRVector2f &proj_point = projection.projections[0].shape.pointcloud.points[proj_point_index];
						const bool bIsInRange= 
							PSVR_Vector2fDistanceSquared(&model_proj_point, &proj_point) <= k_reprojection_correspondance_tolerance_px_sqrd;

						if (bIsInRange)
						{
							histogram(proj_point_index, model_point_index)= histogram(proj_point_index, model_point_index) + 1;
							bFound= true;
						}

						if (TrackerManagerConfig::are_debug_flags_enabled(PSMTrackerDebugFlags_trackingModel))
						{
							cv::drawMarker(
								*drawingBuffer, 
								cv::Point((int)model_proj_point.x, (int)model_proj_point.y), 
								bIsInRange ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 
								bIsInRange ? cv::MarkerTypes::MARKER_DIAMOND : cv::MarkerTypes::MARKER_SQUARE,
								bIsInRange ? (int)k_reprojection_correspondance_tolerance_px : 2);
						}
					}
				}

				if (bFound)
				{
					for (int tuple_index = 0; tuple_index < 3; ++tuple_index)
					{
						const int proj_point_index= D[tuple_index];
						const int model_point_index= L[tuple_index];

						histogram(proj_point_index, model_point_index)= histogram(proj_point_index, model_point_index) + 1;
					}
				}
			}
		}
	}

	// Search the histogram for the best correspondences
	{
		std::vector<bool> proj_point_used(proj_point_count, false);

		out_point_correspondences.clear();
		for (int model_point_index = 0; model_point_index < model_vertex_count; ++model_point_index)
		{
			MonoPointCorrespondence correspondence;
			correspondence.clear();
			correspondence.model_point_index= model_point_index;

			int best_correspondence_count= 0;
			for (int proj_point_index = 0; proj_point_index < proj_point_count; ++proj_point_index)
			{
				if (proj_point_used[proj_point_index])
					continue;

				int correspondence_count = histogram(proj_point_index, model_point_index);
				if (correspondence_count > best_correspondence_count)
				{
					correspondence.proj_point_index= proj_point_index;
					best_correspondence_count= correspondence_count;
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
			}
		}
	}

	// Need at least 4 point correspondences to use SolvePnP in the next step
	return out_point_correspondences.size() >= 4;
}