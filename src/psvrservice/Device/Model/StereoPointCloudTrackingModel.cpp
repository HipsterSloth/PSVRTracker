// -- include -----
#include "StereoPointCloudTrackingModel.h"
#include "MathAlignment.h"
#include "MathTypeConversion.h"
#include "ServerTrackerView.h"
#include "TrackingModelMath.h"
#include "TrackerMath.h"

#include <vector>
#include <array>

//-- constants ---
static const float k_default_epipolar_correspondance_tolerance_px = 3.0f;
static const double k_max_allowed_icp_alignment_error= 10.0;

//-- private structures ----
typedef std::pair<int, int> t_point_index_pair;

struct StereoPixelData
{
    PSVRVector2f left_pixel;
    PSVRVector2f right_pixel;
	int left_pixel_projection_index; // index into PSVRTrackingProjectionData::shape.pointcloud.points
	int right_pixel_projection_index; // index into PSVRTrackingProjectionData::shape.pointcloud.points
	int best_correlated_model_point_index; // index in PSVRTrackingShape::shape.pointcloud.points
    float disparity;
    Eigen::Vector3f triangulated_point_cm;
};

struct StereoPointCloudTrackingModelState
{
    std::vector<StereoPixelData> lastTriangulatedPoints;

    std::vector<Eigen::Vector3f> sourceVertices;
    std::vector<Eigen::Affine3f> sourceTriangleBasisList;
    std::vector<int> sourceTriangleIndices;

    Eigen::Vector3dMatrix icpModelVertices;
    std::vector<Eigen::Vector3f> modelVertices;
	std::vector<Eigen::Vector3f> modelNormals;
    std::vector<Eigen::Affine3f> modelTriangleBasisList;
    std::vector<t_tri_index_tuple> modelTriangleIndices;

    Eigen::Affine3d modelToSourceTransform;
    bool bIsModelToSourceTransformValid;
};

//-- private methods -----
static bool triangulate_stereo_projection(
    const ServerTrackerView *tracker_view,
    const PSVRTrackingProjection &trackingProjection,
    StereoPointCloudTrackingModelState *state);
static bool find_best_2d_point_cloud_correlation(
    const PSVRTrackerIntrinsics *tracker_intrinsics,
    const PSVRTrackingProjection *trackingProjection,
    std::vector<int> &bestLeftToRightPointCorrespondence);
static void find_all_epipolar_correlations(
    const PSVRTrackerIntrinsics *tracker_intrinsics,
    const PSVRTrackingProjection *tracking_projection,
    std::vector<t_point_index_pair> &out_epipolar_pairs);
static int align_2d_point_clouds_using_correlation(
    const PSVRTrackingProjection *trackingProjection,
    const std::vector<t_point_index_pair> &epipolar_pairs,
    const t_point_index_pair &test_correlation,
    std::vector<int> &out_left_to_right_point_correspondence,
    float &out_matching_error);

static void compute_transform_using_triangle_correlation(
    StereoPointCloudTrackingModelState *state);
static int align_3d_point_clouds_using_correlation(
    const StereoPointCloudTrackingModelState *state,
    const Eigen::Affine3f &source_basis,
    const Eigen::Affine3f &target_basis,
    std::vector<int> &out_source_to_target_correspondence,
    float &out_matching_error);
static int find_best_3d_point_cloud_correlation(
    StereoPointCloudTrackingModelState *state,
    std::vector<int> &best_source_to_model_point_correspondence,
    float *out_best_correlation_error);

//-- public implementation -----
StereoPointCloudTrackingModel::StereoPointCloudTrackingModel() :
    m_state(new StereoPointCloudTrackingModelState)
{
    m_state->modelToSourceTransform= Eigen::Affine3d::Identity();
    m_state->bIsModelToSourceTransformValid= false;
}

StereoPointCloudTrackingModel::~StereoPointCloudTrackingModel()
{
    delete m_state;
}

bool StereoPointCloudTrackingModel::init(PSVRTrackingShape *tracking_shape)
{
    bool bSuccess= false;

    if (tracking_shape->shape_type == PSVRTrackingShape_PointCloud &&
        tracking_shape->shape.pointcloud.point_count >= 3)
    {
        const int model_point_count= tracking_shape->shape.pointcloud.point_count;

        m_state->modelToSourceTransform = Eigen::Affine3d::Identity();

        m_state->modelVertices.resize(model_point_count);
		m_state->modelNormals.resize(model_point_count);
        for (int source_index = 0; source_index < model_point_count; ++source_index)
        {
            const PSVRVector3f &point= tracking_shape->shape.pointcloud.points[source_index];
			const PSVRVector3f &normal= tracking_shape->shape.pointcloud.normals[source_index];

            m_state->modelVertices[source_index]= Eigen::Vector3f(point.x, point.y, point.z);
			m_state->modelNormals[source_index]= Eigen::Vector3f(normal.x, normal.y, normal.z);
        }
        compute_all_visible_triangle_transforms_for_point_cloud(
            m_state->modelVertices,
            m_state->modelNormals,
            m_state->modelTriangleIndices, 
            m_state->modelTriangleBasisList);

        m_state->icpModelVertices.resize(Eigen::NoChange, tracking_shape->shape.pointcloud.point_count);
        for (int source_index = 0; source_index < tracking_shape->shape.pointcloud.point_count; ++source_index)
        {
            const PSVRVector3f &point= tracking_shape->shape.pointcloud.points[source_index];

            m_state->icpModelVertices(0, source_index) = point.x;
            m_state->icpModelVertices(1, source_index) = point.y;
            m_state->icpModelVertices(2, source_index) = point.z;
        }

        bSuccess= true;
    }

    return bSuccess;
}

bool StereoPointCloudTrackingModel::applyShapeProjectionFromTracker(
	const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
    const class ServerTrackerView *tracker_view,
    const PSVRTrackingProjection &projection)
{
    assert(projection.projection_count == STEREO_PROJECTION_COUNT);

    bool bSuccess= false;

    if (triangulate_stereo_projection(tracker_view, projection, m_state))
    {
        compute_transform_using_triangle_correlation(m_state);
        bSuccess= true;
    }

    return bSuccess;
}

bool StereoPointCloudTrackingModel::getShapeOrientation(PSVRQuatf &out_orientation) const
{
    if (m_state->bIsModelToSourceTransformValid)
    {
        Eigen::Quaternionf q= Eigen::Quaternionf(
            m_state->modelToSourceTransform.linear().cast<float>());

        out_orientation= eigen_quaternionf_to_PSVR_quatf(q);
    }

    return m_state->bIsModelToSourceTransformValid;
}

bool StereoPointCloudTrackingModel::getShapePosition(PSVRVector3f &out_position) const
{
    if (m_state->bIsModelToSourceTransformValid)
    {
        out_position=
            eigen_vector3f_to_PSVR_vector3f(
                m_state->modelToSourceTransform.translation().cast<float>());
    }

    return m_state->bIsModelToSourceTransformValid;
}

bool StereoPointCloudTrackingModel::getShape(PSVRTrackingShape &out_shape) const
{
    if (m_state->bIsModelToSourceTransformValid)
    {
        const int point_count= static_cast<int>(m_state->modelVertices.size());

        out_shape.shape_type= PSVRTrackingShape_PointCloud;
        out_shape.shape.pointcloud.point_count= point_count;
        for (int point_index = 0; point_index < point_count; ++point_index)
        {
            const Eigen::Vector3d model_vertex= m_state->modelVertices[point_index].cast<double>();
            const Eigen::Vector3f world_vertex= (m_state->modelToSourceTransform * model_vertex).cast<float>();

            out_shape.shape.pointcloud.points[point_index]= eigen_vector3f_to_PSVR_vector3f(world_vertex);
        }
    }

    return m_state->bIsModelToSourceTransformValid;
}

bool StereoPointCloudTrackingModel::getPointCloudProjectionShapeCorrelation(PSVRTrackingProjection &projection) const
{
	assert(projection.shape_type == PSVRShape_PointCloud);

	if (m_state->bIsModelToSourceTransformValid)
    {
		for (const StereoPixelData &pixel_pair : m_state->lastTriangulatedPoints)
		{
			if (pixel_pair.left_pixel_projection_index != -1 &&
				pixel_pair.right_pixel_projection_index != -1 &&
				pixel_pair.best_correlated_model_point_index != -1)
			{
				projection.projections[LEFT_PROJECTION_INDEX].shape.pointcloud.shape_point_index[pixel_pair.left_pixel_projection_index]= 
					pixel_pair.best_correlated_model_point_index;
				projection.projections[RIGHT_PROJECTION_INDEX].shape.pointcloud.shape_point_index[pixel_pair.right_pixel_projection_index]= 
					pixel_pair.best_correlated_model_point_index;
			}
		}
    }

    return m_state->bIsModelToSourceTransformValid;
}

//-- private implementation -----
static bool triangulate_stereo_projection(
    const ServerTrackerView *tracker_view,
    const PSVRTrackingProjection &trackingProjection,
    StereoPointCloudTrackingModelState *state)
{
    state->lastTriangulatedPoints.clear();
        
    assert(trackingProjection.shape_type == PSVRShape_PointCloud);
    assert(trackingProjection.projection_count == STEREO_PROJECTION_COUNT);

    // Undistorted/rectified points
    const PSVRVector2f *leftPoints = trackingProjection.projections[LEFT_PROJECTION_INDEX].shape.pointcloud.points;
    const PSVRVector2f *rightPoints = trackingProjection.projections[RIGHT_PROJECTION_INDEX].shape.pointcloud.points;
    const int leftPointCount = trackingProjection.projections[LEFT_PROJECTION_INDEX].shape.pointcloud.point_count;
    const int rightPointCount = trackingProjection.projections[RIGHT_PROJECTION_INDEX].shape.pointcloud.point_count;

    // Get the "Stereo Reprojection Matrix" Q used to triangulate a pixel using stereo disparity
    PSVRTrackerIntrinsics tracker_intrinsics;
    tracker_view->getCameraIntrinsics(tracker_intrinsics);
    assert(tracker_intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS);
    Eigen::Matrix4d Q= PSVR_matrix4d_to_eigen_matrix4d(tracker_intrinsics.intrinsics.stereo.reprojection_matrix);

    // Find the best fit correlation between the left and right point clouds
    std::vector<int> bestLeftToRightPointCorrespondence;
    if (find_best_2d_point_cloud_correlation(
            &tracker_intrinsics,
            &trackingProjection,
            bestLeftToRightPointCorrespondence))
    {
        // Use the correspondence table to make correlated-pixel-pairs
        // for every pair that satisfies the epipolar distance constraint
        for (int leftPointIndex = 0; leftPointIndex < leftPointCount; ++leftPointIndex)
        {
            const int rightPointIndex= bestLeftToRightPointCorrespondence[leftPointIndex];
            if (rightPointIndex == -1)
                continue;
                    
            const PSVRVector2f &leftPoint = leftPoints[leftPointIndex];
            const cv::Mat cvLeftPoint = cv::Mat(cv::Point2f(leftPoint.x, leftPoint.y));

            const PSVRVector2f &rightPoint = rightPoints[rightPointIndex];
            cv::Mat cvRightPoint = cv::Mat(cv::Point2f(rightPoint.x, rightPoint.y));

            // Compute the horizontal pixel disparity between the left and right corresponding pixels
            const double disparity= (double)(leftPoint.x - rightPoint.x);

            if (fabs(disparity) > k_real64_epsilon)
            {
                // Project the left pixel + disparity into the world using
                // the projection matrix 'Q' computed during stereo calibration
                Eigen::Vector4d pixel((double)leftPoint.x, (double)leftPoint.y, disparity, 1.0);
                Eigen::Vector4d homogeneus_point= Q * pixel;

                // Get the triangulated 3d position
                const double w = homogeneus_point.w();

                if (fabs(w) > k_real64_epsilon)
                {
                    StereoPixelData pair;
                    pair.left_pixel= leftPoint;
                    pair.right_pixel= rightPoint;
					pair.left_pixel_projection_index= leftPointIndex;
					pair.right_pixel_projection_index= rightPointIndex;
					pair.best_correlated_model_point_index= -1;
                    pair.disparity= (float)disparity;

                    // Q matrix transforms pixels to tracker relative positions in millimeters
                    Eigen::Vector3f triangulated_point_mm= Eigen::Vector3f(
                        (float)(homogeneus_point.x() / w),
                        (float)(-homogeneus_point.y() / w), // Q matrix has flipped Y-axis
                        (float)(homogeneus_point.z() / w));
                    pair.triangulated_point_cm= triangulated_point_mm * PSVR_MILLIMETERS_TO_CENTIMETERS;

                    // Add to the list of world space points we saw this frame
                    state->lastTriangulatedPoints.push_back(pair);
                }
            }
        }
    }

    return state->lastTriangulatedPoints.size() >= 3;
}

static bool find_best_2d_point_cloud_correlation(
    const PSVRTrackerIntrinsics *tracker_intrinsics,
    const PSVRTrackingProjection *trackingProjection,
    std::vector<int> &bestLeftToRightPointCorrespondence)
{
    std::vector<t_point_index_pair> epipolar_pairs;
    find_all_epipolar_correlations(tracker_intrinsics, trackingProjection, epipolar_pairs);

    bool bFoundCorrelation= false;
    float bestCorrelationError= k_real_max;
    int bestMatchCount= 0;
    for (const t_point_index_pair &test_epipolar_pair : epipolar_pairs)
    {
        std::vector<int> testLeftToRightPointCorrespondence;
        float correlationError= 0.f;
        int matchCount = align_2d_point_clouds_using_correlation(
            trackingProjection,
            epipolar_pairs,
            test_epipolar_pair,
            testLeftToRightPointCorrespondence,
            correlationError);

        if (matchCount > bestMatchCount ||
            (matchCount == bestMatchCount && correlationError < bestCorrelationError))
        {
            bestLeftToRightPointCorrespondence= testLeftToRightPointCorrespondence;
            bestCorrelationError= correlationError;
            bestMatchCount= matchCount;
            bFoundCorrelation= true;
        }
    }

    return bFoundCorrelation;
}

static void find_all_epipolar_correlations(
    const PSVRTrackerIntrinsics *tracker_intrinsics,
    const PSVRTrackingProjection *tracking_projection,
    std::vector<t_point_index_pair> &out_epipolar_pairs)
{
    // Undistorted/rectified points
    const PSVRVector2f *leftPoints = tracking_projection->projections[LEFT_PROJECTION_INDEX].shape.pointcloud.points;
    const PSVRVector2f *rightPoints = tracking_projection->projections[RIGHT_PROJECTION_INDEX].shape.pointcloud.points;
    const int leftPointCount = tracking_projection->projections[LEFT_PROJECTION_INDEX].shape.pointcloud.point_count;
    const int rightPointCount = tracking_projection->projections[RIGHT_PROJECTION_INDEX].shape.pointcloud.point_count;

    // Get the "Fundamental Matrix" for the stereo camera computed during camera calibration
    Eigen::Matrix3f F_ab= PSVR_matrix3d_to_eigen_matrix3f(tracker_intrinsics->intrinsics.stereo.fundamental_matrix);

    // For each point in one tracking projection A, 
    // try and find the corresponding point in the projection B
    for (int leftPointIndex = 0; leftPointIndex < leftPointCount; ++leftPointIndex)
    {
        const PSVRVector2f &leftPoint = leftPoints[leftPointIndex];
        const Eigen::Vector3f a(leftPoint.x, leftPoint.y, 1.f);

        for (int rightPointIndex = 0; rightPointIndex < rightPointCount; ++rightPointIndex)
        {
            const PSVRVector2f &rightPoint = rightPoints[rightPointIndex];
            const Eigen::Vector3f b(rightPoint.x, rightPoint.y, 1.f);

            //See if image point A * Fundamental Matrix * image point B <= tolerance
            const float epipolar_distance = fabsf(a.transpose() * F_ab * b);
            if (epipolar_distance <= k_default_epipolar_correspondance_tolerance_px)
            {
                out_epipolar_pairs.push_back(t_point_index_pair(leftPointIndex, rightPointIndex));
            }
        }
    }
}

static int align_2d_point_clouds_using_correlation(
    const PSVRTrackingProjection *trackingProjection,
    const std::vector<t_point_index_pair> &epipolar_pairs,
    const t_point_index_pair &test_correlation,
    std::vector<int> &out_left_to_right_point_correspondence,
    float &out_matching_error)
{
    // Undistorted/rectified points
    const PSVRVector2f *leftPoints = trackingProjection->projections[LEFT_PROJECTION_INDEX].shape.pointcloud.points;
    const PSVRVector2f *rightPoints = trackingProjection->projections[RIGHT_PROJECTION_INDEX].shape.pointcloud.points;
    const int leftPointCount = trackingProjection->projections[LEFT_PROJECTION_INDEX].shape.pointcloud.point_count;
    const int rightPointCount = trackingProjection->projections[RIGHT_PROJECTION_INDEX].shape.pointcloud.point_count;

    // Treat the correlation points to the origin
    const PSVRVector2f leftOrigin= leftPoints[test_correlation.first];
    const PSVRVector2f rightOrigin= rightPoints[test_correlation.second];

    // Recenter the projection points about the origin
    PSVRVector2f recenteredLeftPoints[MAX_POINT_CLOUD_POINT_COUNT];
    PSVRVector2f recenteredRightPoints[MAX_POINT_CLOUD_POINT_COUNT];
    PSVR_Vector2fArrayTranslate(leftPoints, leftPointCount, &leftOrigin, -1.f, recenteredLeftPoints);
    PSVR_Vector2fArrayTranslate(rightPoints, rightPointCount, &rightOrigin, -1.f, recenteredRightPoints);

    // Initialize the correspondence tables
    std::vector<int> left_to_right_point_correspondence(leftPointCount);
    std::vector<int> right_to_left_point_correspondence(rightPointCount);
    std::vector<float> point_distance(leftPointCount);
    std::fill(left_to_right_point_correspondence.begin(), left_to_right_point_correspondence.end(), -1);
    std::fill(right_to_left_point_correspondence.begin(), right_to_left_point_correspondence.end(), -1);
    std::fill(point_distance.begin(), point_distance.end(), k_real_max);

    // For each point in the left tracking projection, 
    // try and find the corresponding point in the right tracking projection
    for (int leftPointIndex = 0; leftPointIndex < leftPointCount; ++leftPointIndex)
    {
        const PSVRVector2f recenteredLeftPoint = recenteredLeftPoints[leftPointIndex];

        // Find the closest point on the same epipolar line
        int bestRightPointIndex= -1;
        PSVRVector2f bestRightPoint;
        float bestSqrdDist= k_real_max;
        for (const t_point_index_pair &epipolar_pair : epipolar_pairs)
        {
            if (epipolar_pair.first == leftPointIndex)
            {
                const int rightPointIndex= epipolar_pair.second;
                const PSVRVector2f recenteredRightPoint = recenteredRightPoints[rightPointIndex];
                const float sqrdDist= PSVR_Vector2fDistanceSquared(&recenteredLeftPoint, &recenteredRightPoint);

                if (sqrdDist < bestSqrdDist)
                {
                    bestRightPointIndex= rightPointIndex;
                    bestRightPoint= recenteredRightPoint;
                    bestSqrdDist= sqrdDist;
                }
            }
        }

        if (bestRightPointIndex != -1)
        {
            // Associate the left point with the right point
            left_to_right_point_correspondence[leftPointIndex]= bestRightPointIndex;
            point_distance[leftPointIndex]= bestSqrdDist;
        }
    }

    // For each point in the right tracking projection, 
    // try and find the corresponding point in the left tracking projection
    for (int rightPointIndex = 0; rightPointIndex < rightPointCount; ++rightPointIndex)
    {
        const PSVRVector2f recenteredRightPoint = recenteredRightPoints[rightPointIndex];

        // Find the closest point on the same epipolar line
        int bestLeftPointIndex= -1;
        float bestSqrdDist= k_real_max;
        for (const t_point_index_pair &epipolar_pair : epipolar_pairs)
        {
            if (epipolar_pair.second == rightPointIndex)
            {
                const int leftPointIndex= epipolar_pair.first;
                const PSVRVector2f recenteredLeftPoint = recenteredLeftPoints[leftPointIndex];
                const float sqrdDist= PSVR_Vector2fDistanceSquared(&recenteredLeftPoint, &recenteredRightPoint);

                if (sqrdDist < bestSqrdDist)
                {
                    bestLeftPointIndex= leftPointIndex;
                    bestSqrdDist= sqrdDist;
                }
            }
        }

        if (bestLeftPointIndex != -1)
        {
            // Associate the left point with the right point
            right_to_left_point_correspondence[rightPointIndex]= bestLeftPointIndex;
        }
    }

    // Only consider left and right points in correspondence
    // if they agree they correspond with each other
    int matched_points= 0;
    out_matching_error= 0.f;
    out_left_to_right_point_correspondence.resize(leftPointCount);
    for (int leftPointIndex = 0; leftPointIndex < leftPointCount; ++leftPointIndex)
    {
        const int rightPointIndex= left_to_right_point_correspondence[leftPointIndex];
            
        if (rightPointIndex != -1 && 
            right_to_left_point_correspondence[rightPointIndex] == leftPointIndex)
        {
            out_left_to_right_point_correspondence[leftPointIndex]= rightPointIndex;
            out_matching_error+= point_distance[leftPointIndex];
            matched_points++;
        }
        else
        {
            out_left_to_right_point_correspondence[leftPointIndex]= -1;
        }
    }

    return matched_points;
}

static void compute_transform_using_triangle_correlation(
    StereoPointCloudTrackingModelState *state)
{
    const int source_point_count = static_cast<int>(state->lastTriangulatedPoints.size());

    // Extract the source points into an array    
    state->sourceVertices.clear();
    std::for_each(state->lastTriangulatedPoints.begin(), state->lastTriangulatedPoints.end(),
        [state](const StereoPixelData &correlated_pixel_pair){
            state->sourceVertices.push_back(correlated_pixel_pair.triangulated_point_cm);
        });

    // Find the best correspondence from source point to model point
    std::vector<int> best_source_to_model_point_correspondence;
    int point_match_count= 0;
    {
        float align_matching_error= k_real_max;

        if (state->bIsModelToSourceTransformValid)
        {
            // Use the most recent transform to find a correspondence
            point_match_count= align_3d_point_clouds_using_correlation(
                state,
                state->modelToSourceTransform.cast<float>(), 
                Eigen::Affine3f::Identity(),
                best_source_to_model_point_correspondence,
                align_matching_error);
        }

        if (source_point_count >= 3 && align_matching_error > k_max_allowed_icp_alignment_error)
        {
            std::vector<int> alt_best_source_to_model_point_correspondence;
            float alt_align_matching_error;

            // Brute force search for best triangle on the source point cloud
            // to align with a triangle on the model point cloud
            int alt_point_match_count=
                find_best_3d_point_cloud_correlation(
                    state, 
                    alt_best_source_to_model_point_correspondence,
                    &alt_align_matching_error);

            if (alt_point_match_count >= point_match_count &&
                alt_align_matching_error < align_matching_error)
            {
                point_match_count= alt_point_match_count;
                best_source_to_model_point_correspondence= alt_best_source_to_model_point_correspondence;
                align_matching_error= alt_align_matching_error;
            }
        }
    }

    // Copy the source vertices and their corresponding model vertices into parallel arrays
    if (point_match_count > 0)
    {
        Eigen::Vector3dMatrix icp_source_vertices;
        Eigen::Vector3dMatrix icp_corresponding_model_vertices;
        {
            int write_col_index= 0;

            icp_source_vertices.resize(Eigen::NoChange, point_match_count);
            icp_corresponding_model_vertices.resize(Eigen::NoChange, point_match_count);

            for (int source_point_index = 0; source_point_index < source_point_count; ++source_point_index)
            {
                // Find the closest vertices on the model to aligned source vertices
                const int corresponding_model_point_index = 
                    best_source_to_model_point_correspondence[source_point_index];

                if (corresponding_model_point_index != -1)
                {
                    const Eigen::Vector3f &point = 
                        state->lastTriangulatedPoints[source_point_index].triangulated_point_cm;

                    icp_source_vertices(0, write_col_index) = point.x();
                    icp_source_vertices(1, write_col_index) = point.y();
                    icp_source_vertices(2, write_col_index) = point.z();

                    icp_corresponding_model_vertices.col(write_col_index)=
                        state->icpModelVertices.col(corresponding_model_point_index);

                    ++write_col_index;

					// Also keep track of which model point we believe the projections correspond to
					state->lastTriangulatedPoints[source_point_index].best_correlated_model_point_index= 
						corresponding_model_point_index;
                }
            }
        }

        // Compute the rigid transform from model vertices to source vertices
        if (point_match_count >= 3)
        {
            state->modelToSourceTransform= 
                eigen_alignment_compute_point_to_point_transform(
                    icp_corresponding_model_vertices, icp_source_vertices);
            state->bIsModelToSourceTransformValid= true;
        }
        else if (point_match_count == 2)
        {
            const Eigen::Vector3d source_center= 
                (icp_source_vertices.col(0) + icp_source_vertices.col(1))*0.5f;
            const Eigen::Vector3d model_center= 
                (icp_corresponding_model_vertices.col(0) + icp_corresponding_model_vertices.col(1))*0.5f;
            const Eigen::Vector3d translate_model_to_source= source_center - model_center;

            const Eigen::Vector3d source_vector= 
                icp_source_vertices.col(1) - icp_source_vertices.col(0);
            const Eigen::Vector3d model_vector= 
                icp_corresponding_model_vertices.col(1) - icp_corresponding_model_vertices.col(0);
            const Eigen::Quaterniond rotate_model_to_source= Eigen::Quaterniond::FromTwoVectors(model_vector, source_vector);

            state->modelToSourceTransform= Eigen::Affine3d::Identity();
            state->modelToSourceTransform.translation()= translate_model_to_source;
            state->modelToSourceTransform.linear()= rotate_model_to_source.toRotationMatrix();
            state->bIsModelToSourceTransformValid= true;
        }
        else if (point_match_count == 1)
        {
            const Eigen::Vector3d source_center= icp_source_vertices.col(0);
            const Eigen::Vector3d model_center= icp_corresponding_model_vertices.col(0);
            const Eigen::Vector3d translate_model_to_source= source_center - model_center;

            state->modelToSourceTransform= Eigen::Affine3d::Identity();
            state->modelToSourceTransform.translation()= translate_model_to_source;
            state->bIsModelToSourceTransformValid= true;
        }
    }
    else
    {
        state->modelToSourceTransform= Eigen::Affine3d::Identity();
        state->bIsModelToSourceTransformValid= false;
    }
}

static int align_3d_point_clouds_using_correlation(
    const StereoPointCloudTrackingModelState *state,
    const Eigen::Affine3f &source_basis,
    const Eigen::Affine3f &target_basis,
    std::vector<int> &out_source_to_target_correspondence,
    float &out_matching_error)
{
    const int source_point_count= static_cast<int>(state->sourceVertices.size());
    const int model_point_count= static_cast<int>(state->modelVertices.size());

    const Eigen::Affine3f inv_source_basis= source_basis.inverse();
    const Eigen::Affine3f inv_target_basis= target_basis.inverse();

    std::vector<Eigen::Vector3f> recentered_source_points;
    std::for_each(state->sourceVertices.begin(), state->sourceVertices.end(), 
        [&recentered_source_points, &inv_source_basis](const Eigen::Vector3f &v){
            recentered_source_points.push_back(inv_source_basis * v);
        });

    std::vector<Eigen::Vector3f> recentered_model_points;
    std::for_each(state->modelVertices.begin(), state->modelVertices.end(), 
        [&recentered_model_points, &inv_target_basis](const Eigen::Vector3f &v){
            recentered_model_points.push_back(inv_target_basis * v);
        });

    // Initialize the correspondence tables
    std::vector<int> source_to_model_point_correspondence(source_point_count);
    std::vector<int> model_to_source_point_correspondence(model_point_count);
    std::vector<float> point_distance(source_point_count);
    std::fill(source_to_model_point_correspondence.begin(), source_to_model_point_correspondence.end(), -1);
    std::fill(model_to_source_point_correspondence.begin(), model_to_source_point_correspondence.end(), -1);
    std::fill(point_distance.begin(), point_distance.end(), k_real_max);

    // For each source point try and find the corresponding model point
    for (int source_point_index = 0; source_point_index < source_point_count; ++source_point_index)
    {
        const Eigen::Vector3f &recentered_source_point = recentered_source_points[source_point_index];

        // Find the closest point in the model points
        int best_model_point_index= -1;
        Eigen::Vector3f best_model_point;
        float best_sqrd_dist= k_real_max;
        for (int model_point_index= 0; model_point_index < model_point_count; ++model_point_index)
        {
            const Eigen::Vector3f &recentered_model_point = recentered_model_points[model_point_index];
            const float sqrd_dist= (recentered_source_point - recentered_model_point).squaredNorm();

            if (sqrd_dist < best_sqrd_dist)
            {
                best_model_point_index= model_point_index;
                best_model_point= recentered_model_point;
                best_sqrd_dist= sqrd_dist;
            }
        }

        if (best_model_point_index != -1)
        {
            // Associate the left point with the right point
            source_to_model_point_correspondence[source_point_index]= best_model_point_index;
            point_distance[source_point_index]= best_sqrd_dist;
        }
    }

    // For each model point try and find the corresponding source point
    for (int model_point_index = 0; model_point_index < model_point_count; ++model_point_index)
    {
        const Eigen::Vector3f &recentered_model_point = recentered_model_points[model_point_index];

        // Find the closest point in the source points
        int bestsource_point_index= -1;
        float bestSqrdDist= k_real_max;
        for (int source_point_index = 0; source_point_index < source_point_count; ++source_point_index)
        {
            const Eigen::Vector3f &recentered_source_point = recentered_source_points[source_point_index];
            const float sqrd_dist= (recentered_source_point - recentered_model_point).squaredNorm();

            if (sqrd_dist < bestSqrdDist)
            {
                bestsource_point_index= source_point_index;
                bestSqrdDist= sqrd_dist;
            }
        }

        if (bestsource_point_index != -1)
        {
            // Associate the left point with the right point
            model_to_source_point_correspondence[model_point_index]= bestsource_point_index;
        }
    }

    // Only consider left and right points in correspondence
    // if they agree they correspond with each other
    int matched_points= 0;
    out_matching_error= 0.f;
    out_source_to_target_correspondence.resize(source_point_count);
    for (int source_point_index = 0; source_point_index < source_point_count; ++source_point_index)
    {
        const int model_point_index= source_to_model_point_correspondence[source_point_index];
            
        if (model_point_index != -1 && 
            model_to_source_point_correspondence[model_point_index] == source_point_index)
        {
            out_source_to_target_correspondence[source_point_index]= model_point_index;
            out_matching_error+= point_distance[source_point_index];
            matched_points++;
        }
        else
        {
            out_source_to_target_correspondence[source_point_index]= -1;
        }
    }

    return matched_points;
}

static int find_best_3d_point_cloud_correlation(
    StereoPointCloudTrackingModelState *state,
    std::vector<int> &best_source_to_model_point_correspondence,
    float *out_best_correlation_error)
{
    bool found_correlation= false;
    float best_correlation_error= k_real_max;
    int best_match_count= 0;

    std::vector<t_tri_index_tuple> source_triangle_indices;
    std::vector<Eigen::Affine3f> source_triangle_basis_list;

    if (compute_all_possible_triangle_transforms_for_point_cloud(
            state->sourceVertices, source_triangle_indices, source_triangle_basis_list))
    {
        for (int source_triangle_index= 0; 
            source_triangle_index < source_triangle_basis_list.size();
            ++source_triangle_index)
        {
            const Eigen::Affine3f &souce_basis= source_triangle_basis_list[source_triangle_index];

            for (int model_triangle_index= 0; 
                model_triangle_index < state->modelTriangleBasisList.size(); 
                ++model_triangle_index)
            {
                const Eigen::Affine3f &model_basis= state->modelTriangleBasisList[model_triangle_index];
                std::vector<int> testLeftToRightPointCorrespondence;
                float correlationError= 0.f;

                int match_count= align_3d_point_clouds_using_correlation(
                    state,
                    souce_basis, 
                    model_basis,
                    testLeftToRightPointCorrespondence,
                    correlationError);

                if (match_count >= 3 &&
                    (match_count > best_match_count ||
                    (match_count == best_match_count && correlationError < best_correlation_error)))
                {
                    best_source_to_model_point_correspondence= testLeftToRightPointCorrespondence;
                    best_correlation_error= correlationError;
                    best_match_count= match_count;
                }
            }
        }
    }

    if (out_best_correlation_error)
    {
        *out_best_correlation_error= best_correlation_error;
    }

    return best_match_count;
}