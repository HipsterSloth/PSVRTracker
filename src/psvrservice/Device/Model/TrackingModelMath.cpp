#include "TrackingModelMath.h"
#include "MathAlignment.h"

bool compute_all_possible_tri_index_combinations(
	std::size_t point_count, 
	std::vector<t_tri_index_tuple> &out_triangle_indices)
{
    out_triangle_indices.clear();

    if (point_count < 3)
        return false;

    for (int p0_index = 0; p0_index < point_count; ++p0_index)
    {
        for (int p1_index = p0_index + 1; p1_index < point_count; ++p1_index)
        {
            for (int p2_index = p1_index + 1; p2_index < point_count; ++p2_index)
            {
				out_triangle_indices.push_back({p0_index, p1_index, p2_index});
            }
        }
    }

	return true;
}

bool compute_all_possible_tri_index_permutations(
	std::size_t point_count,
	std::vector<t_tri_index_tuple> &out_triangle_indices)
{
	std::vector<t_tri_index_tuple> triangle_combinations;
	if (compute_all_possible_tri_index_combinations(point_count, triangle_combinations))
	{
		for (const t_tri_index_tuple &combo : triangle_combinations)
		{
			out_triangle_indices.push_back({combo[0], combo[1], combo[2]});
			out_triangle_indices.push_back({combo[0], combo[2], combo[1]});
			out_triangle_indices.push_back({combo[1], combo[0], combo[2]});
			out_triangle_indices.push_back({combo[1], combo[2], combo[0]});
			out_triangle_indices.push_back({combo[2], combo[0], combo[1]});
			out_triangle_indices.push_back({combo[2], combo[1], combo[0]});
		}

		return true;
	}

	return false;
}

bool compute_all_possible_triangle_transforms_for_point_cloud(
    const std::vector<Eigen::Vector3f> &points,
    std::vector<t_tri_index_tuple> &out_triangle_indices,
    std::vector<Eigen::Affine3f> &out_triangle_basis_list)
{
    const int point_count= static_cast<int>(points.size());

    out_triangle_basis_list.clear();
    out_triangle_indices.clear();

	if (compute_all_possible_tri_index_combinations(point_count, out_triangle_indices))
	{
	    compute_triangle_transforms_for_triangles(out_triangle_indices, points, out_triangle_basis_list);
		return true;
	}

	return false;
}

bool compute_all_visible_tri_index_combinations(
    const std::vector<Eigen::Vector3f> &points,
    const Eigen::Vector3f &center,
    std::vector<t_tri_index_tuple> &out_triangle_indices)
{
    const int point_count= static_cast<int>(points.size());

    out_triangle_indices.clear();

    if (point_count < 3)
        return false;

    for (int p0_index = 0; p0_index < point_count; ++p0_index)
    {
        const Eigen::Vector3f &p0= points[p0_index];
        const Eigen::Vector3f n0= p0 - center;

        for (int p1_index = p0_index + 1; p1_index < point_count; ++p1_index)
        {
            const Eigen::Vector3f &p1= points[p1_index];
            const Eigen::Vector3f n1= p1 - center;

            for (int p2_index = p1_index + 1; p2_index < point_count; ++p2_index)
            {
                const Eigen::Vector3f &p2= points[p2_index];
                const Eigen::Vector3f n2= p2 - center;

                if (n0.dot(n1) >= 0 && n0.dot(n2) >= 0)
                {
					out_triangle_indices.push_back({p0_index, p1_index, p2_index});
                }
            }
        }
    }

    return true;
}

bool compute_all_visible_tri_index_permutations(
    const std::vector<Eigen::Vector3f> &points,
    const Eigen::Vector3f &center,
	std::vector<t_tri_index_tuple> &out_triangle_indices)
{
	std::vector<t_tri_index_tuple> triangle_combinations;
	if (compute_all_visible_tri_index_combinations(points, center, triangle_combinations))
	{
		for (const t_tri_index_tuple &combo : triangle_combinations)
		{
			out_triangle_indices.push_back({combo[0], combo[1], combo[2]});
			out_triangle_indices.push_back({combo[0], combo[2], combo[1]});
			out_triangle_indices.push_back({combo[1], combo[0], combo[2]});
			out_triangle_indices.push_back({combo[1], combo[2], combo[0]});
			out_triangle_indices.push_back({combo[2], combo[0], combo[1]});
			out_triangle_indices.push_back({combo[2], combo[1], combo[0]});
		}

		return true;
	}

	return false;
}

bool compute_all_visible_triangle_transforms_for_point_cloud(
    const std::vector<Eigen::Vector3f> &points,
    const Eigen::Vector3f &center,
    std::vector<t_tri_index_tuple> &out_triangle_indices,
    std::vector<Eigen::Affine3f> &out_triangle_basis_list)
{
    out_triangle_basis_list.clear();

	if (compute_all_visible_tri_index_combinations(points, center, out_triangle_indices))
	{
		compute_triangle_transforms_for_triangles(out_triangle_indices, points, out_triangle_basis_list);

		return true;
	}

	return false;
}

void compute_triangle_transforms_for_triangles(
    const std::vector<t_tri_index_tuple> &indices,
    const std::vector<Eigen::Vector3f> &points,
    std::vector<Eigen::Affine3f> &out_triangle_basis_list)
{
    const int triangle_count= static_cast<int>(indices.size());
    const int point_count= static_cast<int>(points.size());

    const Eigen::Vector3f cloud_centroid= eigen_vector3f_compute_mean(points.data(), static_cast<int>(points.size()));

    out_triangle_basis_list.clear();

    for (int triangle_index = 0; triangle_index < triangle_count; triangle_index++)
    {
        int p0_index= indices[triangle_index][0];
        int p1_index= indices[triangle_index][1];
        int p2_index= indices[triangle_index][2];

        // Compute the transform of the triangle
        const Eigen::Vector3f &p0= points[p0_index];
        const Eigen::Vector3f &p1= points[p1_index];
        const Eigen::Vector3f &p2= points[p2_index];
        const Eigen::Vector3f tri_centroid= (p0+p1+p2) / 3.f;

        const Eigen::Vector3f forward_reference= 
            (point_count > 3) 
            ? tri_centroid - cloud_centroid
            : Eigen::Vector3f(0.f, 0.f, -1.f);

        Eigen::Vector3f forward= ((p1-p0).cross(p2-p0)).normalized();
        if (forward.dot(forward_reference) < 0)
            forward = -forward;

        const Eigen::Vector3f side= (forward.cross(Eigen::Vector3f(0.f, 1.f, 0.f))).normalized();
        const Eigen::Vector3f up= (side.cross(forward)).normalized();

        Eigen::Transform<float, 3, Eigen::Isometry>::LinearMatrixType tri_orientation; 
        tri_orientation << side[0], up[0], -forward[0],
                            side[1], up[1], -forward[1],
                            side[2], up[2], -forward[2];

        Eigen::Affine3f tri_transform;
        tri_transform.linear()= tri_orientation;
        tri_transform.translation()= tri_centroid;

        // Add the triangle basis to the list
        out_triangle_basis_list.push_back(tri_transform);
    }
}