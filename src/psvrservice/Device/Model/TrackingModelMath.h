#ifndef TRACKING_MODEL_MATH_H
#define TRACKING_MODEL_MATH_H

// -- includes -----
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <vector>

// -- typedefs -----
typedef std::array<int, 3> t_tri_index_tuple;

// -- interface -----
bool compute_all_possible_tri_index_combinations(
	std::size_t point_count, 
	std::vector<t_tri_index_tuple> &out_triangle_indices);

bool compute_all_possible_tri_index_permutations(
	std::size_t point_count,
	std::vector<t_tri_index_tuple> &out_triangle_indices);

bool compute_all_visible_tri_index_combinations(
    const std::vector<Eigen::Vector3f> &points,
    const std::vector<Eigen::Vector3f> &normals,
    std::vector<t_tri_index_tuple> &out_triangle_indices);

bool compute_all_visible_tri_index_permutations(
    const std::vector<Eigen::Vector3f> &points,
    const std::vector<Eigen::Vector3f> &normals,
	std::vector<t_tri_index_tuple> &out_tri_index_permutations);

bool compute_all_possible_triangle_transforms_for_point_cloud(
    const std::vector<Eigen::Vector3f> &points,
    std::vector<t_tri_index_tuple> &out_triangle_indices,
    std::vector<Eigen::Affine3f> &out_triangle_basis_list);

bool compute_all_visible_triangle_transforms_for_point_cloud(
    const std::vector<Eigen::Vector3f> &points,
    const std::vector<Eigen::Vector3f> &normals,
    std::vector<t_tri_index_tuple> &out_triangle_indices,
    std::vector<Eigen::Affine3f> &out_triangle_basis_list);

void compute_triangle_transforms_for_triangles(
    const std::vector<t_tri_index_tuple> &indices,
    const std::vector<Eigen::Vector3f> &points,
    std::vector<Eigen::Affine3f> &out_triangle_basis_list);

#endif // TRACKING_MODEL_MATH_H

