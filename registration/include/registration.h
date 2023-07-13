#pragma once

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
class Registration
{
private:
	void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals);
public:
	Registration();
	~Registration();
	/**
	 * \brief Registration the two point cloud with the key points.
	 * \param source_keypoint The source key points.
	 * \param target_keypoint The target key points.
	 * \param source_cloud The source point cloud.
	 * \param target_cloud The target point cloud.
	 * \param icp1_maximum_iterations The maximum iterations of the first ICP.
	 * \param icp2_maximum_iterations The maximum iterations of the second ICP.
	 * \return 0
	 */
	int registration(std::string source_keypoint, std::string target_keypoint, std::string source_cloud, std::string target_cloud, int icp1_maximum_iterations, int icp2_maximum_iterations);
};