#include "registration.h"

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
int Registration::registration(std::string source_keypoint,std::string target_keypoint,std::string source_cloud,std::string target_cloud,int icp1_maximum_iterations,int icp2_maximum_iterations)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceKeypoint(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetKeypoint(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceKeypointAligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr sourceAligned(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(source_keypoint.c_str(), *sourceKeypoint);
	pcl::io::loadPLYFile(target_keypoint.c_str(), *targetKeypoint);
	pcl::io::loadPLYFile(source_cloud.c_str(), *source);
	pcl::io::loadPLYFile(target_cloud.c_str(), *target);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp1;

	// Set the input source and target
	icp1.setInputSource(sourceKeypoint);
	icp1.setInputTarget(targetKeypoint);

	icp1.setMaximumIterations(icp1_maximum_iterations);

	// Perform the alignment
	icp1.align(*sourceKeypointAligned);
	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4f transformation1 = icp1.getFinalTransformation();

	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceTransform1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source, *sourceTransform1, transformation1);

	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(sourceTransform1, source_with_normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(target, target_with_normals);

	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp2;
	icp2.setInputSource(source_with_normals);
	icp2.setInputTarget(target_with_normals);

	icp2.setMaximumIterations(icp2_maximum_iterations);

	// Perform the alignment
	icp2.align(*sourceAligned);

	pcl::io::savePLYFile("face_camera_icp_aligned.ply", *sourceAligned);
	return 0;
}

void Registration::cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
	//-----------------拼接点云数据与法线信息---------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//设置openMP的线程数
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	n.compute(*normals);//开始进行法向计
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}

Registration::Registration()
{
	
}

Registration::~Registration()
{
	
}
