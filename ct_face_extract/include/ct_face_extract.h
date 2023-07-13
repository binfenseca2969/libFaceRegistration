// SkinExtract.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once
#include <vtkClipPolyData.h>
#include <vtkFeatureEdges.h>
#include <vtkNamedColors.h>
#include <vtkSmartPointer.h>
#include <vtkPlane.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <itkImage.h>
#include <itkGDCMImageIO.h>
#include <itkGDCMSeriesFileNames.h>
#include <itkImageSeriesReader.h>
#include <itkImageToVTKImageFilter.h>
#include <vtkMarchingCubes.h>
#include <vtkLineSource.h>
#include <vtkOBBTree.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPoints.h>
#include <vtkPLYWriter.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/io.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkContourFilter.h>
#include <vtkOBJReader.h>
#include <vtkOBJWriter.h>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing.h>
#include <dlib/opencv.h>
#include <direct.h>
#include <corecrt_io.h>

class CtFaceExtract
{
private:
	void obj_reader(const std::string& obj_file, std::vector<float>& x, std::vector<float>& y, std::vector<float>& z,
		std::vector<float>& alpha, std::vector<float>& beta, std::vector<float>& theta);
	void point2gray(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z,
		const std::vector<float>& alpha, const std::vector<float>& beta, const std::vector<float>& theta);
	cv::Mat img_norm(const cv::Mat& img);
	std::vector<dlib::full_object_detection> GetShapes(cv::Mat& image, std::vector<dlib::rectangle>& dets);
	void DrawPoint(std::vector<dlib::full_object_detection>& faces, cv::Mat& image);
public:
	CtFaceExtract();
	~CtFaceExtract();
	/**
	 * \brief Extract the face mesh from the DICOM file.
	 * \param dirName The directory of the DICOM file.
	 * \param min The minimum threshold of MarchingCube.
	 * \param max The maximum threshold of MarchingCube.
	 * \param x X coordinates of the split plane.
	 * \param y Y coordinates of the split plane.
	 * \param z Z coordinates of the split plane.
	 * \param ax X coordinates of the normal vector of the split plane.
	 * \param ay Y coordinates of the normal vector of the split plane.
	 * \param az Z coordinates of the normal vector of the split plane.
	 * \param result The result after the extract.
	 * \return EXIT_FAILURE, EXIT_SUCCESS
	 */
	int ct_face_extract(std::string dirName, int min, int max, int x, int y, int z, int ax, int ay, int az, vtkPolyData* result);
	/**
	 * \brief Extract the surface point cloud from a poly data.
	 * \param poly_data The input poly data.
	 * \param output The output of the point cloud.
	 * \param resolution The resolution of the grid line.
	 * \return 0
	 */
	int surface_pointcloud_extract(vtkPolyData* poly_data, vtkPolyData* output, int resolution);
	/**
	 * \brief The surface reconstruction from the point cloud.
	 * \param input The input point cloud.
	 * \param neighborhood_size The neighborhood size of the surface reconstruction filter.
	 * \param sample_spacing The sample spacing of the surface reconstruction filter.
	 * \return 0
	 */
	int surface_reconstruction(vtkPolyData* input, int neighborhood_size, double sample_spacing);
	/**
	 * \brief Convert the obj mesh to a 2D picture.
	 * \param dir_name The directory of the obj file.
	 * \return 0
	 */
	int face_to_picture(std::string dir_name);
	/**
	 * \brief Extract the key point from the obj face file using the 2D picture.
	 * \param pic_dir The directory of the 2D face picture.
	 * \param obj_dir The directory of the obj face file.
	 * \return 0
	 */
	int face_keypoint_extract(std::string pic_dir, std::string obj_dir);
}; 