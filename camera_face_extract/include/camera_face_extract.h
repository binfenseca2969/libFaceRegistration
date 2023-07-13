#pragma once

#include <iostream>
#include <io.h>
#include <direct.h>
#include <string.h>
#include <Windows.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <vector>
#include "rs_frame_image.h"
#include "markup_68.h"
#include <opencv2/opencv.hpp>

class CameraFaceExtract
{
private:
	std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PassthroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double up, double bottom, double left, double right);
	std::vector<dlib::image_window::overlay_line > render_face(dlib::full_object_detection const& face, dlib::rgb_pixel const& color);
	bool find_depth_from(rs2::depth_frame const& frame, float const depth_scale, dlib::full_object_detection const& face, markup_68 markup_from, markup_68 markup_to, float* p_average_depth);
	bool validate_face(rs2::depth_frame const& frame,float const depth_scale,dlib::full_object_detection const& face);
	float get_depth_scale(rs2::device dev);
public:
	CameraFaceExtract();
	~CameraFaceExtract();
	/**
	 * \brief Extract the key area of the face point cloud by intel realsense camera. The point cloud will be saved at C:/temp.
	 *		face.ply--the key area of the face point cloud.
	 *		face_keypoints.ply--the key points of the face point cloud recognized by the dlib.
	 *		depth_frame.ply--the depth frame point cloud from the intel realsense camera.
	 *		face_picture.jpg--the RGB picture from the intel realsense camera.
	 * \return 0
	 *			EXIT_FAILURE
	 */
	int camera_face_extract();
};
