#include "camera_face_extract.h"

std::tuple<uint8_t, uint8_t, uint8_t> CameraFaceExtract::get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(
		texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CameraFaceExtract::points_to_pcl(const rs2::points & points, const rs2::video_frame & color)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto tex_coords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	for (int i = 0; i < points.size(); ++i)
	{
		cloud->points[i].x = vertices[i].x;
		cloud->points[i].y = vertices[i].y;
		cloud->points[i].z = vertices[i].z;

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);

		cloud->points[i].r = std::get<0>(current_color);
		cloud->points[i].g = std::get<1>(current_color);
		cloud->points[i].b = std::get<2>(current_color);
		if (i < 500)
			std::cout << int(cloud->points[i].r) << "\t" << int(cloud->points[i].g) << "\t" << int(cloud->points[i].b) << "\t" << std::endl;
	}

	return cloud;
}

/**
 * \brief 直通滤波器，得到面部指定区域的点云
 * \param cloud 输入的点云
 * \param up 最高点值（眉毛）
 * \param bottom 最低点值（鼻尖）
 * \param left 左端点值
 * \param right 右端点值
 * \return cloud_filtered 滤波后的点云
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CameraFaceExtract::PassthroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double up,
                                                         double bottom, double left, double right)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> passY;
	pcl::PassThrough<pcl::PointXYZRGB> passX;
	//y向滤波
	passY.setInputCloud(cloud);
	passY.setFilterFieldName("y");
	passY.setFilterLimits(up,bottom);
	passY.filter(*cloud_filtered);
	//x向滤波
	 passX.setInputCloud(cloud_filtered);
	 passX.setFilterFieldName("x");
	 passX.setFilterLimits(left, right);
	 passX.filter(*cloud_filtered);

	return cloud_filtered;
}

/**
 * \brief 对脸部关键点进行标识划线
 *		License: Apache 2.0. See LICENSE file in root directory.
 *		Copyright(c) 2019 Intel Corporation. All Rights Reserved.
 * \param face 检测到的脸部
 * \param color 上色的颜色
 * \return lines 画线vector
 */
std::vector<dlib::image_window::overlay_line> CameraFaceExtract::render_face(
	dlib::full_object_detection const& face,
	dlib::rgb_pixel const& color
)
{
	typedef dlib::image_window::overlay_line overlay_line;
	std::vector<overlay_line> lines;

	// Around Chin. Ear to Ear
    for( unsigned long i = markup_68::JAW_FROM; i < markup_68::JAW_TO; ++i )
        lines.push_back( overlay_line( face.part( i ), face.part( i + 1 ), color ) );

    // Nose
    for( unsigned long i = markup_68::NOSE_RIDGE_FROM; i < markup_68::NOSE_RIDGE_TO; ++i )
        lines.push_back( overlay_line( face.part( i ), face.part( i + 1 ), color ) );
    lines.push_back( overlay_line( face.part( markup_68::NOSE_TIP ), face.part( markup_68::NOSE_BOTTOM_FROM ), color ) );
    lines.push_back( overlay_line( face.part( markup_68::NOSE_TIP ), face.part( markup_68::NOSE_BOTTOM_TO ), color ) );

    // Left eyebrow
    for( unsigned long i = markup_68::RIGHT_EYEBROW_FROM; i < markup_68::RIGHT_EYEBROW_TO; ++i )
        lines.push_back( overlay_line( face.part( i ), face.part( i + 1 ), color ) );

    // Right eyebrow
    for( unsigned long i = markup_68::LEFT_EYEBROW_FROM; i < markup_68::LEFT_EYEBROW_TO; ++i )
        lines.push_back( overlay_line( face.part( i ), face.part( i + 1 ), color ) );

    // Right eye
    for( unsigned long i = markup_68::RIGHT_EYE_FROM; i < markup_68::RIGHT_EYE_TO; ++i )
        lines.push_back( overlay_line( face.part( i ), face.part( i + 1 ), color ) );
    lines.push_back( overlay_line( face.part( markup_68::RIGHT_EYE_FROM ), face.part( markup_68::RIGHT_EYE_TO ), color ) );

    // Left eye
    for( unsigned long i = markup_68::LEFT_EYE_FROM; i < markup_68::LEFT_EYE_TO; ++i )
        lines.push_back( overlay_line( face.part( i ), face.part( i + 1 ), color ) );
    lines.push_back( overlay_line( face.part( markup_68::LEFT_EYE_FROM ), face.part( markup_68::LEFT_EYE_TO ), color ) );

    // Lips inside part
    for( unsigned long i = markup_68::MOUTH_INNER_FROM; i < markup_68::MOUTH_INNER_TO; ++i )
        lines.push_back( overlay_line( face.part( i ), face.part( i + 1 ), color ) );
    lines.push_back( overlay_line( face.part( markup_68::MOUTH_INNER_FROM ), face.part( markup_68::MOUTH_INNER_TO ), color ) );

	lines.push_back(overlay_line(face.part(RIGHT_EAR), face.part(RIGHT_1), color));
	lines.push_back(overlay_line(face.part(RIGHT_1), face.part(RIGHT_2), color));
	lines.push_back(overlay_line(face.part(RIGHT_2), face.part(NOSE_BOTTOM_2), color));
	lines.push_back(overlay_line(face.part(NOSE_BOTTOM_2), face.part(LEFT_2), color));
	lines.push_back(overlay_line(face.part(LEFT_2), face.part(LEFT_1), color));
	lines.push_back(overlay_line(face.part(LEFT_1), face.part(LEFT_EAR), color));
	lines.push_back(overlay_line(face.part(LEFT_EAR), face.part(LEFT_EYEBROW_2), color));
	lines.push_back(overlay_line(face.part(LEFT_EYEBROW_2), face.part(RIGHT_EYEBROW_2), color));
	lines.push_back(overlay_line(face.part(RIGHT_EYEBROW_2), face.part(RIGHT_EAR), color));

	return lines;
}

/*
	Calculates the average depth for a range of two-dimentional points in face, such that:
		point(n) = face.part(n)
	and puts the result in *p_average_depth.
	Points for which no depth is available (is 0) are ignored and not factored into the average.
	Returns true if an average is available (at least one point has depth); false otherwise.
*/
bool CameraFaceExtract::find_depth_from(
	rs2::depth_frame const& frame,
	float const depth_scale,
	dlib::full_object_detection const& face,
	markup_68 markup_from, markup_68 markup_to,
	float* p_average_depth
)
{
	uint16_t const* data = reinterpret_cast<uint16_t const*>(frame.get_data());

	float average_depth = 0;
	size_t n_points = 0;
	for (int i = markup_from; i <= markup_to; ++i)
	{
		auto pt = face.part(i);
		auto depth_in_pixels = *(data + pt.y() * frame.get_width() + pt.x());
		if (!depth_in_pixels)
			continue;
		average_depth += depth_in_pixels * depth_scale;
		++n_points;
	}
	if (!n_points)
		return false;
	if (p_average_depth)
		*p_average_depth = average_depth / n_points;
	return true;
}

bool CameraFaceExtract::validate_face(
	rs2::depth_frame const& frame,
	float const depth_scale,
	dlib::full_object_detection const& face
)
{
	// Collect all the depth information for the different facial parts

	// For the ears, only one may be visible -- we take the closer one!
	float left_ear_depth = 100, right_ear_depth = 100;
	if (!find_depth_from(frame, depth_scale, face, markup_68::RIGHT_EAR, markup_68::RIGHT_1, &right_ear_depth)
		&& !find_depth_from(frame, depth_scale, face, markup_68::LEFT_1, markup_68::LEFT_EAR, &left_ear_depth))
		return false;
	float ear_depth = std::min(right_ear_depth, left_ear_depth);

	float chin_depth;
	if (!find_depth_from(frame, depth_scale, face, markup_68::CHIN_FROM, markup_68::CHIN_TO, &chin_depth))
		return false;

	float nose_depth;
	if (!find_depth_from(frame, depth_scale, face, markup_68::NOSE_RIDGE_2, markup_68::NOSE_TIP, &nose_depth))
		return false;

	float right_eye_depth;
	if (!find_depth_from(frame, depth_scale, face, markup_68::RIGHT_EYE_FROM, markup_68::RIGHT_EYE_TO, &right_eye_depth))
		return false;
	float left_eye_depth;
	if (!find_depth_from(frame, depth_scale, face, markup_68::LEFT_EYE_FROM, markup_68::LEFT_EYE_TO, &left_eye_depth))
		return false;
	float eye_depth = std::min(left_eye_depth, right_eye_depth);

	float mouth_depth;
	if (!find_depth_from(frame, depth_scale, face, markup_68::MOUTH_OUTER_FROM, markup_68::MOUTH_INNER_TO, &mouth_depth))
		return false;

	// We just use simple heuristics to determine whether the depth information agrees with
	// what's expected: that the nose tip, for example, should be closer to the camera than
	// the eyes.

	// These heuristics are fairly basic but nonetheless serve to illustrate the point that
	// depth data can effectively be used to distinguish between a person and a picture of a
	// person...

	if (nose_depth >= eye_depth)
		return false;
	if (eye_depth - nose_depth > 0.07f)
		return false;
	if (ear_depth <= eye_depth)
		return false;
	if (mouth_depth <= nose_depth)
		return false;
	if (mouth_depth > chin_depth)
		return false;

	// All the distances, collectively, should not span a range that makes no sense. I.e.,
	// if the face accounts for more than 20cm of depth, or less than 2cm, then something's
	// not kosher!
	float x = std::max({ nose_depth, eye_depth, ear_depth, mouth_depth, chin_depth });
	float n = std::min({ nose_depth, eye_depth, ear_depth, mouth_depth, chin_depth });
	if (x - n > 0.20f)
		return false;
	if (x - n < 0.02f)
		return false;

	return true;
}

/*
	The data pointed to by 'frame.get_data()' is a uint16_t 'depth unit' that needs to be scaled.
	We return the scaling factor to convert these pixels to meters. This can differ depending
	on different cameras.
*/
float CameraFaceExtract::get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

/**
 * \brief Extract the key area of the face point cloud by intel realsense camera. The point cloud will be saved at C:/temp.
 *		face.ply--the key area of the face point cloud.
 *		face_keypoints.ply--the key points of the face point cloud recognized by the dlib.
 *		depth_frame.ply--the depth frame point cloud from the intel realsense camera.
 *		face_picture.jpg--the RGB picture from the intel realsense camera.
 * \return 0
 *			EXIT_FAILURE
 */
int CameraFaceExtract::camera_face_extract() try
{
	rs2::pipeline pipe;
	rs2::config cfg;

	// cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
	// cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, 30);

	rs2::frameset frames;
	dlib::image_window win;
	std::vector<dlib::full_object_detection> faces;

	rs2::pipeline_profile profile = pipe.start();
	float const DEVICE_DEPTH_SCALE = get_depth_scale(profile.get_device());

	dlib::frontal_face_detector face_bbox_detector = dlib::get_frontal_face_detector();//创建dlib检测器用于检测人脸
	dlib::shape_predictor face_landmark_annotator;
	dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> face_landmark_annotator;
	/*
		 shape_predictor_68_face_landmarks.dat下载网址：
		 http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
	*/

	rs2::align align_to_color(RS2_STREAM_COLOR);

	dlib::rgb_pixel bad_color(255, 0, 0);
	dlib::rgb_pixel good_color(0, 255, 0);

	while (!win.is_closed())
	{
		frames = pipe.wait_for_frames();
		frames = align_to_color.process(frames);
		// auto depth = frames.get_depth_frame();
		auto color = frames.get_color_frame();
		auto depth = frames.get_depth_frame();

		rs_frame_image<dlib::rgb_pixel, RS2_FORMAT_RGB8>image(color);//Create a dlib image for face detection

		std::vector<dlib::rectangle> face_bboxex = face_bbox_detector(image);
		faces.resize(0);
		for (auto const& bbox : face_bboxex)
			faces.push_back(face_landmark_annotator(image, bbox));

		win.clear_overlay();
		win.set_image(image);
		// dlib::rgb_pixel lineColor = { 255,0,0 };
		for(auto const & face:faces)
		{
			dlib::rgb_pixel& color =
				validate_face(depth, DEVICE_DEPTH_SCALE, face)
				? good_color : bad_color;
			win.add_overlay(render_face(face, color));
		}
	}

	//创建点云
	auto depthFrame = frames.get_depth_frame();
	auto colorFrame = frames.get_color_frame();

	cv::Mat picture(cv::Size(640, 480), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
	cv::Mat picture_bgr;
	cv::cvtColor(picture, picture_bgr, cv::COLOR_RGB2BGR);
	
	rs2::pointcloud pc;
	rs2::points points;
	
	pc.map_to(colorFrame);
	points = pc.calculate(depthFrame);//这里顺序不能反，否则rgb信息不对
	
	auto pcl_points = points_to_pcl(points, colorFrame);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_points_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	for (auto const& face : faces)//找到裁剪的关键点
	{
		auto eyebrow = pcl_points->points[face.part(LEFT_EYEBROW_2).y() * depthFrame.get_width() + face.part(LEFT_EYEBROW_2).x()];
		auto noseBottom = pcl_points->points[face.part(NOSE_BOTTOM_2).y() * depthFrame.get_width() + face.part(NOSE_BOTTOM_2).x()];
		auto leftEye = pcl_points->points[face.part(LEFT_EYE_L).y() * depthFrame.get_width() + face.part(LEFT_EYE_L).x()];
		auto rightEye = pcl_points->points[face.part(RIGHT_EYE_R).y() * depthFrame.get_width() + face.part(RIGHT_EYE_R).x()];
	
		pcl_points_filtered = PassthroughFilter(pcl_points, eyebrow.y, noseBottom.y, rightEye.x - 0.02f, leftEye.x + 0.02f);
		for(int i=17;i<68;i++)
		{
			pcl_keypoints->push_back(pcl_points->points[face.part(i).y() * depthFrame.get_width() + face.part(i).x()]);
		}
	}
	
	//保存点云
	std::string folderPath = "C:/temp/";
	if (_access(folderPath.c_str(),0)==-1) 
	{
		_mkdir(folderPath.c_str());
		// flag 为 true 说明创建成功
	}
	pcl::PLYWriter writer;
	writer.write("C:/temp/face.ply", *pcl_points_filtered);
	writer.write("C:/temp/face_keypoints.ply", *pcl_keypoints);
	writer.write("C:/temp/depth_frame.ply", *pcl_points);
	cv::imwrite("C:/temp/face_picture.jpg",picture_bgr);
	
	//显示点云
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZRGB>(pcl_points_filtered);
	
	while (!viewer->wasStopped()) 
	{
		viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(1));
	}
	return 0;
}
catch (dlib::serialization_error const& e)
{
	std::cerr << "You need dlib's default face landmarking model file to run this example." << std::endl;
	std::cerr << "You can get it from the following URL: " << std::endl;
	std::cerr << "   http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << std::endl;
	std::cerr << std::endl << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const rs2::error& e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

CameraFaceExtract::CameraFaceExtract(){}

CameraFaceExtract::~CameraFaceExtract(){}
