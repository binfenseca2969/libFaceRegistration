#include "ct_face_extract.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);//Failed getting the TextRenderer instance!
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingContextOpenGL2);

CtFaceExtract::CtFaceExtract(){}

CtFaceExtract::~CtFaceExtract(){}

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
int CtFaceExtract::ct_face_extract(std::string dirName,int min,int max,int x,int y,int z,int ax,int ay,int az,vtkPolyData* result)
{
    //itk读取Dicom
    using PixelType = signed short;
    constexpr unsigned int Dimension = 3;
    using ImageType = itk::Image<PixelType, Dimension>;

    using ReaderType = itk::ImageSeriesReader<ImageType>;
    auto reader = ReaderType::New();

    using ImageIOType = itk::GDCMImageIO;
    auto dicomIO = ImageIOType::New();

    reader->SetImageIO(dicomIO);

    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();
    nameGenerator->SetInputDirectory(dirName);

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames = nameGenerator->GetInputFileNames();

    reader->SetFileNames(fileNames);

    try
    {
        reader->Update();
    }
    catch (itk::ExceptionObject& ex)
    {
        std::cout << ex << std::endl;
        return EXIT_FAILURE;
    }

    //itk转换为vtk
    using FilterType = itk::ImageToVTKImageFilter<ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(reader->GetOutput());

    try
    {
        filter->Update();
    }
    catch (itk::ExceptionObject& error)
    {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }
    vtkSmartPointer<vtkMarchingCubes> skinExtractor =
        vtkSmartPointer<vtkMarchingCubes>::New();
    skinExtractor->SetInputData(filter->GetOutput());
    skinExtractor->SetValue(min,max);//皮肤提取阈值
    skinExtractor->Update();

    vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
    plane->SetOrigin(x,y,z);
    plane->SetNormal(ax,ay,az);

    vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
    clipper->SetInputConnection(skinExtractor->GetOutputPort());
    clipper->SetClipFunction(plane);
    clipper->SetValue(0);
    clipper->Update();

    result->DeepCopy(clipper->GetOutput());

    std::string folderPath = "C:/temp/";
    if (_access(folderPath.c_str(), 0) == -1)
    {
        _mkdir(folderPath.c_str());
        // flag 为 true 说明创建成功
    }
    vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
    writer->SetFileName("C:/temp/ct_face.ply");
    writer->SetInputData(result);
    writer->Write();
    writer->Update();

    return EXIT_SUCCESS;
}

/**
 * \brief Extract the surface point cloud from a poly data.
 * \param poly_data The input poly data.
 * \param output The output of the point cloud.
 * \param resolution The resolution of the grid line.
 * \return 0
 */
int CtFaceExtract::surface_pointcloud_extract(vtkPolyData* poly_data,vtkPolyData* output,int resolution)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::vtkPolyDataToPointCloud(poly_data, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    //确定点云x轴z轴范围
    pcl::PointXYZ px_min = *std::min_element(cloud->begin(), cloud->end(),
        [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.x < pt2.x; });
    pcl::PointXYZ px_max = *std::max_element(cloud->begin(), cloud->end(),
        [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.x < pt2.x; });
    pcl::PointXYZ pz_min = *std::min_element(cloud->begin(), cloud->end(),
        [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.z < pt2.z; });
    pcl::PointXYZ pz_max = *std::max_element(cloud->begin(), cloud->end(),
        [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.z < pt2.z; });
    //确定网格细分大小
    float delta_x = (px_max.x - px_min.x) / resolution;
    float delta_z = (pz_max.z - pz_min.z) / resolution;
    //找到网格内y的最大值和最小值
    std::vector<int> indexs((resolution + 1) * (resolution + 1));
    std::vector<float> min((resolution + 1) * (resolution + 1), INT_MAX);
    for (size_t i = 0; i < cloud->size(); i++)
    {
        int id_x = (cloud->points[i].x - px_min.x) / delta_x;
        int id_z = (cloud->points[i].z - pz_min.z) / delta_z;
        int id = id_z * (resolution + 1) + id_x;//确定最大值的点的下标存储在indexs中的位置
        if (cloud->points[i].y < min[id])
        {
            min[id] = cloud->points[i].y;
            indexs[id] = i;
        }
    }
    pcl::copyPointCloud(*cloud, indexs, *cloud_boundary);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < cloud_boundary->size(); ++i)
    {
        vtkIdType pid[1];
        pid[0] = points->InsertNextPoint(cloud_boundary->at(i).x, cloud_boundary->at(i).y, cloud_boundary->at(i).z);
        vertices->InsertNextCell(1, pid);
    }

    output->SetPoints(points);
    output->SetVerts(vertices);
    return 0;
}

/**
 * \brief The surface reconstruction from the point cloud.
 * \param input The input point cloud.
 * \param neighborhood_size The neighborhood size of the surface reconstruction filter.
 * \param sample_spacing The sample spacing of the surface reconstruction filter.
 * \return 0
 */
int CtFaceExtract::surface_reconstruction(vtkPolyData* input,int neighborhood_size,double sample_spacing)
{
    vtkSmartPointer<vtkSurfaceReconstructionFilter> surface = vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();
    surface->SetInputData(input);
    surface->SetNeighborhoodSize(neighborhood_size);
    surface->SetSampleSpacing(sample_spacing);
    surface->Update();

    vtkSmartPointer<vtkContourFilter> contour = vtkSmartPointer<vtkContourFilter>::New();
    contour->SetInputConnection(surface->GetOutputPort());
    contour->SetValue(0, 0.0);
    contour->Update();

    std::string folderPath = "C:/temp/";
    if (_access(folderPath.c_str(), 0) == -1)
    {
        _mkdir(folderPath.c_str());
        // flag 为 true 说明创建成功
    }
    //保存
    vtkSmartPointer<vtkOBJWriter> writer = vtkSmartPointer<vtkOBJWriter>::New();
    writer->SetFileName("C:/temp/ct_face.obj");
    writer->SetInputConnection(contour->GetOutputPort());
    writer->Write();
    writer->Update();

    return 0;
}

/**
 * \brief Convert the obj mesh to a 2D picture.
 * \param dir_name The directory of the obj file.
 * \return 0
 */
int CtFaceExtract::face_to_picture(std::string dir_name)
{
    std::vector<float> x, y, z, alpha, beta, theta;
    obj_reader(dir_name, x, y, z, alpha, beta, theta);
    point2gray(x, y, z, alpha, beta, theta);
    return 0;
}

void CtFaceExtract::obj_reader(const std::string& obj_file, std::vector<float>& x, std::vector<float>& y, std::vector<float>& z,
    std::vector<float>& alpha, std::vector<float>& beta, std::vector<float>& theta)
{
    std::vector<float> u_list, v_list, z_list;
    std::ifstream file(obj_file.c_str());
    std::string line;
    while (std::getline(file, line)) {
        if (line.size() > 1) {
            if (line[0] == 'v' && line[1] != 'n') {
                std::istringstream iss(line);
                std::string vertex_type;
                float x_val, y_val, z_val;
                iss >> vertex_type >> x_val >> y_val >> z_val;
                x.push_back(x_val);
                y.push_back(y_val);
                z.push_back(z_val);
            }
            else if (line[0] == 'v' && line[1] == 'n') {
                std::istringstream iss(line);
                std::string vertex_type;
                float alpha_val, beta_val, theta_val;
                iss >> vertex_type >> alpha_val >> beta_val >> theta_val;
                alpha.push_back(alpha_val);
                beta.push_back(beta_val);
                theta.push_back(theta_val);
            }
        }
    }
}

void CtFaceExtract::point2gray(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z,
    const std::vector<float>& alpha, const std::vector<float>& beta, const std::vector<float>& theta)
{
    std::vector<float> u_list, v_list, z_list;
    for (size_t i = 0; i < x.size(); ++i) {
        u_list.push_back(x[i]);
        v_list.push_back(y[i]);
        z_list.push_back(z[i]);
    }
    int height = static_cast<int>(*std::max_element(u_list.begin(), u_list.end())) -
        static_cast<int>(*std::min_element(u_list.begin(), u_list.end()));
    int width = static_cast<int>(*std::max_element(z_list.begin(), z_list.end())) -
        static_cast<int>(*std::min_element(z_list.begin(), z_list.end()));
    cv::Mat gray_img = cv::Mat::zeros(width + 1, height + 1, CV_32F);
    cv::Mat alpha_img = cv::Mat::zeros(width + 1, height + 1, CV_32F);
    cv::Mat beta_img = cv::Mat::zeros(width + 1, height + 1, CV_32F);
    cv::Mat theta_img = cv::Mat::zeros(width + 1, height + 1, CV_32F);
    cv::Mat rst_img = cv::Mat::zeros(width + 1, height + 1, CV_32FC3);
    float u_min = *std::min_element(u_list.begin(), u_list.end());
    float z_max = *std::max_element(z_list.begin(), z_list.end());
    std::vector<int> u_list_int(u_list.size()), z_list_int(z_list.size());
    std::transform(u_list.begin(), u_list.end(), u_list_int.begin(), [&](float val) { return static_cast<int>(val - u_min); });
    std::transform(z_list.begin(), z_list.end(), z_list_int.begin(), [&](float val) { return static_cast<int>(z_max - val); });

    for (size_t i = 0; i < u_list_int.size(); ++i) {
        if (z_list_int[i] >= width + 1)
        {
            z_list_int[i] = width;
        }
        if (u_list_int[i] >= height + 1)
        {
            u_list_int[i] = height;
        }
        gray_img.at<float>(z_list_int[i], u_list_int[i]) = v_list[i];
        alpha_img.at<float>(z_list_int[i], u_list_int[i]) = std::abs(alpha[i]);
        beta_img.at<float>(z_list_int[i], u_list_int[i]) = std::abs(beta[i]);
        theta_img.at<float>(z_list_int[i], u_list_int[i]) = std::abs(theta[i]);
    }

    std::string folderPath = "C:/temp/";
    if (_access(folderPath.c_str(), 0) == -1)
    {
        _mkdir(folderPath.c_str());
        // flag 为 true 说明创建成功
    }

    cv::Mat img_gray = img_norm(gray_img);
    cv::imwrite("C:/temp/gray.jpg", img_gray);

    alpha_img = img_norm(alpha_img);
    cv::imwrite("C:/temp/alpha.jpg", alpha_img);

    beta_img = img_norm(beta_img);
    cv::imwrite("C:/temp/beta.jpg", beta_img);

    theta_img = img_norm(theta_img);
    cv::imwrite("C:/temp/theta.jpg", theta_img);

    cv::Mat channels[3];
    channels[0] = img_gray;
    channels[1] = alpha_img;
    channels[2] = theta_img;
    cv::merge(channels, 3, rst_img);
    cv::imwrite("C:/temp/rst.jpg", rst_img);
}

cv::Mat CtFaceExtract::img_norm(const cv::Mat& img)
{
    double img_max, img_min;
    cv::minMaxLoc(img, &img_min, &img_max);
    cv::Mat img_new = (img - img_min) * 255.0 / (img_max - img_min);
    double th = (0 - img_min) * 255.0 / (img_max - img_min);
    img_new.setTo(0, img_new == th);
    cv::medianBlur(img_new, img_new, 3);
    return img_new;
}

/**
 * \brief Extract the key point from the obj face file using the 2D picture.
 * \param pic_dir The directory of the 2D face picture.
 * \param obj_dir The directory of the obj face file.
 * \return 0
 */
int CtFaceExtract::face_keypoint_extract(std::string pic_dir,std::string obj_dir)
{
    cv::Mat image = cv::imread(pic_dir.c_str(), cv::IMREAD_GRAYSCALE);
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(obj_dir.c_str());
    reader->Update();

    //图片人脸关键点识别
    std::vector<dlib::rectangle> dets;
    std::vector<dlib::full_object_detection> faces;
    faces = GetShapes(image, dets);
    DrawPoint(faces, image);
    cv::imshow("Output", image);
    cv::waitKey(0);


    double bounds[6];
    reader->GetOutput()->GetBounds(bounds);//确定各个坐标轴方向的最大值最小值

    /*在关键点位置，做垂直于xoz平面的直线，直线与模型相交得到的点即为obj模型的人脸关键点*/
    vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
    tree->SetDataSet(reader->GetOutput());
    tree->BuildLocator();

    vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();//相交点
    vtkSmartPointer<vtkPoints> allPoints = vtkSmartPointer<vtkPoints>::New();//所有相交点的点集
    vtkIdType number;//直线与模型相交的点的数量，一般为1
    double lineP0[3], lineP1[3];//定义直线两端顶点

    for (auto const& face : faces)
    {
        for (int i = 17; i < 68; i++)
        {
            //图片关键点坐标和obj模型坐标的转换关系，确定obj模型画的直线的位置
            lineP0[0] = face.part(i).x() + bounds[0];
            lineP0[1] = 0.0;
            lineP0[2] = bounds[5] - face.part(i).y();
            lineP1[0] = face.part(i).x() + bounds[0];
            lineP1[1] = bounds[2] + 1;
            lineP1[2] = bounds[5] - face.part(i).y();

            tree->IntersectWithLine(lineP0, lineP1, intersectPoints, NULL);
            number = intersectPoints->GetNumberOfPoints();
            if (number == 1)
            {
                allPoints->InsertNextPoint(intersectPoints->GetPoint(0));
            }
        }
    }

    vtkSmartPointer<vtkPolyData> pointsPolyData = vtkSmartPointer<vtkPolyData>::New();
    pointsPolyData->SetPoints(allPoints);

    vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexGlyphFilter->AddInputData(pointsPolyData);
    vertexGlyphFilter->Update();

    std::string folderPath = "C:/temp/";
    if (_access(folderPath.c_str(), 0) == -1)
    {
        _mkdir(folderPath.c_str());
        // flag 为 true 说明创建成功
    }

    vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
    plyWriter->SetFileName("C:/temp/FaceKeypoint_CT.ply");
    plyWriter->SetInputConnection(vertexGlyphFilter->GetOutputPort());
    plyWriter->Write();

    return 0;
}

/**
 * \brief 识别2D人脸关键点
 * \param image 输入的2D图像
 * \param dets 识别到的人脸
 * \return 人脸序列
 */
std::vector<dlib::full_object_detection> CtFaceExtract::GetShapes(cv::Mat& image, std::vector<dlib::rectangle>& dets)
{
    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
    dlib::shape_predictor sp;
    dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> sp;
    dlib::array2d<dlib::bgr_pixel> dimg;
    dlib::assign_image(dimg, dlib::cv_image<uchar>(image));
    dets = detector(dimg);
    std::vector<dlib::full_object_detection> shapes;
    for (auto const& det : dets)
        shapes.push_back(sp(dimg, det));
    return shapes;
}

/**
 * \brief 画出人脸关键点
 * \param faces 输入的人脸序列
 * \param image 输入的2D图像
 */
void CtFaceExtract::DrawPoint(std::vector<dlib::full_object_detection>& faces, cv::Mat& image)
{
    cv::Point point;
    for (auto const& face : faces)
    {
        for (int i = 0; i < 68; i++)
        {
            point.x = face.part(i).x();
            point.y = face.part(i).y();
            cv::circle(image, point, 3, (0, 255, 0), -1);
        }
    }
}