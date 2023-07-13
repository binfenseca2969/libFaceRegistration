# libFaceRegistration

## Introduction
The libFaceRegistration is the library to registration with the face depth images from the structure light camera and the CT images. The library is still under development. The source articles of this library is A NEW METHOD IN FACIAL REGISTRATION IN CLINICS BASED ON STRUCTURE LIGHT IMAGES. 

The following is an abstract of the article.

Background and Objective: In neurosurgery, fusing clinical images and depth images that can improve the information and details is beneficial to surgery. We found that the registration of face depth images was invalid frequently using existing methods. To abundant traditional image methods with depth information, a method in registering with depth images and traditional clinical images was investigated. Methods: We used the dlib library, a C++ library that could be used in face recognition, and recognized the key points on faces from the structure light camera and CT image. The two key point clouds were registered for coarse registration by the ICP method. Fine registration was finished after coarse registration by the ICP method. Results: The root means squared error (RMSE) after coarse and fine registration is as low as 0.995913 mm. Compared with traditional methods, it also takes less time. Conclusions: The new method successfully registered the facial depth image from structure light images and CT with a low error, and that would be promising and efficient in clinical application of neurosurgery.

## The library API

### CameraFaceExtract

```cpp
int camera_face_extract();
```

Extract the key area of the face point cloud by intel realsense camera. The point cloud will be saved at C:/temp.
*	face.ply--the key area of the face point cloud.
*	face_keypoints.ply--the key points of the face point cloud recognized by the dlib.
*	depth_frame.ply--the depth frame point cloud from the intel realsense camera.
*	face_picture.jpg--the RGB picture from the intel realsense camera.

### CtFaceExtract

```cpp
int ct_face_extract(std::string dirName, int min, int max, int x, int y, int z, int ax, int ay, int az, vtkPolyData* result);
```

Extract the face mesh from the DICOM file.
* dirName--The directory of the DICOM file.
* min--The minimum threshold of MarchingCube.
* max--The maximum threshold of MarchingCube.
* x--X coordinates of the split plane.
* y--Y coordinates of the split plane.
* z--Z coordinates of the split plane.
* ax--X coordinates of the normal vector of the split plane.
* ay--Y coordinates of the normal vector of the split plane.
* az--Z coordinates of the normal vector of the split plane.
* result--The result after the extract.

```cpp
int surface_pointcloud_extract(vtkPolyData* poly_data, vtkPolyData* output, int resolution);
```

Extract the surface point cloud from a poly data.
* poly_data--The input poly data.
* output--The output of the point cloud.
* resolution--The resolution of the grid line.

```cpp
int surface_reconstruction(vtkPolyData* input, int neighborhood_size, double sample_spacing);
```

The surface reconstruction from the point cloud.
* input--The input point cloud.
* neighborhood_size--The neighborhood size of the surface reconstruction filter.
* sample_spacing--The sample spacing of the surface reconstruction filter.

```cpp
int face_to_picture(std::string dir_name);
```

Convert the obj mesh to a 2D picture.
* dir_name--The directory of the obj file.

```cpp
int face_to_picture(std::string dir_name);
```

Extract the key point from the obj face file using the 2D picture.
* pic_dir--The directory of the 2D face picture.
* obj_dir--The directory of the obj face file.

### Registration

```cpp
int registration(std::string source_keypoint, std::string target_keypoint, std::string source_cloud, std::string target_cloud, int icp1_maximum_iterations, int icp2_maximum_iterations);
```

Registration the two point cloud with the key points.
* source_keypoint--The source key points.
* target_keypoint--The target key points.
* source_cloud--The source point cloud.
* target_cloud--The target point cloud.
* icp1_maximum_iterations--The maximum iterations of the first ICP.
* icp2_maximum_iterations--The maximum iterations of the second ICP.
