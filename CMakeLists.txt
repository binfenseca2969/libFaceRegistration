# CMakeList.txt: libFaceRegistration 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

project ("libFaceRegistration")
#PCL
set(PCL_DIR "C:/PCL 1.8.1/cmake")
find_package(PCL REQUIRED)
#libRealsense2
set(realsense2_DIR "C:/librealsense-2.53.1/lib/cmake/realsense2")
find_package(realsense2 REQUIRED)
#dlib
find_package(dlib REQUIRED)
#VTK
set(VTK_DIR "C:/VTK-7.1.1/lib/cmake/vtk-7.1")
find_package(VTK REQUIRED)
#ITK
set(ITK_DIR "C:/ITK-4.12/lib/cmake/ITK-4.12")
find_package(ITK REQUIRED)

add_subdirectory(camera_face_extract lib/camera_face_extract)
add_subdirectory(ct_face_extract lib/ct_face_extract)
add_subdirectory(registration lib/registration)

# TODO: 如有需要，请添加测试并安装目标。
