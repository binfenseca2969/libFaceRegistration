set(SUBSYS_NAME "camera_face_extract")
set(CAMERA_FACE_EXTRACT_SRC "./src/camera_face_extract.cpp")
set(CAMERA_FACE_EXTRACT_INCLUDE "./include/")

add_library(camera_face_extract STATIC ${CAMERA_FACE_EXTRACT_SRC})
target_link_libraries(camera_face_extract PUBLIC ${PCL_LIBRARIES} ${realsense2_LIBRARIES} dlib::dlib)
target_include_directories(camera_face_extract PUBLIC ${CAMERA_FACE_EXTRACT_INCLUDE} ${PCL_INCLUDE_DIRS} ${realsense2_INCLUDE_DIR} ${dlib_INCLUDE_DIRS})