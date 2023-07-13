set(SUBSYS_NAME "ct_face_extract")
set(CT_FACE_EXTRACT_SRC "./src/ct_face_extract.cpp")
set(CT_FACE_EXTRACT_INCLUDE "./include/")

add_library(${SUBSYS_NAME} STATIC ${CT_FACE_EXTRACT_SRC})
target_link_libraries(${SUBSYS_NAME} PUBLIC ${VTK_LIBRARIES} ${ITK_LIBRARIES} ${PCL_LIBRARIES})
target_include_directories(${SUBSYS_NAME} PUBLIC ${CT_FACE_EXTRACT_INCLUDE} ${VTK_INCLUDE_DIRS} ${ITK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})