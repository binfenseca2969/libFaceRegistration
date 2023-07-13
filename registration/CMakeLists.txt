set(SUBSYS_NAME "registration")
set(REGISTRATION_SRC "./src/registration.cpp")
set(REGISTRATION_INCLUDE "./include/")

add_library(registration STATIC ${REGISTRATION_SRC})
target_link_libraries(registration PUBLIC ${PCL_LIBRARIES})
target_include_directories(registration PUBLIC ${REGISTRATION_INCLUDE} ${PCL_INCLUDE_DIRS})