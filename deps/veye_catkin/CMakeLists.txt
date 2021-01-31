cmake_minimum_required(VERSION 3.5.0)
project(veye_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

FILE(GLOB SRC_LIST_C ${PROJECT_SOURCE_DIR}/src/*.c)
FILE(GLOB GL_LIST_C  ${PROJECT_SOURCE_DIR}/include/gl_scenes/*.c)

INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/gl_scenes
  ${PROJECT_SOURCE_DIR}/include/gl_scenes
  /opt/vc/include/
  /opt/vc/include/interface/vctypes/ 
  /opt/vc/include/interface/vcsm
)

cs_add_library(${PROJECT_NAME} SHARED ${SRC_LIST_C} ${GL_LIST_C})

target_link_libraries(${PROJECT_NAME} 
/opt/vc/lib/libmmal_core.so
/opt/vc/lib/libmmal_util.so
/opt/vc/lib/libmmal_vc_client.so
/opt/vc/lib/libvcos.so
/opt/vc/lib/libbcm_host.so
/opt/vc/lib/libbrcmEGL.so
/opt/vc/lib/libbrcmGLESv2.so
pthread
${PROJECT_SOURCE_DIR}/libveyecam.a
)

set(${PROJECT_NAME}_LOCAL_INCLUDE_DIR  ${PROJECT_SOURCE_DIR}/include)

message("333333" ${PROJECT_NAME})
message("4444444444" ${CMAKE_SOURCE_DIR})
message("4444444444" ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
message("555555555" ${CATKIN_DEVEL_PREFIX})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION ${CATKIN_DEVEL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS veye-extras.cmake)
