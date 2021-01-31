cmake_minimum_required(VERSION 3.5.0)
project(opencv4_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

SET(CONTRIB_NAME "opencv4_contrib")
ExternalProject_Add(${CONTRIB_NAME}
  URL https://gitee.com/mirrors_opencv/opencv_contrib/repository/archive/4.2.0.zip
#  URL_MD5 43a77973751767a953385e5ff662ad946
  SOURCE_DIR ${CONTRIB_NAME}_src
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

SET(OPENCV_SRC_PATH "opencv4_src")
ExternalProject_Add(opencv4_src
  DEPENDS ${CONTRIB_NAME}
  URL https://gitee.com/mirrors_opencv/opencv/repository/archive/4.2.0.zip
# URL_MD5 0cc4be5d872818a245fce75751193cfc
  UPDATE_COMMAND ""
  SOURCE_DIR ${OPENCV_SRC_PATH}
  BINARY_DIR opencv4_build
  CONFIGURE_COMMAND cd ../${OPENCV_SRC_PATH} && mkdir -p build && cd build && rm -rf * && cmake
    -DOPENCV_EXTRA_MODULES_PATH=../../${CONTRIB_NAME}_src/modules .
    -DWITH_TBB=ON
    -DBUILD_opencv_python2=ON
    -DBUILD_opencv_python3=OFF
    -DWITH_GTK_2_X=ON  # Can't use GTK3 as it links against system protobuf.
    -DWITH_V4L=ON
    -DINSTALL_C_EXAMPLES=OFF
    -DINSTALL_PYTHON_EXAMPLES=OFF
    -DBUILD_EXAMPLES=OFF
    -DWITH_QT=OFF # Needed by clang under Ubuntu 14.04 and GTK_WIDGET(cvGetWindowHandle(...)) with gcc (image_view)
    -DWITH_OPENGL=ON
    -DWITH_VTK=ON
    -DENABLE_PRECOMPILED_HEADERS=OFF
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SHARED_LIBS=ON
    -DWITH_CUDA=OFF
    -DWITH_OPENCL=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_opencv_ts=OFF
    -DBUILD_TESTS=OFF
    # opencv_contrib packages
    -DBUILD_opencv_alphamat=OFF
    -DBUILD_opencv_apps=OFF
    -DBUILD_opencv_aruco=ON
    -DBUILD_opencv_bgsegm=OFF
    -DBUILD_opencv_bioinspired=OFF
    -DBUILD_opencv_calib3d=ON
    -DBUILD_opencv_ccalib=OFF
    -DBUILD_opencv_core=ON    
    -DBUILD_opencv_datasets=OFF    
    -DBUILD_opencv_dnn=OFF # Pulls in the system protobuf as a dependency!
    -DBUILD_opencv_dnn_objdetect=OFF
    -DBUILD_opencv_dnn_superres=OFF
    -DBUILD_opencv_dpm=OFF
    -DBUILD_opencv_face=OFF
    -DBUILD_opencv_features2d=ON
    -DBUILD_opencv_flann=ON    
    -DBUILD_opencv_freetype=OFF
    -DBUILD_opencv_fuzzy=OFF
    -DBUILD_opencv_gapi=OFF
    -DBUILD_opencv_hdf=OFF
    -DBUILD_opencv_hfs=OFF
    -DBUILD_opencv_highgui=ON
    -DBUILD_opencv_img_hash=OFF
    -DBUILD_opencv_imgcodecs=ON
    -DBUILD_opencv_imgproc=ON
    -DBUILD_opencv_intensity_transform=OFF
    -DBUILD_opencv_java_bindings_generator=OFF
    -DBUILD_opencv_js=OFF
    -DBUILD_opencv_line_descriptor=ON
    -DBUILD_opencv_ml=OFF        
    -DBUILD_opencv_objdetect=ON
    -DBUILD_opencv_optflow=ON
    -DBUILD_opencv_phase_unwrapping=OFF
    -DBUILD_opencv_photo=OFF
    -DBUILD_opencv_plot=OFF
    -DBUILD_opencv_python2=OFF
    -DBUILD_opencv_python_bindings_generator=OFF
    -DBUILD_opencv_python_tests=OFF        
    -DBUILD_opencv_quality=ON
    -DBUILD_opencv_rapid=ON
    -DBUILD_opencv_reg=ON
    -DBUILD_opencv_rgbd=ON
    -DBUILD_opencv_saliency=ON
    -DBUILD_opencv_sfm=OFF
    -DBUILD_opencv_shape=ON
    -DBUILD_opencv_stereo=ON
    -DBUILD_opencv_stitching=ON
    -DBUILD_opencv_structured_light=OFF
    -DBUILD_opencv_superres=OFF
    -DBUILD_opencv_surface_matching=OFF
    -DBUILD_opencv_text=OFF
    -DBUILD_opencv_tracking=OFF
    -DBUILD_opencv_ts=ON
    -DBUILD_opencv_video=ON
    -DBUILD_opencv_videoio=ON
    -DBUILD_opencv_videostab=ON
    -DBUILD_opencv_viz=ON
    -DBUILD_opencv_world=OFF
    -DBUILD_opencv_xfeatures2d=ON    
    -DBUILD_opencv_ximgproc=ON
    -DBUILD_opencv_xobjdetect=ON
    -DBUILD_opencv_xphoto=ON ..
  BUILD_COMMAND cd ../${OPENCV_SRC_PATH}/build && make -j8
  INSTALL_COMMAND cd ../${OPENCV_SRC_PATH}/build && make install -j8
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} opencv4_src)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libopencv_core${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
CFG_EXTRAS opencv4-extras.cmake)
