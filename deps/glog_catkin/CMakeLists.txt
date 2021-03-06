cmake_minimum_required(VERSION 3.5.0)
project(glog_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

set(VERSION 0.4.0)
ExternalProject_Add(glog_src
  URL https://gitee.com/mirrors/glog/repository/archive/v${VERSION}.zip
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cd ../glog_src/ && mkdir -p build && cd build && rm -rf * && cmake  
    -DWITH_GFLAGS=ON
    -DBUILD_SHARED_LIBS=ON 
    -DCMAKE_BUILD_TYPE=Release 
    -Dgflags_DIR=${gflags_catkin_PREFIX}  
    -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} ..
  BUILD_COMMAND cd ../glog_src/build && make -j1
  INSTALL_COMMAND cd ../glog_src/build && make install -j1
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} glog_src)
target_link_libraries(${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/lib/libglog${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS glog-extras.cmake)
