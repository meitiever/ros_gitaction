cmake_minimum_required(VERSION 3.5.0)
project(jsoncpp_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(
  jsoncpp_src
  URL https://github.com/open-source-parsers/jsoncpp/archive/1.9.4.zip
# URL_MD5 5cb0a1b38740ed596edb7f86cd5b3bd8
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cd ../jsoncpp_src &&
     cmake . 
       -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
       -DBUILD_SHARED_LIBS:BOOL=true
       -DCMAKE_BUILD_TYPE:STRING=Release
  BUILD_COMMAND cd ../jsoncpp_src && make -j 8
  INSTALL_COMMAND cd ../jsoncpp_src && make install -j 8
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} jsoncpp_src)
target_link_libraries(${PROJECT_NAME} 
  ${CATKIN_DEVEL_PREFIX}/lib/libjsoncpp${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS jsoncpp-extras.cmake)
