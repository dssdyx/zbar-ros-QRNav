cmake_minimum_required(VERSION 2.8.3)
project(zbar_opencv)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge nodelet std_msgs geometry_msgs sensor_msgs image_transport)
find_package(OpenCV)


find_package(PkgConfig)
pkg_check_modules(PC_ZBAR QUIET zbar)
set(ZBAR_DEFINITIONS ${PC_ZBAR_CFLAGS_OTHER})
find_library(ZBAR_LIBRARIES NAMES zbar
             HINTS ${PC_ZBAR_LIBDIR} ${PC_ZBAR_LIBRARY_DIRS} )
find_path(ZBAR_INCLUDE_DIR Decoder.h
          HINTS ${PC_ZBAR_INCLUDEDIR} ${PC_ZBAR_INCLUDE_DIRS}
          PATH_SUFFIXES zbar )
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZBAR  DEFAULT_MSG  ZBAR_LIBRARIES ZBAR_INCLUDE_DIR)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zbar_opencv_nodelet
  CATKIN_DEPENDS nodelet cv_bridge roscpp
#  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${opencv_INCLUDE_DIRS}
#        ${ZBAR_INCLUDE_DIR}
        )


 add_library(${PROJECT_NAME}_nodelet src/zbar_opencv_nodelet.cpp src/zbar_opencv.cpp)
 target_link_libraries(${PROJECT_NAME}_nodelet
     ${ZBAR_LIBRARIES}
     ${catkin_LIBRARIES}
     )


 install(TARGETS ${PROJECT_NAME}_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(FILES nodelet.xml
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


