project(BlamSlam C CXX)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#pcl
add_definitions(-DNOMINMAX)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

LINK_DIRECTORIES(${Boost_LIBRARY_DIR_RELEASE})

#pcap
set(PCAP_DEFINES -DHAVE_PCAP -DWIN32)
set(PCAP_INCLUDE_DIRS ${BlamSlam_SOURCE_DIR}/WpdPack/include)
set(PCAP_LIBRARIES ${BlamSlam_SOURCE_DIR}/WpdPack/Lib/x64/wpcap.lib ${BlamSlam_SOURCE_DIR}/WpdPack/Lib/x64/Packet.lib)
include_directories(${PCAP_INCLUDE_DIRS})
add_definitions(${PCAP_DEFINES})

#opencv
#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#target_link_libraries( ${OpenCV_LIBS} )\

#gtsam
find_package( Gtsam REQUIRED)
include_directories( ${GTSAM_INCLUDE_DIR} )

include_directories(../src)
include_directories(./include)

#add_definitions(-DPCL_NO_VISUALIZATION)
add_executable(BlamSlamMapping 		./src/BlamSlamMain.cpp
									./src/hdl_grabber_.cpp
									./src/hdl_grabber_.h
									./src/vlp_grabber_.cpp
									./src/vlp_grabber_.h
									./src/VelodyneOpr.h
									./src/VelodyneOpr.cpp
									./src/BlamSlam.h
									./src/BlamSlam.cc
									./src/LaserLoopClosure.h
									./src/LaserLoopClosure.cc
									./src/PointCloudFilter.h
									./src/PointCloudFilter.cc
									./src/PointCloudLocalization.h
									./src/PointCloudLocalization.cc
									./src/PointCloudMapper.h
									./src/PointCloudMapper.cc
									./src/PointCloudOdometry.h
									./src/PointCloudOdometry.cc
									./include/parameter_utils/slamBase.h
									./src/slamBase.cc
									)
target_link_libraries (BlamSlamMapping ${PCL_LIBRARIES} ${PCAP_LIBRARIES} gtsam)

add_executable(BlamLoopDetect 		./src/LoopDetectMain.cpp
									./src/hdl_grabber_.cpp
									./src/hdl_grabber_.h
									./src/vlp_grabber_.cpp
									./src/vlp_grabber_.h
									./src/VelodyneOpr.h
									./src/VelodyneOpr.cpp
									./src/BlamSlam.h
									./src/BlamSlam.cc
									./src/LaserLoopClosure.h
									./src/LaserLoopClosure.cc
									./src/PointCloudFilter.h
									./src/PointCloudFilter.cc
									./src/PointCloudLocalization.h
									./src/PointCloudLocalization.cc
									./src/PointCloudMapper.h
									./src/PointCloudMapper.cc
									./src/PointCloudOdometry.h
									./src/PointCloudOdometry.cc
									./src/LoopClosureOpr.h
									./src/LoopClosureOpr.cpp
									./src/ImuBlam.cpp
									./src/ImuBlam.h
									./include/parameter_utils/slamBase.h
									./src/slamBase.cc
									)
target_link_libraries (BlamLoopDetect ${PCL_LIBRARIES} ${PCAP_LIBRARIES} gtsam)