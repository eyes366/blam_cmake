#ifndef BLAM_SLAM_H
#define BLAM_SLAM_H

//#include <measurement_synchronizer/MeasurementSynchronizer.h>
#include "PointCloudFilter.h"
#include "PointCloudOdometry.h"
#include "LaserLoopClosure.h"
#include "PointCloudLocalization.h"
#include "PointCloudMapper.h"
//#include <pcl_ros/point_cloud.h>
//// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include<string>
#include <fstream>
#include"common.h"


class BlamSlam {
 public:
	 typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

  BlamSlam();
  ~BlamSlam();

  bool Initialize();
  bool HandleLoopClosures(const PointCloud::ConstPtr& scan, bool* new_keyframe);
  void ProcessPointCloudMessage(const PointCloud::ConstPtr& msg);
  bool showPointCloud(int FrameCounter);
  std::string itos(int i);   // ½«int ×ª»»³Éstring 

private:

  // The node's name.
  std::string name_;
  // Update rates and callback timers.
  double estimate_update_rate_;
  double visualization_update_rate_;

  // Names of coordinate frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;
public:
  PointCloudFilter filter_;
  PointCloudOdometry odometry_;
  LaserLoopClosure loop_closure_;
  PointCloudLocalization localization_;
  PointCloudMapper mapper_;
  std::ofstream m_Log;
  std::string m_szLogPath;
  std::ofstream m_fInterMediaFilePath;
  void SaveTrackAndMap(const PointCloud::ConstPtr& msg);
};

#endif
