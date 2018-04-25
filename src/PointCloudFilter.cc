#include "PointCloudFilter.h"
#include "parameter_utils/slamBase.h"
//#include <parameter_utils/ParameterUtils.h>


#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "random_sample_.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

using namespace std;
//namespace pu = parameter_utils;

PointCloudFilter::PointCloudFilter() 
{
	m_Log.open("PointCloudFilter.log");
}
PointCloudFilter::~PointCloudFilter() {}

bool PointCloudFilter::Initialize() {
  if (!LoadParameters()) {
  
    return false;
  }
  return true;
}

bool PointCloudFilter::LoadParameters() {
  // Load filtering parameters.
  ParameterReader pd;
  params_.grid_filter = atoi(pd.getData("grid_filter").c_str());
  params_.grid_res = atof(pd.getData("grid_res").c_str());
  params_.random_filter = atoi(pd.getData("random_filter").c_str());
  params_.decimate_percentage = atof(pd.getData("decimate_percentage").c_str());
  params_.outlier_filter = atoi(pd.getData("outlier_filter").c_str());
  params_.outlier_std = atof(pd.getData("outlier_std").c_str());
  params_.outlier_knn = atoi(pd.getData("outlier_knn").c_str());

  params_.radius_filter = atoi(pd.getData("radius_filter").c_str());
  params_.radius = atof(pd.getData("radius").c_str());
  params_.radius_knn = atoi(pd.getData("radius_knn").c_str());

  // Cap to [0.0, 1.0].
  params_.decimate_percentage =
      std::min(1.0, std::max(0.0, params_.decimate_percentage));

  return true;
}


bool PointCloudFilter::Filter(const PointCloud::ConstPtr& points,
                              PointCloud::Ptr points_filtered) /*const*/ {
	char szLog[1024] = { 0 };
	m_Log << "Filter start" << endl;
	m_Log.flush();
  if (points_filtered == NULL) {
  //  ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Copy input points.
  *points_filtered = *points;

  // Apply a random downsampling filter to the incoming point cloud.
  if (params_.random_filter) {
    const int n_points = static_cast<int>((1.0 - params_.decimate_percentage) *
                                          points_filtered->size());
	pcl::RandomSample<pcl::PointXYZI> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(points_filtered);
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptOut(new pcl::PointCloud<pcl::PointXYZI>);
	m_Log << "save points_filtered" << endl;
	m_Log.flush();
//	pcl::io::savePCDFileBinary("points_filtered.pcd", *points_filtered);
	m_Log << "filter start" << endl;
	m_Log.flush();
    random_filter.filter(*ptOut);
	m_Log << "filter finish" << endl;
	m_Log.flush();
	(*points_filtered) = (*ptOut);
  }

  // Apply a voxel grid filter to the incoming point cloud.
  if (params_.grid_filter) {
	  pcl::VoxelGrid<pcl::PointXYZI> grid;
    grid.setLeafSize(params_.grid_res, params_.grid_res, params_.grid_res);
    grid.setInputCloud(points_filtered);
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptOut(new pcl::PointCloud<pcl::PointXYZI>);
    grid.filter(*ptOut);
	(*points_filtered) = (*ptOut);
  }

  // Remove statistical outliers in incoming the point cloud.
  if (params_.outlier_filter) {
	  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(points_filtered);
    sor.setMeanK(params_.outlier_knn);
    sor.setStddevMulThresh(params_.outlier_std);
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptOut(new pcl::PointCloud<pcl::PointXYZI>);
    sor.filter(*ptOut);
	(*points_filtered) = (*ptOut);
  }

  // Remove points without a threshold number of neighbors within a specified
  // radius.
  if (params_.radius_filter) {
	  pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad;
    rad.setInputCloud(points_filtered);
    rad.setRadiusSearch(params_.radius);
    rad.setMinNeighborsInRadius(params_.radius_knn);
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptOut(new pcl::PointCloud<pcl::PointXYZI>);
    rad.filter(*ptOut);
	(*points_filtered) = (*ptOut);
  }

  m_Log << "filter finish out" << endl;
  m_Log.flush();

  return true;
}
