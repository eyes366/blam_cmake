#pragma once

#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include "LaserLoopClosure.h"
#include "ImuBlam.h"
#include "common.h"

class LoopClosureOpr
{
public:
	LoopClosureOpr();
	~LoopClosureOpr();

	int ReadData(std::string szDataPath);

	int StartLoopDetect();

public:
	void viewerPsycho(pcl::visualization::PCLVisualizer& viewer);
	bool HandleLoopClosures(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
		bool* new_keyframe);
	std::string m_szDataPath;
	CImuBlam m_PoseDataOpr;
	LaserLoopClosure m_LaserLoopClosureOpr;
	pcl::visualization::CloudViewer m_viewer;
	Eigen::Matrix4d m_PosePre;
	Eigen::Matrix4d m_PoseNow;
	unsigned int m_nInd;
    std::vector<std::vector<double> > m_GravityDatas;
    int m_nGravityInd;
};
