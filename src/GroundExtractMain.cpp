#include <string>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>
#include "BlamSlam.h"
#include "VelodyneOpr.h"
#include "parameter_utils/slamBase.h"

#include "GroundExtract.h"

BlamSlam g_blam_slam_;
pcl::visualization::CloudViewer g_viewer("viewer");
CGroundExtractParam pp;
CGroundExtractor geopr;

void VelodyneDataCallBack(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& sweep)
{
//	g_blam_slam_.ProcessPointCloudMessage(sweep);
//	cout << sweep->header.seq << endl;

	
	
// 	pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
// 	std::ifstream aaa;
// 	aaa.open("C:\\Users\\Admin\\Desktop\\point.txt");
// 	while (!aaa.eof())
// 	{
// 		pcl::PointXYZI pt;
// 		aaa >> pt.x;
// 		aaa >> pt.y;
// 		aaa >> pt.z;
// 		temp->push_back(pt);
// 	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr pt_(new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<int> indecs;
	geopr.RemoveGround(sweep->makeShared(), pt_, indecs);
	cout << sweep->size() << ":" << pt_->size() << endl;
	pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
	pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZRGB>(*sweep, color_cloud);
	for (unsigned int i = 0; i < color_cloud.size(); i++)
	{
		color_cloud[i].r = 0.0;
		color_cloud[i].g = 0.0;
		color_cloud[i].b = 255.0;
	}
	for (unsigned int i = 0; i < indecs.size(); i++)
	{
		color_cloud[indecs[i]].r = 255.0;
		color_cloud[indecs[i]].g = 0;
		color_cloud[indecs[i]].b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> color_cloud_;
	std::vector<int> indecs_nan;
	removeNaNFromPointCloud(color_cloud, color_cloud_, indecs_nan);
	g_viewer.showCloud(color_cloud_.makeShared());

	boost::this_thread::sleep(boost::posix_time::milliseconds(10000000));
}

int main_(int argc, char ** argv)
{
	geopr.Init(pp);


	g_blam_slam_.Initialize();

	ParameterReader param_rd;
	std::string fileNamePcap = "D:\\MyData\\20180420143424\\0.pcap";// param_rd.getData("pcap_file_path");
	cout << "fileNamePcap:" << fileNamePcap << endl;

	VelodyneDataReadParam VelParam;
	VelParam.bUseExternalCallBack = false;
	VelParam.nDataFetchType = 1;
	VelParam.nDevType = 0;
	VelParam.nReadType = 1;
	VelParam.szPcapPath = fileNamePcap;
	VelParam.bIncludeNanPoint = true;
	CVelodyneOpr velOpr;
	velOpr.Init(VelParam);
	VelodyneCallBackSweep callback_sweep = boost::bind(&VelodyneDataCallBack, _1);
	velOpr.SetCallBackSweep(callback_sweep);
	velOpr.Start();

	boost::thread::sleep(boost::get_system_time() + boost::posix_time::hours(24));

	return 1;
}
