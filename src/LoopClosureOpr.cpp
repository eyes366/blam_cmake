#include <fstream>
#include <ctime>
#include <boost/algorithm/string.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "LoopClosureOpr.h"
#include "geometry_utils/Transform3.h"
//#include "geometry_utils/MatrixNxNBase.h"

using namespace std;
using namespace pcl;

void static key_callback(const pcl::visualization::KeyboardEvent& key_event, void* puser)
{

}

LoopClosureOpr::LoopClosureOpr():
	m_viewer("LoopDetect")
{
	pcl::visualization::CloudViewer::VizCallable callback0 = 
		boost::bind(&LoopClosureOpr::viewerPsycho, this, _1);
    m_viewer.runOnVisualizationThread(callback0);

	m_viewer.registerKeyboardCallback(key_callback, this);

	m_LaserLoopClosureOpr.Initialize();

	m_nInd = 0;
    m_nGravityInd = 0;
}

LoopClosureOpr::~LoopClosureOpr()
{

}

void LoopClosureOpr::viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    if (!m_LaserLoopClosureOpr.m_bIsLoopInfoUpdate)
    {
        return;
    }
    m_LaserLoopClosureOpr.m_bIsLoopInfoUpdate = false;
    pcl::visualization::ShapeActorMapPtr ShapeMap = viewer.getShapeActorMap();
    pcl::visualization::CloudActorMapPtr CloudMap = viewer.getCloudActorMap();
 	std::vector<LaserLoopClosure::ConstraitLine>& LoopLines = m_LaserLoopClosureOpr.m_LoopLines;
 	for (unsigned int i = 0; i < LoopLines.size(); i++)
 	{
 		char szItem[1024] = { 0 };
 		sprintf(szItem, "Line%d", i);
//        if (viewer.contains(szItem))
        if (ShapeMap->find (szItem) != ShapeMap->end () ||
            CloudMap->find (szItem) != CloudMap->end ())
 		{
 			viewer.removeShape(szItem);
 		}
 		viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(LoopLines[i].first, LoopLines[i].second,
 			0.0, 255.0, 0.0, szItem);
//        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, szItem);
 	}

    std::vector<geometry_utils::Transform3> gravitys = m_LaserLoopClosureOpr.m_gravityLocate;
    for (unsigned int i = 0; i < gravitys.size(); i++)
    {
        Eigen::Matrix<double, 3, 3> R = gravitys[i].rotation.Eigen();
        Eigen::Matrix<double, 3, 1> T = gravitys[i].translation.Eigen();
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();;
        tf.block(0, 0, 3, 3) = R;
        tf.block(0, 3, 3, 1) = T;
        Eigen::Transform<double, 3, Eigen::Affine> ICP_Transform =
                Eigen::Transform<double, 3, Eigen::Affine>(tf);
        char szItem[1024] = { 0 };
        sprintf(szItem, "gravity%d", i);
        viewer.removeCoordinateSystem(szItem);
        viewer.addCoordinateSystem(5.0, (Eigen::Affine3f)ICP_Transform, szItem);
    }

// 	if (viewer.contains("m_closureCondidates"))
    if (ShapeMap->find ("m_closureCondidates") != ShapeMap->end () ||
        CloudMap->find ("m_closureCondidates") != CloudMap->end ())
 	{
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "m_closureCondidates");
 	}
}

int LoopClosureOpr::ReadData(std::string szDataPath)
{
	m_szDataPath = szDataPath;
	if (m_szDataPath.back() != '\\' && m_szDataPath.back() != '/')
	{
		m_szDataPath = m_szDataPath + string("/");
	}

	m_PoseDataOpr.ReadFile(m_szDataPath);

    ifstream fs;
    fs.open("/home/xinzi/opensource/blam_/build-trunk-Desktop-Release/GravityPose.txt");
    string szLine;
    while(getline(fs, szLine))
    {
        vector<double> dataT;
        vector<string> vecStr;
        boost::split(vecStr, szLine, boost::is_any_of(","));
        if (vecStr.size() < 4)
        {
            break;
        }
        double dd;
        dd = atof(vecStr[0].c_str());
        dataT.push_back(dd);
        dd = atof(vecStr[1].c_str());
        dataT.push_back(dd);
        dd = atof(vecStr[2].c_str());
        dataT.push_back(dd);
        dd = atof(vecStr[3].c_str());
        dataT.push_back(dd);
        m_GravityDatas.push_back(dataT);
        cout << dataT[0] << "," << dataT[1] << "," << dataT[2] << "," << dataT[3] << endl;
    }

    if (m_GravityDatas.size() <= 0)
    {
        cout << "No gravity data!" << endl;
    }

//    getchar();

	return 1;
}

int LoopClosureOpr::StartLoopDetect()
{
	if (m_PoseDataOpr.m_Data.size() <= 0)
	{
		return -1;
	}

	PointCloud<PointXYZI>::Ptr pc_all(new PointCloud<PointXYZI>);
	for (m_nInd = 0; m_nInd < m_PoseDataOpr.m_Data.size(); m_nInd++)
	{
//		cout << "Frame: " << m_nInd << endl;
		clock_t t0 = clock();

		GnssData PoseData = m_PoseDataOpr.m_Data[m_nInd];
		m_PoseNow = PoseData.toRT();
		PointCloud<PointXYZI>::Ptr pc(new PointCloud<PointXYZI>);
		char szPcdPath[1024] = { 0 };
		sprintf(szPcdPath, "%s%06d.pcd", m_szDataPath.c_str(), m_nInd);
		pcl::io::loadPCDFile(szPcdPath, *pc);
        pc->header.seq = m_nInd;
        pc->header.stamp = uint64_t(PoseData.dSecInWeek);

		clock_t t1 = clock();
//		cout << "Load pcd" << (double)(t1 - t0) / CLOCKS_PER_SEC << "s" << endl;

// 		PointCloud<PointXYZI>::Ptr pc_(new PointCloud<PointXYZI>);
// 		pcl::transformPointCloud(*pc, *pc_, pose.cast<float>().matrix());
// 		(*pc_all) += (*pc_);
// 		m_viewer.showCloud(pc_all, "pc_all");
		if (m_nInd == 0)
		{
			m_LaserLoopClosureOpr.AddKeyScanPair(m_nInd, pc);
		}
		else
		{
			bool bKeyFrame = false;
			HandleLoopClosures(pc, &bKeyFrame);
		}

		clock_t t2 = clock();
//		cout << "HandleLoopClosures" << (double)(t2 - t1) / CLOCKS_PER_SEC << "s" << endl;
		
		m_viewer.showCloud(m_LaserLoopClosureOpr.GetTrack()->makeShared(), "track");
		m_viewer.showCloud(m_LaserLoopClosureOpr.m_closureCondidates.makeShared(), "m_closureCondidates");

		clock_t t3 = clock();
//		cout << "showCloud" << (double)(t3 - t2) / CLOCKS_PER_SEC << "s" << endl;

		m_PosePre = m_PoseNow;//update pose
	}

	return 1;
}

bool LoopClosureOpr::HandleLoopClosures(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
	bool* new_keyframe)
{
	if (new_keyframe == NULL) {
		return false;
	}

	clock_t t0 = clock();

	// Add the new pose to the pose graph.
	unsigned int pose_key;
	geometry_utils::MatrixNxNBase<double, 6> covariance;
	covariance.Zeros();
	for (int i = 0; i < 3; ++i)
		covariance(i, i) = 0.01;
	for (int i = 3; i < 6; ++i)
		covariance(i, i) = 0.004;

	//const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
	Eigen::Matrix4d delta = m_PosePre.inverse()*m_PoseNow;
	geometry_utils::Vec3d T(delta(0, 3), delta(1, 3), delta(2, 3));
	geometry_utils::Rot3d R(delta(0, 0), delta(0, 1), delta(0, 2), 
							delta(1, 0), delta(1, 1), delta(1, 2), 
							delta(2, 0), delta(2, 1), delta(2, 2));
	geometry_utils::Transform3 deltaRT(T, R);

    if (m_nGravityInd >= 0 && m_nGravityInd < m_GravityDatas.size())
    {
        double dTime = m_GravityDatas[m_nGravityInd][0];
        if (scan->header.stamp >= dTime)
        {
            geometry_utils::Vec3d gravity(m_GravityDatas[m_nGravityInd][1],
                    m_GravityDatas[m_nGravityInd][2],m_GravityDatas[m_nGravityInd][3]);
//            geometry_utils::Vec3d gravity(0,0,-1);
            m_LaserLoopClosureOpr.AddGravityFactor(deltaRT,
                                                   covariance, &pose_key,gravity);
            printf("Time:%lld\n", scan->header.stamp);
            m_nGravityInd++;
            return false;
        }
    }

 	if (!m_LaserLoopClosureOpr.AddBetweenFactor(deltaRT,
 		covariance, &pose_key))
	{
		return false;				//δ�ﵽ�ؼ�֡��������
	}
	*new_keyframe = true;
//	cout << "Key frame detected!" << endl;

	clock_t t1 = clock();
//	cout << "AddBetweenFactor" << (double)(t1 - t0) / CLOCKS_PER_SEC << "s" << endl;

	if (!m_LaserLoopClosureOpr.AddKeyScanPair(pose_key, scan)) {
		return false;
	}
//	cout << "Key frame added!" << endl;

	clock_t t2 = clock();
//	cout << "AddKeyScanPair" << (double)(t2 - t1) / CLOCKS_PER_SEC << "s" << endl;

//    return false;

	std::vector<unsigned int> closure_keys;
	if (!m_LaserLoopClosureOpr.FindLoopClosures(pose_key, &closure_keys))
	{
		return false;
	}

	clock_t t3 = clock();
//	cout << "FindLoopClosures" << (double)(t3 - t2) / CLOCKS_PER_SEC << "s" << endl;

	for (const auto& closure_key : closure_keys)
	{
		char szLog[1024] = { 0 };
		sprintf(szLog, ": Closed loop between poses %u and %u.\n",
			pose_key, closure_key);
//		m_Log << szLog << endl;
//		m_Log.flush();
		printf(": Closed loop between poses %u and %u.\n",
			pose_key, closure_key);
	}
	return true;
}
