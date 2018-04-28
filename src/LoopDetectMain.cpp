#include <pcl/io/pcd_io.h>
#include <fstream>
#include "LoopClosureOpr.h"

int main(void)
{
	LoopClosureOpr LCOpr;
//	LCOpr.ReadData("D:\\MyWork\\CloudPointMappingLocation\\trunk\\BlamSlam\\build\\InterMedia20180423113047_");
LCOpr.ReadData("/media/xinzi/新加卷/MyWork/CloudPointMappingLocation/trunk/BlamSlam/build/InterMedia20180423113047_");
	LCOpr.StartLoopDetect();

//    pcl::io::savePCDFileBinary("m_track.pcd", LCOpr.m_LaserLoopClosureOpr.m_track);
    std::ofstream fs("m_track.txt");
    for (unsigned int i = 0; i < LCOpr.m_LaserLoopClosureOpr.m_track.size(); i++)
    {
        char szLog[1024] = {0};
        pcl::PointXYZRGB& pt = LCOpr.m_LaserLoopClosureOpr.m_track[i];
        sprintf(szLog, "%.3f %.3f %.3f\n", pt.x, pt.y, pt.z);
        fs << szLog;
        fs.flush();
    }

	getchar();

	return 0;
}
