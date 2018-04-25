#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "ImuBlam.h"

using namespace std;

CImuBlam::CImuBlam()
{
	m_nSearchInd = 0;
}

CImuBlam::~CImuBlam()
{

}

int CImuBlam::ReadFile(std::string szFilePathName)
{
	cout << "Reading:" << szFilePathName << endl;
	ifstream fs((szFilePathName + "Pose.txt").c_str());
	if (!fs.is_open())
	{
		return -1;
	}
	string szLine;
	while (getline(fs, szLine))
	{
		GnssData dataT;
		vector<string> str_split;
		boost::split(str_split, szLine, boost::is_any_of(" "), boost::token_compress_on);
		if (str_split.size() != 18)
		{
			break;
		}

		dataT.dSecInWeek = atof(str_split[1].c_str());
//		dataT.dSecInWeek /= 1000000.0;
		dataT.nPositionType = 1;
		dataT.dX = atof(str_split[5].c_str());
		dataT.dY = atof(str_split[9].c_str());
		dataT.dZ = atof(str_split[13].c_str());

		dataT.nPoseType = 1;
		Eigen::Matrix3d rot;
		rot << atof(str_split[2].c_str()), atof(str_split[3].c_str()), atof(str_split[4].c_str()),
			atof(str_split[6].c_str()), atof(str_split[7].c_str()), atof(str_split[8].c_str()),
			atof(str_split[10].c_str()), atof(str_split[11].c_str()), atof(str_split[12].c_str());
		Eigen::Quaterniond q(rot);
		dataT.qx = q.x();
		dataT.qy = q.y();
		dataT.qz = q.z();
		dataT.qw = q.w();
		m_Data.push_back(dataT);
	}

	if (m_Data.size() <= 0)
	{
		cout << "m_PoseList.size() <= 0" << endl;
		getchar();
		return -1;
	}

	cout << "m_PoseList start:" << endl;
	printf("%.6f\n", m_Data.front().dSecInWeek);
	printf("%.6f\n", m_Data.back().dSecInWeek);

	return 1;
}

int CImuBlam::GetDataByTime(uint64_t nTime, GnssData& data)
{
	if (m_Data.size() <= 0)
	{
		return -1;
	}
	if (nTime > m_Data.back().dSecInWeek ||
		nTime < m_Data.front().dSecInWeek)
	{
		return -1;
	}

	for (unsigned int i = m_nSearchInd; i+1 < m_Data.size(); i++)
	{
		if (nTime >= m_Data[i].dSecInWeek &&
			nTime < m_Data[i+1].dSecInWeek)
		{
			m_nSearchInd = i;
			GnssData data0 = m_Data[i];
			GnssData data1 = m_Data[i+1];
			double dP1 = (double)(nTime-m_Data[i].dSecInWeek)/(double)(m_Data[i+1].dSecInWeek-m_Data[i].dSecInWeek);
			double dP0 = (double)(m_Data[i+1].dSecInWeek-nTime)/(double)(m_Data[i+1].dSecInWeek-m_Data[i].dSecInWeek);
			data = m_Data[i];
			data.dGpsWeek= dP0*data0.dGpsWeek + dP1*data1.dGpsWeek;
			data.dSecInWeek = dP0*data0.dSecInWeek + dP1*data1.dSecInWeek;
			data.dLongitude = dP0*data0.dLongitude + dP1*data1.dLongitude;
			data.dLatitude = dP0*data0.dLatitude + dP1*data1.dLatitude;
			data.dAltitude = dP0*data0.dAltitude + dP1*data1.dAltitude;
			data.dRoll = dP0*data0.dRoll + dP1*data1.dRoll;
			data.dPitch = dP0*data0.dPitch + dP1*data1.dPitch;
			data.dHeading = dP0*data0.dHeading + dP1*data1.dHeading;
			Eigen::Quaterniond q0(data0.qw, data0.qx, data0.qy, data0.qz);//��ֵ
			Eigen::Quaterniond q1(data1.qw, data1.qx, data1.qy, data1.qz);//
			Eigen::Quaterniond qres = q0.slerp(dP1, q1);
			data.qw = qres.w();
			data.qx = qres.x();
			data.qy = qres.y();
			data.qz = qres.z();

			return 1;
		}
	}

	return -1;
}

void CImuBlam::split(std::string& s, std::string& delim,std::vector< std::string >* ret)
{  
	size_t last = 0;  
	size_t index=s.find_first_of(delim,last);  
	while (index!=std::string::npos)  
	{  
		ret->push_back(s.substr(last,index-last));  
		last=index+1;  
		index=s.find_first_of(delim,last);  
	}  
	if (index-last>0)  
	{  
		ret->push_back(s.substr(last,index-last));  
	}  
} 