#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <vector>
#include <fstream>

#include "ManualLoopDetector.h"
#include "ImuBlam.h"
#include "IMUFile.h"
#include "EncoderFileParse.h"

using namespace std;
using gtsam::BetweenFactor;
using gtsam::ISAM2;
using gtsam::ISAM2Params;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Vector6;

ManualLoopDetector::ManualLoopDetector()
{

}

int ManualLoopDetector::ReadData(std::string szDataPath)
{
//    Eigen::Vector3d gravity_vector_(1.0, 1.0, 1.0);
//    Eigen::Quaterniond rotation_ = Eigen::Quaterniond::FromTwoVectors(
//                Eigen::Vector3d::UnitZ(), gravity_vector_);
//    Eigen::Matrix3d rotM = rotation_.toRotationMatrix();
//    Eigen::Vector3d rot_ = rotM.eulerAngles(2, 0, 1);//zxy
//        rot_ = rot_ / 3.141592654 * 180.0;
//        double dRoll_ = rot_(1);
//        double dPitch_ = rot_(2);
//        double dHeading_ = rot_(0);
//        if (dRoll_ >= 90.0)
//        {
//            dRoll_ = -1.0*dRoll_ + 180.0;
//            dHeading_ += 180.0;
//        }
//        if (dRoll_ <= -90.0)
//        {
//            dRoll_ = -1.0*dRoll_ - 180.0;
//            dHeading_ += 180.0;
//        }

//        if (dPitch_ >= 90.0)
//        {
//            dPitch_ -= 180.0;
//        }
//        if (dPitch_ <= -90.0)
//        {
//            dPitch_ += 180.0;
//        }

//    cout << dRoll_ << " " << dPitch_ << " " << dHeading_ << endl;


    CImuBlam blam_opr;
    blam_opr.ReadFile(szDataPath);

    IMUFileParse imu_opr;
    imu_opr.OpenFile("/media/xinzi/新加卷/MyData/20180201/0.imu");
    vector<IMUData> ImuDatas;
    for (unsigned int i = 0; i < 1000000; i++)
    {
        IMUData dataT;
        if (imu_opr.ReadData(dataT) < 0)
        {
            break;
        }
        dataT.sec = (double)dataT.week*7*24*3600 + dataT.sec;
 //       cout << i << ":" << dataT.week << ":" << dataT.sec << endl;
        dataT.sec *= 1000000.0;
        ImuDatas.push_back(dataT);
    }
    if (ImuDatas.size() <= 0)
    {
        cout << "ImuDatas.size() <= 0" << endl;
        return -1;
    }
    ImuDatas.pop_back();
    cout << "Imu data:" << endl;
    printf("%.6f\n", ImuDatas.front().sec);
    printf("%.6f\n", ImuDatas.back().sec);

    EncoderFileParse odo_opr;
    odo_opr.openFile("/media/xinzi/新加卷/MyData/20180201/0.odo");
    vector<EncoderData> OdoDatas;
    for (unsigned int i = 0; i < 1000000; i++)
    {
        EncoderData dataT;
        if (odo_opr.readData(dataT) < 0)
        {
            break;
        }
        dataT.sec = dataT.week*7*24*3600 + dataT.sec;
        dataT.sec *= 1000000.0;
        OdoDatas.push_back(dataT);
    }
    if (OdoDatas.size() <= 0)
    {
        cout << "OdoDatas.size() <= 0" << endl;
        return -1;
    }
    OdoDatas.pop_back();
    cout << "odo data:" << endl;
    printf("%.6f\n", OdoDatas.front().sec);
    printf("%.6f\n", OdoDatas.back().sec);


    /////////////////////////////////////////////////////////////////////////
    /// \brief Check stop event
    ///
    cout << "stop time: " << endl;
    double dStopTimeThreshold = 2.0;//if odometry stop time greater then 2.0, add a stop event
    double dStopImmuneTimeThreshold = 10.0;//if a stop event added, other stop event will be ignored in following 10.0 seconds
    double dStopIntegrateTime = 0.0;
    double dStopImmuneTime = 0.0;
    vector<double> StopList;
    for (unsigned int i = 1; i < OdoDatas.size(); i++)
    {
        double dDeltaTime = fabs(OdoDatas[i].sec - OdoDatas[i-1].sec)/1000000.0;
        if (OdoDatas[i].speed[0] == 0 && OdoDatas[i].speed[1] == 0)
        {
            dStopIntegrateTime += dDeltaTime;
        }
        else
        {
            dStopIntegrateTime = 0.0;
        }

        if (dStopImmuneTime >= 0.0)
        {
            dStopImmuneTime -= dDeltaTime;
        }
        if (dStopImmuneTime > 0.0)
        {
            continue;
        }

        if (dStopIntegrateTime >= dStopTimeThreshold)
        {
            StopList.push_back(OdoDatas[i].sec);
            dStopIntegrateTime = 0.0;
            dStopImmuneTime = dStopImmuneTimeThreshold;
            printf("Stop Time: %.6f\n", OdoDatas[i].sec);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    /// \brief Add Pose
    ///
    ofstream fs("GravityPose.txt");
    cout << "stop time and pose: " << endl;
    double x[3] = { -3.1415 *2, - 3.1415, 0.50 };
    Eigen::Matrix3d calib =
            (Eigen::AngleAxisd(x[0], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(x[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(x[2], Eigen::Vector3d::UnitZ())).matrix();
    vector<double> StopListImu;
    vector<Eigen::Vector3d> StopPoseList;
    for (unsigned int i = 1; i < ImuDatas.size(); i++)
    {
        for (unsigned int j = 0; j < StopList.size(); j++)
        {
            if (ImuDatas[i-1].sec <= StopList[j] &&
                ImuDatas[i].sec > StopList[j])
            {
                StopListImu.push_back(ImuDatas[i].sec);
                Eigen::Vector3d gravity_vector(
                        -1.0*ImuDatas[i].accelerometers[0],
                        -1.0*ImuDatas[i].accelerometers[1],
                        -1.0*ImuDatas[i].accelerometers[2]);
//                Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
//                            Eigen::Vector3d::UnitZ(), gravity_vector);
//                Eigen::Matrix3d rot_imu = rotation.toRotationMatrix();
//                Eigen::Matrix3d rot_lidar = rot_imu * calib;
//                Eigen::Vector3d RPY = Rot2RPY(rot_lidar);
//                StopPoseList.push_back(RPY);
                Eigen::Vector3d gravity_vector_imu = gravity_vector;
                Eigen::Vector3d gravity_vector_lidar = calib.inverse() * gravity_vector_imu;
                char szLog[1024] = {0};
//                sprintf(szLog, "%lld, %.3f, %.3f\n", uint64_t(StopList[j]), RPY(0), RPY(1));
                sprintf(szLog, "%lld, %.3f, %.3f, %.3f\n", uint64_t(StopList[j]),
                        gravity_vector_lidar(0), gravity_vector_lidar(1), gravity_vector_lidar(2));
                cout << szLog;
                fs << szLog;
                break;
            }
        }
    }


    return 1;
}

int ManualLoopDetector::Process()
{

    return 1;
}

Eigen::Vector3d ManualLoopDetector::Rot2RPY(Eigen::Matrix3d rot)
{
    Eigen::Matrix3d rotM = rot;
    Eigen::Vector3d rot_ = rotM.eulerAngles(2, 0, 1);//zxy
    rot_ = rot_ / M_PI * 180.0;
    double dRoll_ = rot_(1);
    double dPitch_ = rot_(2);
    double dHeading_ = rot_(0);
    if (dRoll_ >= 90.0)
    {
        dRoll_ = -1.0*dRoll_ + 180.0;
        dHeading_ += 180.0;
    }
    if (dRoll_ <= -90.0)
    {
        dRoll_ = -1.0*dRoll_ - 180.0;
        dHeading_ += 180.0;
    }

    if (dPitch_ >= 90.0)
    {
        dPitch_ -= 180.0;
    }
    if (dPitch_ <= -90.0)
    {
        dPitch_ += 180.0;
    }

    return Eigen::Vector3d(dRoll_, dPitch_, dHeading_);
}
