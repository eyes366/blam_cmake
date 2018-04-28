#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>

class ManualLoopDetector
{
public:
    ManualLoopDetector();

    int ReadData(std::string szDataPath);

    int Process();

private:
    Eigen::Vector3d Rot2RPY(Eigen::Matrix3d rot);
};
