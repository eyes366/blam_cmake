#include "LaserLoopClosure.h"

//#include <parameter_utils/ParameterUtils.h>
#include <parameter_utils/slamBase.h>

#include <pcl/registration/gicp.h>

namespace gu = geometry_utils;
//namespace pu = parameter_utils;

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

gtsam::Point3 g_ptEarthCore(0.0, 0.0, -6400000.0);

LaserLoopClosure::LaserLoopClosure()
    : key_(0), last_closure_key_(std::numeric_limits<int>::min()) 
{
	m_Log.open("LaserLoopClosure.log");
    m_bIsLoopInfoUpdate = false;
}

LaserLoopClosure::~LaserLoopClosure() {}

bool LaserLoopClosure::Initialize() {

  if (!filter_.Initialize()) {
    return false;
  }

  if (!LoadParameters()) {
    return false;
  }

  return true;
}
bool LaserLoopClosure::LoadParameters() {
	ParameterReader pd;
	fixed_frame_id_ = pd.getData("fixed");
	base_frame_id_ = pd.getData("base");

	check_for_loop_closures_ = atoi(pd.getData("check_for_loop_closures").c_str());
	cout << "check_for_loop_closures_:" << check_for_loop_closures_ << endl;
	// Load ISAM2 parameters.
	unsigned int relinearize_skip = 1;
	double relinearize_threshold = 0.01;
	relinearize_skip = atoi(pd.getData("relinearize_skip").c_str());
	relinearize_threshold = atof(pd.getData("relinearize_threshold").c_str());

	//// Load loop closing parameters.
	translation_threshold_ = atof(pd.getData("translation_threshold").c_str());
	proximity_threshold_ = atof(pd.getData("proximity_threshold").c_str());
	max_tolerable_fitness_ = atof(pd.getData("max_tolerable_fitness").c_str());
	skip_recent_poses_ = atoi(pd.getData("skip_recent_poses").c_str());
	poses_before_reclosing_ = atoi(pd.getData("poses_before_reclosing").c_str());

	//// Load ICP parameters.
	icp_tf_epsilon_ = atof(pd.getData("tf_epsilon").c_str());
	icp_corr_dist_ = atof(pd.getData("corr_dist").c_str());
	icp_iterations_ = atoi(pd.getData("iterations").c_str());
	// Load initial position.
	double init_x = 0.0, init_y = 0.0, init_z = 0.0;
	double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
	init_x = atof(pd.getData("position_x").c_str());
	init_y = atof(pd.getData("position_y").c_str());
	init_z = atof(pd.getData("position_z").c_str());

	init_roll = atof(pd.getData("orientation_roll").c_str());
	init_pitch = atof(pd.getData("orientation_pitch").c_str());
	init_yaw = atof(pd.getData("orientation_yaw").c_str());




	// Load initial position and orientation noise.
	double sigma_x = 0.0, sigma_y = 0.0, sigma_z = 0.0;
	double sigma_roll = 0.0, sigma_pitch = 0.0, sigma_yaw = 0.0;
	sigma_x = atof(pd.getData("position_sigma_x").c_str());
	sigma_y = atof(pd.getData("position_sigma_y").c_str());
	sigma_z = atof(pd.getData("position_sigma_z").c_str());

	sigma_roll = atof(pd.getData("orientation_sigma_roll").c_str());
	sigma_pitch = atof(pd.getData("orientation_sigma_pitch").c_str());
	sigma_yaw = atof(pd.getData("orientation_sigma_yaw").c_str());

	// Create the ISAM2 solver.
	ISAM2Params parameters;
//	parameters.enableRelinearization = false;
	parameters.relinearizeSkip = relinearize_skip;
	parameters.relinearizeThreshold = relinearize_threshold;
	isam_.reset(new ISAM2(parameters));
	//Vector3 translation(init_x, init_y, init_z);

    //add earth center pose
    Pose3 pose_earth_center(Rot3(), g_ptEarthCore);
    Vector6 noise_earth;
    noise_earth << 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001;
    LaserLoopClosure::Diagonal::shared_ptr covariance_earth(
        LaserLoopClosure::Diagonal::Sigmas(noise_earth));
    NonlinearFactorGraph new_factor_;
    Values new_value_;
    new_factor_.add(MakePriorFactor(pose_earth_center, covariance_earth));
    new_value_.insert(key_, pose_earth_center);

    isam_->update(new_factor_, new_value_);
    values_ = isam_->calculateEstimate();
    values_.print();
    key_++;

	// Set the initial position.
	gtsam::Point3 translation(init_x, init_y, init_z);
	Rot3 rotation(Rot3::RzRyRx(init_roll, init_pitch, init_yaw));
	Pose3 pose(rotation, translation);

	//// Set the covariance on initial position.

	Vector6 noise;
	noise << sigma_x, sigma_y, sigma_z, sigma_roll, sigma_pitch, sigma_yaw;
	LaserLoopClosure::Diagonal::shared_ptr covariance(
		LaserLoopClosure::Diagonal::Sigmas(noise));

	
	//Rot3 rotation(Rot3::RzRyRx(init_roll, init_pitch, init_yaw));
	// Initialize ISAM2.
	NonlinearFactorGraph new_factor;
	Values new_value;
	/*initial.insert(X(1),
		Pose3(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(0, 0, 2)));*/
	new_factor.add(MakePriorFactor(pose, covariance));
	new_value.insert(key_, pose);
	
	isam_->update(new_factor, new_value);
	values_ = isam_->calculateEstimate();
	
	key_++;
	// Set the initial odometry.
	odometry_ = Pose3::identity();

	return true;
}

int LaserLoopClosure::UpdateDispInfo()
{
    m_track.clear();
    m_poses.clear();
    for (const auto& keyed_pose : values_)
    {
        const unsigned int key = keyed_pose.key;

        if (!keyed_scans_.count(key))
            continue;
        if (key == 0)
            continue;

        Pose3 pose_now = values_.at<Pose3>(key);
        pcl::PointXYZRGB pt;
        pt.x = float(pose_now.translation().x());
        pt.y = float(pose_now.translation().y());
        pt.z = float(pose_now.translation().z());
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        m_track.push_back(pt);
        m_poses.push_back(pose_now);
    }
    m_LoopLines.clear();
    for (unsigned int i = 0; i < loop_edges_.size(); i++)
    {
        const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(loop_edges_[i].first));
        const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(loop_edges_[i].second));
        pcl::PointXYZ pt0, pt1;
        pt0.x = pose1.translation(0);
        pt0.y = pose1.translation(1);
        pt0.z = pose1.translation(2);
        pt1.x = pose2.translation(0);
        pt1.y = pose2.translation(1);
        pt1.z = pose2.translation(2);
        m_LoopLines.push_back(std::make_pair(pt0, pt1));
    }
    m_gravityLocate.clear();
    for (unsigned int i = 0; i < m_gravityInds.size(); i++)
    {
        const gu::Transform3 pose = ToGu(values_.at<Pose3>(m_gravityInds[i]));
        m_gravityLocate.push_back(pose);
        geometry_utils::Vec3d YPR = pose.rotation.GetEulerZYX();
        cout << "m_gravityInds:" << i << ":" << YPR/M_PI*180.0 << endl;

//        Pose3 pose_now = values_.at<Pose3>(m_gravityInds[i]);
//        pcl::PointXYZRGB pt;
//        pt.x = float(pose_now.translation().x());
//        pt.y = float(pose_now.translation().y());
//        pt.z = float(pose_now.translation().z());
//        pt.r = 0;
//        pt.g = 255;
//        pt.b = 255;
//        m_track.push_back(pt);
    }
    m_bIsLoopInfoUpdate = true;
}

#include <gtsam/nonlinear/Marginals.h>

bool LaserLoopClosure::AddGravityFactor(const gu::Transform3& delta,
        const LaserLoopClosure::Mat66& covariance, unsigned int* key,
        geometry_utils::Vec3d gravity)
{
  if (key == NULL) {
    return false;
  }
  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactor(new_odometry, ToGtsam(covariance)));

  Pose3 last_pose = values_.at<Pose3>(key_ - 1);
  new_value.insert(key_, last_pose.compose(new_odometry));

  //Gravity factor
//  Eigen::Matrix3d rot =
//  Eigen::AngleAxisd(0.0 / 180.0 * M_PI, Eigen::Vector3d::UnitZ()) *//yaw
//  Eigen::AngleAxisd(dRoll / 180.0 * M_PI, Eigen::Vector3d::UnitX()) *//roll
//  Eigen::AngleAxisd(dPitch / 180.0 * M_PI, Eigen::Vector3d::UnitY()).matrix();//pitch
//  Rot3 rot_(rot(0,0), rot(0,1), rot(0,2),
//            rot(1,0), rot(1,1), rot(1,2),
//            rot(2,0), rot(2,1), rot(2,2));
//  double dRoll_ = rot_.roll();
//  double dPitch_ = rot_.pitch();
//  double dYaw_ = rot_.yaw();

//  Vector6 noise;
//  noise << 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.01;
//  LaserLoopClosure::Diagonal::shared_ptr covariance_(
//      LaserLoopClosure::Diagonal::Sigmas(noise));
//  Pose3 pose_now_ = last_pose.compose(new_odometry);
//  dYaw_ = pose_now_.rotation().yaw();
//  cout << "yaw: " << dYaw_ << endl;
//  Rot3 rot_1 = Rot3::Ypr(dYaw_, dPitch_, dRoll_);
//  cout << "rot_1: " << rot_1 << endl;
//  printf("%.3f, %.3f, %.3f\n", dRoll_/M_PI*180.0, dPitch_/M_PI*180.0, dYaw_/M_PI*180.0);
//  Pose3 pose(rot_1, pose_now_.translation());
//  new_factor.add(MakePriorFactor(pose, covariance_));
//  cout << "Add gravity:" << endl;
//  cout << pose << endl;
//  printf("%.3f, %.3f, %.3f\n", dRoll_/M_PI*180.0, dPitch_/M_PI*180.0, dYaw_/M_PI*180.0);
//  getchar();

  Pose3 pose_new = last_pose.compose(new_odometry);
  pose_new.print("pose init:\n");
  printf("%.3f, %.3f, %.3f\n", pose_new.rotation().roll()/M_PI*180.0,pose_new.rotation().pitch()/M_PI*180.0,pose_new.rotation().yaw()/M_PI*180.0);
  gtsam::Unit3 measurement3D(gravity(0), gravity(1), gravity(2));
  measurement3D.print("measurement3D");
  static gtsam::SharedNoiseModel model3D(gtsam::noiseModel::Isotropic::Sigma(2, 0.063245553));
  gtsam::BearingFactor<Pose3, Pose3> factor3D(key_, 0, measurement3D, model3D);
  new_factor.add(factor3D);
  m_gravityInds.push_back(key_);

  // Update ISAM2.
//  cout << "update start" << endl;
  isam_->update(new_factor, new_value);
//  cout << "update end" << endl;
  values_ = isam_->calculateEstimate();


//  NonlinearFactorGraph graph =  isam_->getFactorsUnsafe();
//  double dError =  graph.error(values_);
//  graph.printErrors(values_);
//  cout << "dError: " << dError << endl;
//  gtsam::Marginals marginals(graph, values_);
//  cout.precision(2);
//  marginals.print();
//  cout << "marginals.marginalCovariance(key_):" << endl << marginals.marginalCovariance(key_) << endl;
//  marginals.marginalFactor(key_)->print();

//  values_.insert(new_value);
  gtsam::Pose3 aa = values_.at<gtsam::Pose3>(key_);
  aa.print("pose update:\n");
  printf("%.3f, %.3f, %.3f\n", aa.rotation().roll()/M_PI*180.0,aa.rotation().pitch()/M_PI*180.0,aa.rotation().yaw()/M_PI*180.0);

//  Pose3 pose_now = values_.at<Pose3>(key_);
//  pcl::PointXYZRGB pt;
//  pt.x = float(pose_now.translation().x());
//  pt.y = float(pose_now.translation().y());
//  pt.z = float(pose_now.translation().z());
//  pt.r = 255;
//  pt.g = 0;
//  pt.b = 0;
//  m_track.push_back(pt);

  // Assign output and get ready to go again!
  *key = key_++;



  // Is the odometry translation large enough to add a new keyframe to the graph?
//  odometry_ = odometry_.compose(new_odometry);
//  if (odometry_.translation().norm() > translation_threshold_) {
//      odometry_ = Pose3::identity();
//      return true;
//  }
  UpdateDispInfo();

    getchar();
  return false;
}

bool LaserLoopClosure::AddBetweenFactor(
    const gu::Transform3& delta, const LaserLoopClosure::Mat66& covariance, unsigned int* key) {
  if (key == NULL) {
    return false;
  }

  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactor(new_odometry, ToGtsam(covariance)));

  Pose3 last_pose = values_.at<Pose3>(key_ - 1);
  new_value.insert(key_, last_pose.compose(new_odometry));

  Pose3 pose_now_ = last_pose.compose(new_odometry);
  double dRoll_ = pose_now_.rotation().roll();
  double dPitch_ = pose_now_.rotation().pitch();
  double dYaw_ = pose_now_.rotation().yaw();
//  gtsam::VectorValues vvs = isam_->getDelta();
//  double dError = isam_->error(vvs);
  printf("roll:%.3f, pitch:%.3f, yaw:%.3f\n", dRoll_/M_PI*180,
         dPitch_/M_PI*180, dYaw_/M_PI*180);
//  cout << dError << endl;
 
  // Update ISAM2.
//  cout << "update start" << endl;
  isam_->update(new_factor, new_value);
//  cout << "update end" << endl;
  values_ = isam_->calculateEstimate();
//  values_.insert(new_value);

  Pose3 pose_now = values_.at<Pose3>(key_);
  pcl::PointXYZRGB pt;
  pt.x = float(pose_now.translation().x());
  pt.y = float(pose_now.translation().y());
  pt.z = float(pose_now.translation().z());
  pt.r = 255;
  pt.g = 0;
  pt.b = 0;
  m_track.push_back(pt);

  // Assign output and get ready to go again!
  *key = key_++;

 

  // Is the odometry translation large enough to add a new keyframe to the graph?
  odometry_ = odometry_.compose(new_odometry);
  if (odometry_.translation().norm() > translation_threshold_) {
    odometry_ = Pose3::identity();
    return true;
  }

  return false;
}

bool LaserLoopClosure::AddKeyScanPair(unsigned int key,
                                      const PointCloud::ConstPtr& scan) {
  if (keyed_scans_.count(key)) {
    return false;
  }

  // Add the key and scan.
  keyed_scans_.insert(std::pair<unsigned int, PointCloud::ConstPtr>(key, scan));



  return true;
}

bool LaserLoopClosure::FindLoopClosures(
    unsigned int key, std::vector<unsigned int>* closure_keys) {
	m_Log << "Start Find loop" << endl;
	m_Log.flush();

	m_closureCondidates.clear();
	m_closureValid.clear();

  if (!check_for_loop_closures_)
    return false;

  if (closure_keys == NULL) {
    return false;
  }
  closure_keys->clear();

  if (std::fabs((double)key - (double)last_closure_key_) < poses_before_reclosing_)
    return false;

  const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(key));
  const PointCloud::ConstPtr scan1 = keyed_scans_[key];

  bool closed_loop = false;
  vector<gu::Transform3> valid_delta;
  vector<LaserLoopClosure::Mat66> valid_covariance;
  vector<float> valid_score;
  vector<unsigned int> valid_other_keys;
  for (const auto& keyed_pose : values_) {
    const unsigned int other_key = keyed_pose.key;
	
    if (other_key == key)
      continue;

	if (std::fabs((double)key - (double)other_key) < skip_recent_poses_)
      continue;

    if (!keyed_scans_.count(other_key))
      continue;

    const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(other_key));
    gu::Transform3 difference = gu::PoseDelta(pose1, pose2);
	difference.translation.Get(2) = 0.0;
//	difference.Print("1111");
// 	cout << "xxxxxxxxxxxxxxxxx" << endl;
// 	cout << "key:" << key << endl;
    if (difference.translation.Norm() < proximity_threshold_) 
	{
//		cout << "other_key:" << other_key << endl;
      const PointCloud::ConstPtr scan2 = keyed_scans_[other_key];

      gu::Transform3 delta;
      LaserLoopClosure::Mat66 covariance;
      float score;
      if (PerformICP(scan1, scan2, pose1, pose2, &delta, &covariance, score))
	  {
          valid_delta.push_back(delta);
          valid_covariance.push_back(covariance);
          valid_score.push_back(score);
          valid_other_keys.push_back(other_key);

		  m_Log << "Loop:" << other_key << endl;
		  m_Log.flush();
//        NonlinearFactorGraph new_factor;
//        new_factor.add(BetweenFactor<Pose3>(key, other_key, ToGtsam(delta),
//                                            ToGtsam(covariance)));
		m_Log << "update start:" << other_key << endl;
		m_Log.flush();
		m_Log << "value exist: " << key << ":" << isam_->valueExists(key) << endl;
		m_Log << "value exist: " << other_key << ":" << isam_->valueExists(other_key) << endl;
// 		string sssss;
// 		Values values = isam_->getLinearizationPoint();
// 		values.print(sssss);

//		getchar();
//		cout << "update start" << endl;
//        isam_->update(new_factor, Values(), gtsam::FactorIndices(),
//			boost::none, boost::none, boost::none, true);
//        isam_->update(new_factor, Values());
//		cout << "update end" << endl;
		m_Log << "update end:" << other_key << endl;
		m_Log.flush();
//        closed_loop = true;
//        last_closure_key_ = key;

//        loop_edges_.push_back(std::make_pair(key, other_key));
//        closure_keys->push_back(other_key);

// 		pcl::PointXYZRGB pt_;
// 		pt_.x = pose2.translation(0);
// 		pt_.y = pose2.translation(1);
// 		pt_.z = pose2.translation(2);
// 		pt_.g = 255.0;
// 		m_closureValid.push_back(pt_);
        m_Log << "LoopClousure:" <<
              scan1->header.seq << "," << scan1->header.stamp << "," <<
              scan2->header.seq << "," << scan2->header.stamp << "," <<
              delta.translation.Norm() << endl;
      }
	  pcl::PointXYZRGB pt_;
	  pt_.x = pose2.translation(0);
	  pt_.y = pose2.translation(1);
	  pt_.z = pose2.translation(2);
	  pt_.b = 255.0;
	  m_closureCondidates.push_back(pt_);
    }
  }

  int best_ind = -1;
  float best_score = 100000.f;
  for (unsigned int i = 0; i < valid_delta.size(); i++)
  {
        if (valid_score[i] <= best_score)
        {
            best_score = valid_score[i];
            best_ind = i;
        }
  }
  if (best_ind >= 0)
  {
      NonlinearFactorGraph new_factor;
      new_factor.add(BetweenFactor<Pose3>(key, valid_other_keys[best_ind], ToGtsam(valid_delta[best_ind]),
                                          ToGtsam(valid_covariance[best_ind])));
      isam_->update(new_factor, Values());
      closed_loop = true;
      last_closure_key_ = key;
      loop_edges_.push_back(std::make_pair(key, valid_other_keys[best_ind]));
      closure_keys->push_back(valid_other_keys[best_ind]);
      cout << "best closed loop: " << key << "-" << valid_other_keys[best_ind]
              << "  scroe:" << valid_score[best_ind] << endl;
  }

  m_Log << "calculateEstimate: start" << endl;
  m_Log.flush();
  values_ = isam_->calculateEstimate();
  m_Log << "calculateEstimate: end" << endl;
  m_Log.flush();

  if (closed_loop)
  {
      UpdateDispInfo();
//	  m_track.clear();
//	  for (const auto& keyed_pose : values_)
//	  {
//		  const unsigned int key = keyed_pose.key;

//		  if (!keyed_scans_.count(key))
//			  continue;
//          if (key == 0)
//              continue;

//		  Pose3 pose_now = values_.at<Pose3>(key);
//		  pcl::PointXYZRGB pt;
//		  pt.x = float(pose_now.translation().x());
//		  pt.y = float(pose_now.translation().y());
//		  pt.z = float(pose_now.translation().z());
//		  pt.r = 255;
//		  pt.g = 0;
//		  pt.b = 0;
//		  m_track.push_back(pt);
//	  }
//	  m_LoopLines.clear();
//	  for (unsigned int i = 0; i < loop_edges_.size(); i++)
//	  {
//		  const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(loop_edges_[i].first));
//		  const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(loop_edges_[i].second));
//		  pcl::PointXYZ pt0, pt1;
//		  pt0.x = pose1.translation(0);
//		  pt0.y = pose1.translation(1);
//		  pt0.z = pose1.translation(2);
//		  pt1.x = pose2.translation(0);
//		  pt1.y = pose2.translation(1);
//		  pt1.z = pose2.translation(2);
//		  m_LoopLines.push_back(std::make_pair(pt0, pt1));
//	  }
  }

  return closed_loop;
}

void LaserLoopClosure::GetMaximumLikelihoodPoints(PointCloud* points) {
  if (points == NULL) {
    return;
  }
  points->points.clear();

  for (const auto& keyed_pose : values_) {
    const unsigned int key = keyed_pose.key;

    if (!keyed_scans_.count(key))
      continue;

    const gu::Transform3 pose = ToGu(values_.at<Pose3>(key));
    Eigen::Matrix4d b2w;
    b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
    b2w.block(0, 3, 3, 1) = pose.translation.Eigen();

    PointCloud scan_world;
    pcl::transformPointCloud(*keyed_scans_[key], scan_world, b2w);

    *points += scan_world;
  }
}

gu::Transform3 LaserLoopClosure::GetLastPose() const {
	if (key_ > 1) {
		return ToGu(values_.at<Pose3>(key_ - 1));
  } else {
    return ToGu(values_.at<Pose3>(0));
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LaserLoopClosure::GetTrack()
{
	return m_track.makeShared();
}

gu::Transform3 LaserLoopClosure::ToGu(const Pose3& pose) const {
  gu::Transform3 out;
  out.translation(0) = pose.translation().x();
  out.translation(1) = pose.translation().y();
  out.translation(2) = pose.translation().z();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      out.rotation(i, j) = pose.rotation().matrix()(i, j);
  }

  return out;
}

Pose3 LaserLoopClosure::ToGtsam(const gu::Transform3& pose) const {
 // Vector3 t;
	gtsam::Point3 t(pose.translation(0), pose.translation(1), pose.translation(2));

	Rot3 r(pose.rotation(0, 0), pose.rotation(0, 1), pose.rotation(0, 2),
			pose.rotation(1, 0), pose.rotation(1, 1), pose.rotation(1, 2),
			pose.rotation(2, 0), pose.rotation(2, 1), pose.rotation(2, 2));
	return Pose3(r, t);
}

LaserLoopClosure::Mat66 LaserLoopClosure::ToGu(
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) const {
	gtsam::Matrix gtsam_covariance;
	covariance->Covariance(gtsam_covariance);
	LaserLoopClosure::Mat66 out;

	for (int i = 0; i < 6; ++i)
		for (int j = 0; j < 6; ++j)
			out(i, j) = gtsam_covariance(i, j);
  return out;
}

LaserLoopClosure::Gaussian::shared_ptr LaserLoopClosure::ToGtsam(
    const LaserLoopClosure::Mat66& covariance) const {
	gtsam::Matrix6 gtsam_covariance;

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      gtsam_covariance(i, j) = covariance(i, j);

  return Gaussian::Covariance(gtsam_covariance);
}

PriorFactor<Pose3> LaserLoopClosure::MakePriorFactor(
    const Pose3& pose,
    const LaserLoopClosure::Diagonal::shared_ptr& covariance) {
	return PriorFactor<Pose3>(key_, pose, covariance);
}

BetweenFactor<Pose3> LaserLoopClosure::MakeBetweenFactor(
    const Pose3& delta,
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) {
	odometry_edges_.push_back(std::make_pair(key_ - 1, key_));
	return BetweenFactor<Pose3>(key_ - 1, key_, delta, covariance);
}

bool LaserLoopClosure::PerformICP(const PointCloud::ConstPtr& scan1,
                                  const PointCloud::ConstPtr& scan2,
                                  const gu::Transform3& pose1,
                                  const gu::Transform3& pose2,
                                  gu::Transform3* delta,
                                  LaserLoopClosure::Mat66* covariance, float& score) {
    score = 100.0;
  if (delta == NULL || covariance == NULL) {
    return false;
  }

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setTransformationEpsilon(icp_tf_epsilon_);
  icp.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp.setMaximumIterations(icp_iterations_);
  icp.setRANSACIterations(0);

  PointCloud::Ptr scan1_filtered(new PointCloud);
  PointCloud::Ptr scan2_filtered(new PointCloud);
  filter_.Filter(scan1, scan1_filtered);
  filter_.Filter(scan2, scan2_filtered);
  const Eigen::Matrix<double, 3, 3> R1 = pose1.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> t1 = pose1.translation.Eigen();
  Eigen::Matrix4d body1_to_world;
  body1_to_world.block(0, 0, 3, 3) = R1;
  body1_to_world.block(0, 3, 3, 1) = t1;

  const Eigen::Matrix<double, 3, 3> R2 = pose2.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> t2 = pose2.translation.Eigen();
  Eigen::Matrix4d body2_to_world;
  body2_to_world.block(0, 0, 3, 3) = R2;
  body2_to_world.block(0, 3, 3, 1) = t2;

  PointCloud::Ptr source(new PointCloud);
  pcl::transformPointCloud(*scan1_filtered, *source, body1_to_world);
  icp.setInputSource(source);

  PointCloud::Ptr target(new PointCloud);
  pcl::transformPointCloud(*scan2_filtered, *target, body2_to_world);
  icp.setInputTarget(target);

  PointCloud unused_result;
  icp.align(unused_result);

  const Eigen::Matrix4f T = icp.getFinalTransformation();
  gu::Transform3 delta_icp;
  delta_icp.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  delta_icp.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                T(1, 0), T(1, 1), T(1, 2),
                                T(2, 0), T(2, 1), T(2, 2));

  if (!icp.hasConverged())
    return false;
  stringstream ss;
  ss << "getFitnessScore: " << icp.getFitnessScore() << "     " << max_tolerable_fitness_ << std::endl;
  score = icp.getFitnessScore();
  std::cout << ss.str();
  m_Log << ss.str();
  m_Log.flush();
  if (icp.getFitnessScore() > max_tolerable_fitness_) {
    return false;
  }

  const gu::Transform3 update =
      gu::PoseUpdate(gu::PoseInverse(pose1),
                     gu::PoseUpdate(gu::PoseInverse(delta_icp), pose1));

  *delta = gu::PoseUpdate(update, gu::PoseDelta(pose1, pose2));
  m_Log << *delta << endl;
  m_Log.flush();

  covariance->Zeros();
  for (int i = 0; i < 3; ++i)
    (*covariance)(i, i) = 0.01;
  for (int i = 3; i < 6; ++i)
    (*covariance)(i, i) = 0.04;

  source->header.frame_id = fixed_frame_id_;
  target->header.frame_id = fixed_frame_id_;

  return true;
}
