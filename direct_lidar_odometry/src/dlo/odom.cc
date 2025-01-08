/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/odom.h"
#include <chrono>
#include <std_msgs/Float32.h>
using namespace std::chrono;

ros::Publisher calc_time_pub;

std::atomic<bool> dlo::OdomNode::abort_(false);


/**
 * Constructor
 **/

dlo::OdomNode::OdomNode(ros::NodeHandle node_handle) : nh(node_handle) {

  this->getParams();

  this->stop_publish_thread = false;
  this->stop_publish_keyframe_thread = false;
  this->stop_metrics_thread = false;
  this->stop_debug_thread = false;

  this->dlo_initialized = false;
  this->imu_calibrated = false;

  this->icp_sub = this->nh.subscribe("pointcloud", 1, &dlo::OdomNode::icpCB, this);
  this->imu_sub = this->nh.subscribe("imu", 1, &dlo::OdomNode::imuCB, this);

  this->odom_pub = this->nh.advertise<nav_msgs::Odometry>("odom", 1);
  this->pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  this->kf_pub = this->nh.advertise<nav_msgs::Odometry>("kfs", 1, true);
  this->keyframe_pub = this->nh.advertise<sensor_msgs::PointCloud2>("keyframe", 1, true);
  this->save_traj_srv = this->nh.advertiseService("save_traj", &dlo::OdomNode::saveTrajectory, this);
  calc_time_pub = this->nh.advertise<std_msgs::Float32>("/calc_time", 10);

  this->odom.pose.pose.position.x = 0.;
  this->odom.pose.pose.position.y = 0.;
  this->odom.pose.pose.position.z = 0.;
  this->odom.pose.pose.orientation.w = 1.;
  this->odom.pose.pose.orientation.x = 0.;
  this->odom.pose.pose.orientation.y = 0.;
  this->odom.pose.pose.orientation.z = 0.;
  this->odom.pose.covariance = {0.};

  this->origin = Eigen::Vector3f(0., 0., 0.);

  this->T = Eigen::Matrix4f::Identity();
  this->T_s2s = Eigen::Matrix4f::Identity();
  this->T_s2s_prev = Eigen::Matrix4f::Identity();

  this->pose_s2s = Eigen::Vector3f(0., 0., 0.);
  this->rotq_s2s = Eigen::Quaternionf(1., 0., 0., 0.);

  this->pose = Eigen::Vector3f(0., 0., 0.);
  this->rotq = Eigen::Quaternionf(1., 0., 0., 0.);

  this->imu_SE3 = Eigen::Matrix4f::Identity();

  this->imu_bias.gyro.x = 0.;
  this->imu_bias.gyro.y = 0.;
  this->imu_bias.gyro.z = 0.;
  this->imu_bias.accel.x = 0.;
  this->imu_bias.accel.y = 0.;
  this->imu_bias.accel.z = 0.;

  this->imu_meas.stamp = 0.;
  this->imu_meas.ang_vel.x = 0.;
  this->imu_meas.ang_vel.y = 0.;
  this->imu_meas.ang_vel.z = 0.;
  this->imu_meas.lin_accel.x = 0.;
  this->imu_meas.lin_accel.y = 0.;
  this->imu_meas.lin_accel.z = 0.;

  this->imu_buffer.set_capacity(this->imu_buffer_size_);
  this->first_imu_time = 0.;

  this->original_scan = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->current_scan = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->current_scan_t = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);

  this->keyframe_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->keyframes_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->num_keyframes = 0;

  this->submap_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->submap_hasChanged = true;
  this->submap_kf_idx_prev.clear();

  this->source_cloud = nullptr;
  this->target_cloud = nullptr;

  this->convex_hull.setDimension(3);
  this->concave_hull.setDimension(3);
  this->concave_hull.setAlpha(this->keyframe_thresh_dist_);
  this->concave_hull.setKeepInformation(true);

  this->gicp_s2s.setCorrespondenceRandomness(this->gicps2s_k_correspondences_);
  this->gicp_s2s.setMaxCorrespondenceDistance(this->gicps2s_max_corr_dist_);
  this->gicp_s2s.setMaximumIterations(this->gicps2s_max_iter_);
  this->gicp_s2s.setTransformationEpsilon(this->gicps2s_transformation_ep_);
  this->gicp_s2s.setEuclideanFitnessEpsilon(this->gicps2s_euclidean_fitness_ep_);
  this->gicp_s2s.setRANSACIterations(this->gicps2s_ransac_iter_);
  this->gicp_s2s.setRANSACOutlierRejectionThreshold(this->gicps2s_ransac_inlier_thresh_);

  this->gicp.setCorrespondenceRandomness(this->gicps2m_k_correspondences_);
  this->gicp.setMaxCorrespondenceDistance(this->gicps2m_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicps2m_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicps2m_transformation_ep_);
  this->gicp.setEuclideanFitnessEpsilon(this->gicps2m_euclidean_fitness_ep_);
  this->gicp.setRANSACIterations(this->gicps2m_ransac_iter_);
  this->gicp.setRANSACOutlierRejectionThreshold(this->gicps2m_ransac_inlier_thresh_);

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  this->gicp_s2s.setSearchMethodSource(temp, true);
  this->gicp_s2s.setSearchMethodTarget(temp, true);
  this->gicp.setSearchMethodSource(temp, true);
  this->gicp.setSearchMethodTarget(temp, true);

  this->crop.setNegative(true);
  this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
  this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));

  this->vf_scan.setLeafSize(this->vf_scan_res_, this->vf_scan_res_, this->vf_scan_res_);
  this->vf_submap.setLeafSize(this->vf_submap_res_, this->vf_submap_res_, this->vf_submap_res_);

  this->metrics.spaciousness.push_back(0.);

  // CPU Specs
  char CPUBrandString[0x40];
  memset(CPUBrandString, 0, sizeof(CPUBrandString));
  this->cpu_type = "";

#ifdef HAS_CPUID
  unsigned int CPUInfo[4] = {0,0,0,0};
  __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
  unsigned int nExIds = CPUInfo[0];
  for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
    __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    if (i == 0x80000002)
      memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000003)
      memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000004)
      memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
  }

  this->cpu_type = CPUBrandString;
  boost::trim(this->cpu_type);
#endif

  FILE* file;
  struct tms timeSample;
  char line[128];

  this->lastCPU = times(&timeSample);
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;

  file = fopen("/proc/cpuinfo", "r");
  this->numProcessors = 0;
  while(fgets(line, 128, file) != NULL) {
      if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
  }
  fclose(file);

  ROS_INFO("DLO Odom Node Initialized");

}


/**
 * Destructor
 **/

dlo::OdomNode::~OdomNode() {}



/**
 * Odom Node Parameters
 **/

void dlo::OdomNode::getParams() {

  // Version
  ros::param::param<std::string>("~dlo/version", this->version_, "0.0.0");

  // Frames
  ros::param::param<std::string>("~dlo/odomNode/odom_frame", this->odom_frame, "odom");
  ros::param::param<std::string>("~dlo/odomNode/child_frame", this->child_frame, "base_link");

  // Get Node NS and Remove Leading Character
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,1);

  // Concatenate Frame Name Strings
  this->odom_frame = ns + "/" + this->odom_frame;
  this->child_frame = ns + "/" + this->child_frame;

  // Gravity alignment
  ros::param::param<bool>("~dlo/gravityAlign", this->gravity_align_, false);

  // Keyframe Threshold
  ros::param::param<double>("~dlo/odomNode/keyframe/threshD", this->keyframe_thresh_dist_, 0.1);
  ros::param::param<double>("~dlo/odomNode/keyframe/threshR", this->keyframe_thresh_rot_, 1.0);

  // Submap
  ros::param::param<int>("~dlo/odomNode/submap/keyframe/knn", this->submap_knn_, 10);
  ros::param::param<int>("~dlo/odomNode/submap/keyframe/kcv", this->submap_kcv_, 10);
  ros::param::param<int>("~dlo/odomNode/submap/keyframe/kcc", this->submap_kcc_, 10);

  // Initial Position
  ros::param::param<bool>("~dlo/odomNode/initialPose/use", this->initial_pose_use_, false);

  double px, py, pz, qx, qy, qz, qw;
  ros::param::param<double>("~dlo/odomNode/initialPose/position/x", px, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/position/y", py, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/position/z", pz, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/orientation/w", qw, 1.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/orientation/x", qx, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/orientation/y", qy, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/orientation/z", qz, 0.0);
  this->initial_position_ = Eigen::Vector3f(px, py, pz);
  this->initial_orientation_ = Eigen::Quaternionf(qw, qx, qy, qz);

  // Crop Box Filter
  ros::param::param<bool>("~dlo/odomNode/preprocessing/cropBoxFilter/use", this->crop_use_, false);
  ros::param::param<double>("~dlo/odomNode/preprocessing/cropBoxFilter/size", this->crop_size_, 1.0);

  // Voxel Grid Filter
  ros::param::param<bool>("~dlo/odomNode/preprocessing/voxelFilter/scan/use", this->vf_scan_use_, true);
  ros::param::param<double>("~dlo/odomNode/preprocessing/voxelFilter/scan/res", this->vf_scan_res_, 0.05);
  ros::param::param<bool>("~dlo/odomNode/preprocessing/voxelFilter/submap/use", this->vf_submap_use_, false);
  ros::param::param<double>("~dlo/odomNode/preprocessing/voxelFilter/submap/res", this->vf_submap_res_, 0.1);

  // Adaptive Parameters
  ros::param::param<bool>("~dlo/adaptiveParams", this->adaptive_params_use_, false);

  // IMU
  ros::param::param<bool>("~dlo/imu", this->imu_use_, false);
  ros::param::param<int>("~dlo/odomNode/imu/calibTime", this->imu_calib_time_, 3);
  ros::param::param<int>("~dlo/odomNode/imu/bufferSize", this->imu_buffer_size_, 2000);

  // GICP
  ros::param::param<int>("~dlo/odomNode/gicp/minNumPoints", this->gicp_min_num_points_, 100);
  ros::param::param<int>("~dlo/odomNode/gicp/s2s/kCorrespondences", this->gicps2s_k_correspondences_, 20);
  ros::param::param<double>("~dlo/odomNode/gicp/s2s/maxCorrespondenceDistance", this->gicps2s_max_corr_dist_, std::sqrt(std::numeric_limits<double>::max()));
  ros::param::param<int>("~dlo/odomNode/gicp/s2s/maxIterations", this->gicps2s_max_iter_, 64);
  ros::param::param<double>("~dlo/odomNode/gicp/s2s/transformationEpsilon", this->gicps2s_transformation_ep_, 0.0005);
  ros::param::param<double>("~dlo/odomNode/gicp/s2s/euclideanFitnessEpsilon", this->gicps2s_euclidean_fitness_ep_, -std::numeric_limits<double>::max());
  ros::param::param<int>("~dlo/odomNode/gicp/s2s/ransac/iterations", this->gicps2s_ransac_iter_, 0);
  ros::param::param<double>("~dlo/odomNode/gicp/s2s/ransac/outlierRejectionThresh", this->gicps2s_ransac_inlier_thresh_, 0.05);
  ros::param::param<int>("~dlo/odomNode/gicp/s2m/kCorrespondences", this->gicps2m_k_correspondences_, 20);
  ros::param::param<double>("~dlo/odomNode/gicp/s2m/maxCorrespondenceDistance", this->gicps2m_max_corr_dist_, std::sqrt(std::numeric_limits<double>::max()));
  ros::param::param<int>("~dlo/odomNode/gicp/s2m/maxIterations", this->gicps2m_max_iter_, 64);
  ros::param::param<double>("~dlo/odomNode/gicp/s2m/transformationEpsilon", this->gicps2m_transformation_ep_, 0.0005);
  ros::param::param<double>("~dlo/odomNode/gicp/s2m/euclideanFitnessEpsilon", this->gicps2m_euclidean_fitness_ep_, -std::numeric_limits<double>::max());
  ros::param::param<int>("~dlo/odomNode/gicp/s2m/ransac/iterations", this->gicps2m_ransac_iter_, 0);
  ros::param::param<double>("~dlo/odomNode/gicp/s2m/ransac/outlierRejectionThresh", this->gicps2m_ransac_inlier_thresh_, 0.05);

}


/**
 * Start Odom Thread
 **/

void dlo::OdomNode::start() {
  ROS_INFO("Starting DLO Odometry Node");

  printf("\033[2J\033[1;1H");
  std::cout << std::endl << "==== Direct LiDAR Odometry v" << this->version_ << " ====" << std::endl << std::endl;

}


/**
 * Stop Odom Thread
 **/

void dlo::OdomNode::stop() {
  ROS_WARN("Stopping DLO Odometry Node");

  this->stop_publish_thread = true;
  if (this->publish_thread.joinable()) {
    this->publish_thread.join();
  }

  this->stop_publish_keyframe_thread = true;
  if (this->publish_keyframe_thread.joinable()) {
    this->publish_keyframe_thread.join();
  }

  this->stop_metrics_thread = true;
  if (this->metrics_thread.joinable()) {
    this->metrics_thread.join();
  }

  this->stop_debug_thread = true;
  if (this->debug_thread.joinable()) {
    this->debug_thread.join();
  }

  ros::shutdown();
}


/**
 * Abort Timer Callback
 **/

void dlo::OdomNode::abortTimerCB(const ros::TimerEvent& e) {
  if (abort_) {
    stop();
  }
}


/**
 * Publish to ROS
 **/

void dlo::OdomNode::publishToROS() {
  this->publishPose();
  this->publishTransform();
}


/**
 * Publish Pose
 **/

void dlo::OdomNode::publishPose() {

  // Sign flip check
  static Eigen::Quaternionf q_diff{1., 0., 0., 0.};
  static Eigen::Quaternionf q_last{1., 0., 0., 0.};

  q_diff = q_last.conjugate()*this->rotq;

  // If q_diff has negative real part then there was a sign flip
  if (q_diff.w() < 0) {
    this->rotq.w() = -this->rotq.w();
    this->rotq.vec() = -this->rotq.vec();
  }

  q_last = this->rotq;

  this->odom.pose.pose.position.x = this->pose[0];
  this->odom.pose.pose.position.y = this->pose[1];
  this->odom.pose.pose.position.z = this->pose[2];

  this->odom.pose.pose.orientation.w = this->rotq.w();
  this->odom.pose.pose.orientation.x = this->rotq.x();
  this->odom.pose.pose.orientation.y = this->rotq.y();
  this->odom.pose.pose.orientation.z = this->rotq.z();

  this->odom.header.stamp = this->scan_stamp;
  this->odom.header.frame_id = this->odom_frame;
  this->odom.child_frame_id = this->child_frame;
  this->odom_pub.publish(this->odom);

  this->pose_ros.header.stamp = this->scan_stamp;
  this->pose_ros.header.frame_id = this->odom_frame;

  this->pose_ros.pose.position.x = this->pose[0];
  this->pose_ros.pose.position.y = this->pose[1];
  this->pose_ros.pose.position.z = this->pose[2];

  this->pose_ros.pose.orientation.w = this->rotq.w();
  this->pose_ros.pose.orientation.x = this->rotq.x();
  this->pose_ros.pose.orientation.y = this->rotq.y();
  this->pose_ros.pose.orientation.z = this->rotq.z();

  this->pose_pub.publish(this->pose_ros);
}


/**
 * Publish Transform
 **/

void dlo::OdomNode::publishTransform() {

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = this->scan_stamp;
  transformStamped.header.frame_id = this->odom_frame;
  transformStamped.child_frame_id = this->child_frame;

  transformStamped.transform.translation.x = this->pose[0];
  transformStamped.transform.translation.y = this->pose[1];
  transformStamped.transform.translation.z = this->pose[2];

  transformStamped.transform.rotation.w = this->rotq.w();
  transformStamped.transform.rotation.x = this->rotq.x();
  transformStamped.transform.rotation.y = this->rotq.y();
  transformStamped.transform.rotation.z = this->rotq.z();

  br.sendTransform(transformStamped);

}


/**
 * Publish Keyframe Pose and Scan
 **/

void dlo::OdomNode::publishKeyframe() {

  // Publish keyframe pose
  this->kf.header.stamp = this->scan_stamp;
  this->kf.header.frame_id = this->odom_frame;
  this->kf.child_frame_id = this->child_frame;

  this->kf.pose.pose.position.x = this->pose[0];
  this->kf.pose.pose.position.y = this->pose[1];
  this->kf.pose.pose.position.z = this->pose[2];

  this->kf.pose.pose.orientation.w = this->rotq.w();
  this->kf.pose.pose.orientation.x = this->rotq.x();
  this->kf.pose.pose.orientation.y = this->rotq.y();
  this->kf.pose.pose.orientation.z = this->rotq.z();

  this->kf_pub.publish(this->kf);

  // Publish keyframe scan
  if (this->keyframe_cloud->points.size() == this->keyframe_cloud->width * this->keyframe_cloud->height) {
    sensor_msgs::PointCloud2 keyframe_cloud_ros;
    pcl::toROSMsg(*this->keyframe_cloud, keyframe_cloud_ros);
    keyframe_cloud_ros.header.stamp = this->scan_stamp;
    keyframe_cloud_ros.header.frame_id = this->odom_frame;
    this->keyframe_pub.publish(keyframe_cloud_ros);
  }

}


/**
 * Preprocessing
 **/

void dlo::OdomNode::preprocessPoints() {

  // Original Scan
  *this->original_scan = *this->current_scan;

  // Remove NaNs
  std::vector<int> idx;
  this->current_scan->is_dense = false;
  pcl::removeNaNFromPointCloud(*this->current_scan, *this->current_scan, idx);

  // Crop Box Filter
  if (this->crop_use_) {
    this->crop.setInputCloud(this->current_scan);
    this->crop.filter(*this->current_scan);
  }

  // Voxel Grid Filter
  if (this->vf_scan_use_) {
    this->vf_scan.setInputCloud(this->current_scan);
    this->vf_scan.filter(*this->current_scan);
  }

}


/**
 * Initialize Input Target
 **/

void dlo::OdomNode::initializeInputTarget() {

  this->prev_frame_stamp = this->curr_frame_stamp;

  // Convert ros message
  this->target_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->target_cloud = this->current_scan;
  this->gicp_s2s.setInputTarget(this->target_cloud);
  this->gicp_s2s.calculateTargetCovariances();

  // initialize keyframes
  pcl::PointCloud<PointType>::Ptr first_keyframe (new pcl::PointCloud<PointType>);
  pcl::transformPointCloud (*this->target_cloud, *first_keyframe, this->T);

  // voxelization for submap
  if (this->vf_submap_use_) {
    this->vf_submap.setInputCloud(first_keyframe);
    this->vf_submap.filter(*first_keyframe);
  }

  // keep history of keyframes
  this->keyframes.push_back(std::make_pair(std::make_pair(this->pose, this->rotq), first_keyframe));
  *this->keyframes_cloud += *first_keyframe;
  *this->keyframe_cloud = *first_keyframe;

  // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be overwritten by setInputSources())
  this->gicp_s2s.setInputSource(this->keyframe_cloud);
  this->gicp_s2s.calculateSourceCovariances();
  this->keyframe_normals.push_back(this->gicp_s2s.getSourceCovariances());

  this->publish_keyframe_thread = std::thread( &dlo::OdomNode::publishKeyframe, this );
  this->publish_keyframe_thread.detach();

  ++this->num_keyframes;

}


/**
 * Set Input Sources
 **/

void dlo::OdomNode::setInputSources(){

  // set the input source for the S2S gicp
  // this builds the KdTree of the source cloud
  // this does not build the KdTree for s2m because force_no_update is true
  this->gicp_s2s.setInputSource(this->current_scan);

  // set pcl::Registration input source for S2M gicp using custom NanoGICP function
  this->gicp.registerInputSource(this->current_scan);

  // now set the KdTree of S2M gicp using previously built KdTree
  this->gicp.source_kdtree_ = this->gicp_s2s.source_kdtree_;
  this->gicp.source_covs_.clear();

}


/**
 * Gravity Alignment
 **/

void dlo::OdomNode::gravityAlign() {

  // get average acceleration vector for 1 second and normalize
  Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
  const double then = ros::Time::now().toSec();
  int n=0;
  while ((ros::Time::now().toSec() - then) < 1.) {
    lin_accel[0] += this->imu_meas.lin_accel.x;
    lin_accel[1] += this->imu_meas.lin_accel.y;
    lin_accel[2] += this->imu_meas.lin_accel.z;
    ++n;
  }
  lin_accel[0] /= n; lin_accel[1] /= n; lin_accel[2] /= n;

  // normalize
  double lin_norm = sqrt(pow(lin_accel[0], 2) + pow(lin_accel[1], 2) + pow(lin_accel[2], 2));
  lin_accel[0] /= lin_norm; lin_accel[1] /= lin_norm; lin_accel[2] /= lin_norm;

  // define gravity vector (assume point downwards)
  Eigen::Vector3f grav;
  grav << 0, 0, 1;

  // calculate angle between the two vectors
  Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(lin_accel, grav);

  // normalize
  double grav_norm = sqrt(grav_q.w()*grav_q.w() + grav_q.x()*grav_q.x() + grav_q.y()*grav_q.y() + grav_q.z()*grav_q.z());
  grav_q.w() /= grav_norm; grav_q.x() /= grav_norm; grav_q.y() /= grav_norm; grav_q.z() /= grav_norm;

  // set gravity aligned orientation
  this->rotq = grav_q;
  this->T.block(0,0,3,3) = this->rotq.toRotationMatrix();
  this->T_s2s.block(0,0,3,3) = this->rotq.toRotationMatrix();
  this->T_s2s_prev.block(0,0,3,3) = this->rotq.toRotationMatrix();

  // rpy
  auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
  double yaw = euler[0] * (180.0/M_PI);
  double pitch = euler[1] * (180.0/M_PI);
  double roll = euler[2] * (180.0/M_PI);

  std::cout << "done" << std::endl;
  std::cout << "  Roll [deg]: " << roll << std::endl;
  std::cout << "  Pitch [deg]: " << pitch << std::endl << std::endl;
}


/**
 * Initialize 6DOF
 **/

void dlo::OdomNode::initializeDLO() {

  // Calibrate IMU
  if (!this->imu_calibrated && this->imu_use_) {
    return;
  }

  // Gravity Align
  if (this->gravity_align_ && this->imu_use_ && this->imu_calibrated && !this->initial_pose_use_) {
    std::cout << "Aligning to gravity... "; std::cout.flush();
    this->gravityAlign();
  }

  // Use initial known pose
  if (this->initial_pose_use_) {
    std::cout << "Setting known initial pose... "; std::cout.flush();

    // set known position
    this->pose = this->initial_position_;
    this->T.block(0,3,3,1) = this->pose;
    this->T_s2s.block(0,3,3,1) = this->pose;
    this->T_s2s_prev.block(0,3,3,1) = this->pose;
    this->origin = this->initial_position_;

    // set known orientation
    this->rotq = this->initial_orientation_;
    this->T.block(0,0,3,3) = this->rotq.toRotationMatrix();
    this->T_s2s.block(0,0,3,3) = this->rotq.toRotationMatrix();
    this->T_s2s_prev.block(0,0,3,3) = this->rotq.toRotationMatrix();

    std::cout << "done" << std::endl << std::endl;
  }

  this->dlo_initialized = true;
  std::cout << "DLO initialized! Starting localization..." << std::endl;

}


/**
 * ICP Point Cloud Callback
 **/

void dlo::OdomNode::icpCB(const sensor_msgs::PointCloud2ConstPtr& pc) {

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  double then = ros::Time::now().toSec();
  this->scan_stamp = pc->header.stamp;
  this->curr_frame_stamp = pc->header.stamp.toSec();

  // If there are too few points in the pointcloud, try again
  this->current_scan = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*pc, *this->current_scan);
  if (this->current_scan->points.size() < this->gicp_min_num_points_) {
    ROS_WARN("Low number of points!");
    return;
  }

  // DLO Initialization procedures (IMU calib, gravity align)
  if (!this->dlo_initialized) {
    this->initializeDLO();
    return;
  }

  // Preprocess points
  this->preprocessPoints();

  // Compute Metrics
  this->metrics_thread = std::thread( &dlo::OdomNode::computeMetrics, this );
  this->metrics_thread.detach();

  // Set Adaptive Parameters
  if (this->adaptive_params_use_){
    this->setAdaptiveParams();
  }

  // Set initial frame as target
  if(this->target_cloud == nullptr) {
    this->initializeInputTarget();
    return;
  }

  // Set source frame
  this->source_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->source_cloud = this->current_scan;

  // Set new frame as input source for both gicp objects
  this->setInputSources();

  // Get the next pose via IMU + S2S + S2M
  this->getNextPose();

  // Update current keyframe poses and map
  this->updateKeyframes();

  // Update trajectory
  this->trajectory.push_back( std::make_pair(this->pose, this->rotq) );

  // Update next time stamp
  this->prev_frame_stamp = this->curr_frame_stamp;

  // Update some statistics
  this->comp_times.push_back(ros::Time::now().toSec() - then);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>( t2 - t1 ).count() / 1000.0;
  std_msgs::Float32 calc_time;
  calc_time.data = duration;
  calc_time_pub.publish(calc_time);

  // Publish stuff to ROS
  this->publish_thread = std::thread( &dlo::OdomNode::publishToROS, this );
  this->publish_thread.detach();

  // Debug statements and publish custom DLO message
  this->debug_thread = std::thread( &dlo::OdomNode::debug, this );
  this->debug_thread.detach();

}


/**
 * IMU Callback
 **/

void dlo::OdomNode::imuCB(const sensor_msgs::Imu::ConstPtr& imu) {

  if (!this->imu_use_) {
    return;
  }

  double ang_vel[3], lin_accel[3];

  // Get IMU samples
  ang_vel[0] = imu->angular_velocity.x;
  ang_vel[1] = imu->angular_velocity.y;
  ang_vel[2] = imu->angular_velocity.z;

  lin_accel[0] = imu->linear_acceleration.x;
  lin_accel[1] = imu->linear_acceleration.y;
  lin_accel[2] = imu->linear_acceleration.z;

  if (this->first_imu_time == 0.) {
    this->first_imu_time = imu->header.stamp.toSec();
  }

  // IMU calibration procedure - do for three seconds
  if (!this->imu_calibrated) {

    static int num_samples = 0;
    static bool print = true;

    if ((imu->header.stamp.toSec() - this->first_imu_time) < this->imu_calib_time_) {

      num_samples++;

      this->imu_bias.gyro.x += ang_vel[0];
      this->imu_bias.gyro.y += ang_vel[1];
      this->imu_bias.gyro.z += ang_vel[2];

      this->imu_bias.accel.x += lin_accel[0];
      this->imu_bias.accel.y += lin_accel[1];
      this->imu_bias.accel.z += lin_accel[2];

      if(print) {
        std::cout << "Calibrating IMU for " << this->imu_calib_time_ << " seconds... "; std::cout.flush();
        print = false;
      }

    } else {

      this->imu_bias.gyro.x /= num_samples;
      this->imu_bias.gyro.y /= num_samples;
      this->imu_bias.gyro.z /= num_samples;

      this->imu_bias.accel.x /= num_samples;
      this->imu_bias.accel.y /= num_samples;
      this->imu_bias.accel.z /= num_samples;

      this->imu_calibrated = true;

      std::cout << "done" << std::endl;
      std::cout << "  Gyro biases [xyz]: " << this->imu_bias.gyro.x << ", " << this->imu_bias.gyro.y << ", " << this->imu_bias.gyro.z << std::endl << std::endl;

    }

  } else {

    // Apply the calibrated bias to the new IMU measurements
    this->imu_meas.stamp = imu->header.stamp.toSec();

    this->imu_meas.ang_vel.x = ang_vel[0] - this->imu_bias.gyro.x;
    this->imu_meas.ang_vel.y = ang_vel[1] - this->imu_bias.gyro.y;
    this->imu_meas.ang_vel.z = ang_vel[2] - this->imu_bias.gyro.z;

    this->imu_meas.lin_accel.x = lin_accel[0];
    this->imu_meas.lin_accel.y = lin_accel[1];
    this->imu_meas.lin_accel.z = lin_accel[2];

    // Store into circular buffer
    this->mtx_imu.lock();
    this->imu_buffer.push_front(this->imu_meas);
    this->mtx_imu.unlock();

  }

}


/**
 * Get Next Pose
 **/

void dlo::OdomNode::getNextPose() {

  //
  // FRAME-TO-FRAME PROCEDURE
  //

  // Align using IMU prior if available
  pcl::PointCloud<PointType>::Ptr aligned (new pcl::PointCloud<PointType>);

  if (this->imu_use_) {
    this->integrateIMU();
    this->gicp_s2s.align(*aligned, this->imu_SE3);
  } else {
    this->gicp_s2s.align(*aligned);
  }

  // Get the local S2S transform
  Eigen::Matrix4f T_S2S = this->gicp_s2s.getFinalTransformation();

  // Get the global S2S transform
  this->propagateS2S(T_S2S);

  // reuse covariances from s2s for s2m
  this->gicp.source_covs_ = this->gicp_s2s.source_covs_;

  // Swap source and target (which also swaps KdTrees internally) for next S2S
  this->gicp_s2s.swapSourceAndTarget();

  //
  // FRAME-TO-SUBMAP
  //

  // Get current global submap
  this->getSubmapKeyframes();

  if (this->submap_hasChanged) {

    // Set the current global submap as the target cloud
    this->gicp.setInputTarget(this->submap_cloud);

    // Set target cloud's normals as submap normals
    this->gicp.setTargetCovariances( this->submap_normals );
  }

  // Align with current submap with global S2S transformation as initial guess
  this->gicp.align(*aligned, this->T_s2s);

  // Get final transformation in global frame
  this->T = this->gicp.getFinalTransformation();

  // Update the S2S transform for next propagation
  this->T_s2s_prev = this->T;

  // Update next global pose
  // Both source and target clouds are in the global frame now, so tranformation is global
  this->propagateS2M();

  // Set next target cloud as current source cloud
  *this->target_cloud = *this->source_cloud;

}


/**
 * Integrate IMU
 **/

void dlo::OdomNode::integrateIMU() {

  // Extract IMU data between the two frames
  std::vector<ImuMeas> imu_frame;

  for (const auto& i : this->imu_buffer) {

    // IMU data between two frames is when:
    //   current frame's timestamp minus imu timestamp is positive
    //   previous frame's timestamp minus imu timestamp is negative
    double curr_frame_imu_dt = this->curr_frame_stamp - i.stamp;
    double prev_frame_imu_dt = this->prev_frame_stamp - i.stamp;

    if (curr_frame_imu_dt >= 0. && prev_frame_imu_dt <= 0.) {

      imu_frame.push_back(i);

    }

  }

  // Sort measurements by time
  std::sort(imu_frame.begin(), imu_frame.end(), this->comparatorImu);

  // Relative IMU integration of gyro and accelerometer
  double curr_imu_stamp = 0.;
  double prev_imu_stamp = 0.;
  double dt;

  Eigen::Quaternionf q = Eigen::Quaternionf::Identity();

  for (uint32_t i = 0; i < imu_frame.size(); ++i) {

    if (prev_imu_stamp == 0.) {
      prev_imu_stamp = imu_frame[i].stamp;
      continue;
    }

    // Calculate difference in imu measurement times IN SECONDS
    curr_imu_stamp = imu_frame[i].stamp;
    dt = curr_imu_stamp - prev_imu_stamp;
    prev_imu_stamp = curr_imu_stamp;
    
    // Relative gyro propagation quaternion dynamics
    Eigen::Quaternionf qq = q;
    q.w() -= 0.5*( qq.x()*imu_frame[i].ang_vel.x + qq.y()*imu_frame[i].ang_vel.y + qq.z()*imu_frame[i].ang_vel.z ) * dt;
    q.x() += 0.5*( qq.w()*imu_frame[i].ang_vel.x - qq.z()*imu_frame[i].ang_vel.y + qq.y()*imu_frame[i].ang_vel.z ) * dt;
    q.y() += 0.5*( qq.z()*imu_frame[i].ang_vel.x + qq.w()*imu_frame[i].ang_vel.y - qq.x()*imu_frame[i].ang_vel.z ) * dt;
    q.z() += 0.5*( qq.x()*imu_frame[i].ang_vel.y - qq.y()*imu_frame[i].ang_vel.x + qq.w()*imu_frame[i].ang_vel.z ) * dt;

  }

  // Normalize quaternion
  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  q.w() /= norm; q.x() /= norm; q.y() /= norm; q.z() /= norm;

  // Store IMU guess
  this->imu_SE3 = Eigen::Matrix4f::Identity();
  this->imu_SE3.block(0, 0, 3, 3) = q.toRotationMatrix();

}


/**
 * Propagate S2S Alignment
 **/

void dlo::OdomNode::propagateS2S(Eigen::Matrix4f T) {

  this->T_s2s = this->T_s2s_prev * T;
  this->T_s2s_prev = this->T_s2s;

  this->pose_s2s   << this->T_s2s(0,3), this->T_s2s(1,3), this->T_s2s(2,3);
  this->rotSO3_s2s << this->T_s2s(0,0), this->T_s2s(0,1), this->T_s2s(0,2),
                      this->T_s2s(1,0), this->T_s2s(1,1), this->T_s2s(1,2),
                      this->T_s2s(2,0), this->T_s2s(2,1), this->T_s2s(2,2);

  Eigen::Quaternionf q(this->rotSO3_s2s);

  // Normalize quaternion
  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  q.w() /= norm; q.x() /= norm; q.y() /= norm; q.z() /= norm;
  this->rotq_s2s = q;

}


/**
 * Propagate S2M Alignment
 **/

void dlo::OdomNode::propagateS2M() {

  this->pose   << this->T(0,3), this->T(1,3), this->T(2,3);
  this->rotSO3 << this->T(0,0), this->T(0,1), this->T(0,2),
                  this->T(1,0), this->T(1,1), this->T(1,2),
                  this->T(2,0), this->T(2,1), this->T(2,2);

  Eigen::Quaternionf q(this->rotSO3);

  // Normalize quaternion
  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  q.w() /= norm; q.x() /= norm; q.y() /= norm; q.z() /= norm;
  this->rotq = q;

}


/**
 * Transform Current Scan
 **/

void dlo::OdomNode::transformCurrentScan() {
  this->current_scan_t = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  pcl::transformPointCloud (*this->current_scan, *this->current_scan_t, this->T);
}


/**
 * Compute Metrics
 **/

void dlo::OdomNode::computeMetrics() {
  this->computeSpaciousness();
}


/**
 * Compute Spaciousness of Current Scan
 **/

void dlo::OdomNode::computeSpaciousness() {

  // compute range of points
  std::vector<float> ds;

  for (int i = 0; i <= this->current_scan->points.size(); i++) {
    float d = std::sqrt(pow(this->current_scan->points[i].x, 2) + pow(this->current_scan->points[i].y, 2) + pow(this->current_scan->points[i].z, 2));
    ds.push_back(d);
  }

  // median
  std::nth_element(ds.begin(), ds.begin() + ds.size()/2, ds.end());
  float median_curr = ds[ds.size()/2];
  static float median_prev = median_curr;
  float median_lpf = 0.95*median_prev + 0.05*median_curr;
  median_prev = median_lpf;

  // push
  this->metrics.spaciousness.push_back( median_lpf );

}


/**
 * Convex Hull of Keyframes
 **/

void dlo::OdomNode::computeConvexHull() {

  // at least 4 keyframes for convex hull
  if (this->num_keyframes < 4) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);

  for (const auto& k : this->keyframes) {
    PointType pt;
    pt.x = k.first.first[0];
    pt.y = k.first.first[1];
    pt.z = k.first.first[2];
    cloud->push_back(pt);
  }

  // calculate the convex hull of the point cloud
  this->convex_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the convex hull
  pcl::PointCloud<PointType>::Ptr convex_points = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->convex_hull.reconstruct(*convex_points);

  pcl::PointIndices::Ptr convex_hull_point_idx = pcl::PointIndices::Ptr (new pcl::PointIndices);
  this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

  this->keyframe_convex.clear();
  for (int i=0; i<convex_hull_point_idx->indices.size(); ++i) {
    this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
  }

}


/**
 * Concave Hull of Keyframes
 **/

void dlo::OdomNode::computeConcaveHull() {

  // at least 5 keyframes for concave hull
  if (this->num_keyframes < 5) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);

  for (const auto& k : this->keyframes) {
    PointType pt;
    pt.x = k.first.first[0];
    pt.y = k.first.first[1];
    pt.z = k.first.first[2];
    cloud->push_back(pt);
  }

  // calculate the concave hull of the point cloud
  this->concave_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the concave hull
  pcl::PointCloud<PointType>::Ptr concave_points = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->concave_hull.reconstruct(*concave_points);

  pcl::PointIndices::Ptr concave_hull_point_idx = pcl::PointIndices::Ptr (new pcl::PointIndices);
  this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

  this->keyframe_concave.clear();
  for (int i=0; i<concave_hull_point_idx->indices.size(); ++i) {
    this->keyframe_concave.push_back(concave_hull_point_idx->indices[i]);
  }

}


/**
 * Update keyframes
 **/

void dlo::OdomNode::updateKeyframes() {

  // transform point cloud
  this->transformCurrentScan();

  // calculate difference in pose and rotation to all poses in trajectory
  float closest_d = std::numeric_limits<float>::infinity();
  int closest_idx = 0;
  int keyframes_idx = 0;

  int num_nearby = 0;

  for (const auto& k : this->keyframes) {

    // calculate distance between current pose and pose in keyframes
    float delta_d = sqrt( pow(this->pose[0] - k.first.first[0], 2) + pow(this->pose[1] - k.first.first[1], 2) + pow(this->pose[2] - k.first.first[2], 2) );

    // count the number nearby current pose
    if (delta_d <= this->keyframe_thresh_dist_ * 1.5){
      ++num_nearby;
    }

    // store into variable
    if (delta_d < closest_d) {
      closest_d = delta_d;
      closest_idx = keyframes_idx;
    }

    keyframes_idx++;

  }

  // get closest pose and corresponding rotation
  Eigen::Vector3f closest_pose = this->keyframes[closest_idx].first.first;
  Eigen::Quaternionf closest_pose_r = this->keyframes[closest_idx].first.second;

  // calculate distance between current pose and closest pose from above
  float dd = sqrt( pow(this->pose[0] - closest_pose[0], 2) + pow(this->pose[1] - closest_pose[1], 2) + pow(this->pose[2] - closest_pose[2], 2) );

  // calculate difference in orientation
  Eigen::Quaternionf dq = this->rotq * (closest_pose_r.inverse());

  float theta_rad = 2. * atan2(sqrt( pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2) ), dq.w());
  float theta_deg = theta_rad * (180.0/M_PI);

  // update keyframe
  bool newKeyframe = false;

  if (abs(dd) > this->keyframe_thresh_dist_ || abs(theta_deg) > this->keyframe_thresh_rot_) {
    newKeyframe = true;
  }
  if (abs(dd) <= this->keyframe_thresh_dist_) {
    newKeyframe = false;
  }
  if (abs(dd) <= this->keyframe_thresh_dist_ && abs(theta_deg) > this->keyframe_thresh_rot_ && num_nearby <= 1) {
    newKeyframe = true;
  }

  if (newKeyframe) {

    ++this->num_keyframes;

    // voxelization for submap
    if (this->vf_submap_use_) {
      this->vf_submap.setInputCloud(this->current_scan_t);
      this->vf_submap.filter(*this->current_scan_t);
    }

    // update keyframe vector
    this->keyframes.push_back(std::make_pair(std::make_pair(this->pose, this->rotq), this->current_scan_t));

    // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be overwritten by setInputSources())
    *this->keyframes_cloud += *this->current_scan_t;
    *this->keyframe_cloud = *this->current_scan_t;

    this->gicp_s2s.setInputSource(this->keyframe_cloud);
    this->gicp_s2s.calculateSourceCovariances();
    this->keyframe_normals.push_back(this->gicp_s2s.getSourceCovariances());

    this->publish_keyframe_thread = std::thread( &dlo::OdomNode::publishKeyframe, this );
    this->publish_keyframe_thread.detach();

  }

}


/**
 * Set Adaptive Parameters
 **/

void dlo::OdomNode::setAdaptiveParams() {

  // Set Keyframe Thresh from Spaciousness Metric
  if (this->metrics.spaciousness.back() > 20.0){
    this->keyframe_thresh_dist_ = 10.0;
  } else if (this->metrics.spaciousness.back() > 10.0 && this->metrics.spaciousness.back() <= 20.0) {
    this->keyframe_thresh_dist_ = 5.0;
  } else if (this->metrics.spaciousness.back() > 5.0 && this->metrics.spaciousness.back() <= 10.0) {
    this->keyframe_thresh_dist_ = 1.0;
  } else if (this->metrics.spaciousness.back() <= 5.0) {
    this->keyframe_thresh_dist_ = 0.5;
  }

  // set concave hull alpha
  this->concave_hull.setAlpha(this->keyframe_thresh_dist_);

}


/**
 * Push Submap Keyframe Indices
 **/
void dlo::OdomNode::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames) {

  // make sure dists is not empty
  if (!dists.size()) { return; }

  // maintain max heap of at most k elements
  std::priority_queue<float> pq;

  for (auto d : dists) {
    if (pq.size() >= k && pq.top() > d) {
      pq.push(d);
      pq.pop();
    } else if (pq.size() < k) {
      pq.push(d);
    }
  }

  // get the kth smallest element, which should be at the top of the heap
  float kth_element = pq.top();

  // get all elements smaller or equal to the kth smallest element
  for (int i = 0; i < dists.size(); ++i) {
    if (dists[i] <= kth_element)
      this->submap_kf_idx_curr.push_back(frames[i]);
  }

}


/**
 * Get Submap using Nearest Neighbor Keyframes
 **/

void dlo::OdomNode::getSubmapKeyframes() {

  // clear vector of keyframe indices to use for submap
  this->submap_kf_idx_curr.clear();

  //
  // TOP K NEAREST NEIGHBORS FROM ALL KEYFRAMES
  //

  // calculate distance between current pose and poses in keyframe set
  std::vector<float> ds;
  std::vector<int> keyframe_nn; int i=0;
  Eigen::Vector3f curr_pose = this->T_s2s.block(0,3,3,1);

  for (const auto& k : this->keyframes) {
    float d = sqrt( pow(curr_pose[0] - k.first.first[0], 2) + pow(curr_pose[1] - k.first.first[1], 2) + pow(curr_pose[2] - k.first.first[2], 2) );
    ds.push_back(d);
    keyframe_nn.push_back(i); i++;
  }

  // get indices for top K nearest neighbor keyframe poses
  this->pushSubmapIndices(ds, this->submap_knn_, keyframe_nn);

  //
  // TOP K NEAREST NEIGHBORS FROM CONVEX HULL
  //

  // get convex hull indices
  this->computeConvexHull();

  // get distances for each keyframe on convex hull
  std::vector<float> convex_ds;
  for (const auto& c : this->keyframe_convex) {
    convex_ds.push_back(ds[c]);
  }

  // get indicies for top kNN for convex hull
  this->pushSubmapIndices(convex_ds, this->submap_kcv_, this->keyframe_convex);

  //
  // TOP K NEAREST NEIGHBORS FROM CONCAVE HULL
  //

  // get concave hull indices
  this->computeConcaveHull();

  // get distances for each keyframe on concave hull
  std::vector<float> concave_ds;
  for (const auto& c : this->keyframe_concave) {
    concave_ds.push_back(ds[c]);
  }

  // get indicies for top kNN for convex hull
  this->pushSubmapIndices(concave_ds, this->submap_kcc_, this->keyframe_concave);

  //
  // BUILD SUBMAP
  //

  // concatenate all submap clouds and normals
  std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());

  // sort current and previous submap kf list of indices
  std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  std::sort(this->submap_kf_idx_prev.begin(), this->submap_kf_idx_prev.end());

  // check if submap has changed from previous iteration
  if (this->submap_kf_idx_curr == this->submap_kf_idx_prev){
    this->submap_hasChanged = false;
  } else {
    this->submap_hasChanged = true;

    // reinitialize submap cloud, normals
    pcl::PointCloud<PointType>::Ptr submap_cloud_ (boost::make_shared<pcl::PointCloud<PointType>>());
    this->submap_normals.clear();

    for (auto k : this->submap_kf_idx_curr) {

      // create current submap cloud
      *submap_cloud_ += *this->keyframes[k].second;

      // grab corresponding submap cloud's normals
      this->submap_normals.insert( std::end(this->submap_normals), std::begin(this->keyframe_normals[k]), std::end(this->keyframe_normals[k]) );
    }

    this->submap_cloud = submap_cloud_;
    this->submap_kf_idx_prev = this->submap_kf_idx_curr;
  }

}

bool dlo::OdomNode::saveTrajectory(direct_lidar_odometry::save_traj::Request& req,
                                   direct_lidar_odometry::save_traj::Response& res) {
  std::string kittipath = req.save_path + "/kitti_traj.txt";
  std::ofstream out_kitti(kittipath);

  std::cout << std::setprecision(2) << "Saving KITTI trajectory to " << kittipath << "... "; std::cout.flush();

  for (const auto& pose : this->trajectory) {
    const auto& t = pose.first;
    const auto& q = pose.second;
    // Write to Kitti Format
    auto R = q.normalized().toRotationMatrix();
    out_kitti << std::fixed << std::setprecision(9) 
      << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t.x() << " " 
      << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << t.y() << " " 
      << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t.z() << "\n";
  }

  std::cout << "done" << std::endl;
  res.success = true;
  return res.success;
}

/**
 * Debug Statements
 **/

void dlo::OdomNode::debug() {

  // Total length traversed
  double length_traversed = 0.;
  Eigen::Vector3f p_curr = Eigen::Vector3f(0., 0., 0.);
  Eigen::Vector3f p_prev = Eigen::Vector3f(0., 0., 0.);
  for (const auto& t : this->trajectory) {
    if (p_prev == Eigen::Vector3f(0., 0., 0.)) {
      p_prev = t.first;
      continue;
    }
    p_curr = t.first;
    double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));

    if (l >= 0.05) {
      length_traversed += l;
      p_prev = p_curr;
    }
  }

  if (length_traversed == 0) {
    this->publish_keyframe_thread = std::thread( &dlo::OdomNode::publishKeyframe, this );
    this->publish_keyframe_thread.detach();
  }

  // Average computation time
  double avg_comp_time = std::accumulate(this->comp_times.begin(), this->comp_times.end(), 0.0) / this->comp_times.size();

  // RAM Usage
  double vm_usage = 0.0;
  double resident_set = 0.0;
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string num_threads, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
              >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
              >> utime >> stime >> cutime >> cstime >> priority >> nice
              >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
  stat_stream.close();
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  // CPU Usage
  struct tms timeSample;
  clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
      timeSample.tms_utime < this->lastUserCPU) {
      cpu_percent = -1.0;
  } else {
      cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
      cpu_percent /= (now - this->lastCPU);
      cpu_percent /= this->numProcessors;
      cpu_percent *= 100.;
  }
  this->lastCPU = now;
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;
  this->cpu_percents.push_back(cpu_percent);
  double avg_cpu_usage = std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();

  // Print to terminal
  printf("\033[2J\033[1;1H");

  std::cout << std::endl << "==== Direct LiDAR Odometry v" << this->version_ << " ====" << std::endl;

  if (!this->cpu_type.empty()) {
    std::cout << std::endl << this->cpu_type << " x " << this->numProcessors << std::endl;
  }

  std::cout << std::endl << std::setprecision(4) << std::fixed;
  std::cout << "Position    [xyz]  :: " << this->pose[0] << " " << this->pose[1] << " " << this->pose[2] << std::endl;
  std::cout << "Orientation [wxyz] :: " << this->rotq.w() << " " << this->rotq.x() << " " << this->rotq.y() << " " << this->rotq.z() << std::endl;
  std::cout << "Distance Traveled  :: " << length_traversed << " meters" << std::endl;
  std::cout << "Distance to Origin :: " << sqrt(pow(this->pose[0]-this->origin[0],2) + pow(this->pose[1]-this->origin[1],2) + pow(this->pose[2]-this->origin[2],2)) << " meters" << std::endl;

  std::cout << std::endl << std::right << std::setprecision(2) << std::fixed;
  std::cout << "Computation Time :: " << std::setfill(' ') << std::setw(6) << this->comp_times.back()*1000. << " ms    // Avg: " << std::setw(5) << avg_comp_time*1000. << std::endl;
  std::cout << "Cores Utilized   :: " << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: " << std::setw(5) << (avg_cpu_usage/100.) * this->numProcessors << std::endl;
  std::cout << "CPU Load         :: " << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: " << std::setw(5) << avg_cpu_usage << std::endl;
  std::cout << "RAM Allocation   :: " << std::setfill(' ') << std::setw(6) << resident_set/1000. << " MB    // VSZ: " << vm_usage/1000. << " MB" << std::endl;

}
