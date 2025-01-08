#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>   // for opencv4
//#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType {
    VELODYNE, OUSTER, LIVOX, HESAI, VELODYNE_M1600, MULRAN
};

class ParamServer {
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    string pcd_suffix;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    int gpsFrequence;
    bool useGPS;
    bool updateOrigin;
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;
    float GPSDISTANCE;

    // debug setting
    bool debugLidarTimestamp;
    bool debugImu;
    bool debugGps;


    // Save pcd
    bool savePCD;
    string savePCDDirectory;
    string sequence;
    string saveDirectory;
    string configDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int imuType;
    int imuFrequence;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;

    Eigen::Vector3d imuAccBias_N;
    Eigen::Vector3d imuGyrBias_N;
    Eigen::Vector3d imuGravity_N;
    vector<double> imuAccBias_NV;
    vector<double> imuGyrBias_NV;
    vector<double> imuGravity_NV;

    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;
    Eigen::Quaterniond q_sensor_body;
    Eigen::Vector3d t_sensor_body;
    Eigen::Quaterniond q_body_sensor;
    Eigen::Vector3d t_body_sensor;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool loopClosureEnableFlag;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    float globalMapLeafSize;

    ParamServer() {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lio_sam_6axis/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam_6axis/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam_6axis/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam_6axis/gpsTopic", gpsTopic, "fix");

        nh.param<std::string>("lio_sam_6axis/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lio_sam_6axis/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("lio_sam_6axis/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lio_sam_6axis/mapFrame", mapFrame, "map");


        nh.param<bool>("lio_sam_6axis/useGPS", useGPS, false);
        nh.param<bool>("lio_sam_6axis/updateOrigin", updateOrigin, false);
        nh.param<int>("lio_sam_6axis/gpsFrequence", gpsFrequence, 10);
        nh.param<bool>("lio_sam_6axis/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("lio_sam_6axis/useGpsElevation", useGpsElevation, false);
        nh.param<float>("lio_sam_6axis/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("lio_sam_6axis/poseCovThreshold", poseCovThreshold, 25.0);
        nh.param<float>("lio_sam_6axis/gpsDistance", GPSDISTANCE, 0.5);

        nh.param<bool>("lio_sam_6axis/debugLidarTimestamp", debugLidarTimestamp, false);
        nh.param<bool>("lio_sam_6axis/debugImu", debugImu, false);
        nh.param<bool>("lio_sam_6axis/debugGps", debugGps, false);

        nh.param<bool>("lio_sam_6axis/savePCD", savePCD, false);
        nh.param<std::string>("lio_sam_6axis/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");
        nh.param<std::string>("saveDirectory", saveDirectory, "/Downloads/LOAM/");
        nh.param<std::string>("sequence", sequence, "map");
        nh.param<std::string>("configDirectory", configDirectory, "map");

        std::cout << "SAVE DIR:" << saveDirectory << std::endl;

        std::string sensorStr;
        nh.param<std::string>("lio_sam_6axis/sensor", sensorStr, "ouster");
        if (sensorStr == "velodyne") {
            sensor = SensorType::VELODYNE;
        } else if (sensorStr == "ouster") {
            sensor = SensorType::OUSTER;
        } else if (sensorStr == "livox") {
            sensor = SensorType::LIVOX;
        } else if (sensorStr == "hesai") {
            sensor = SensorType::HESAI;
        } else if (sensorStr == "velodyne_m1600") {
            sensor = SensorType::VELODYNE_M1600;
        } else if (sensorStr == "mulran") {
            sensor = SensorType::MULRAN;
        } else {
            ROS_ERROR_STREAM(
                     "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'velodyne_m1600' ): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("lio_sam_6axis/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam_6axis/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("lio_sam_6axis/downsampleRate", downsampleRate, 1);
        nh.param<float>("lio_sam_6axis/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("lio_sam_6axis/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<int>("lio_sam_6axis/imuType", imuType, 0);
        nh.param<int>("lio_sam_6axis/imuFrequence", imuFrequence, 500);
        nh.param<float>("lio_sam_6axis/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("lio_sam_6axis/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("lio_sam_6axis/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("lio_sam_6axis/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("lio_sam_6axis/imuGravity", imuGravity, 9.80511);
        nh.param<float>("lio_sam_6axis/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double >>("lio_sam_6axis/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double >>("lio_sam_6axis/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double >>("lio_sam_6axis/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);
        //    std::cout << "qw:" << extQRPY << std::endl;

        if (sensor == SensorType::HESAI) {
            nh.param<vector<double >>("lio_sam_6axis/imuAccBias_N", imuAccBias_NV, vector<double>());
            nh.param<vector<double >>("lio_sam_6axis/imuGyrBias_N", imuGyrBias_NV, vector<double>());
            nh.param<vector<double >>("lio_sam_6axis/imuGravity_N", imuGravity_NV, vector<double>());
            imuAccBias_N = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(imuAccBias_NV.data(), 3, 1);
            imuGyrBias_N = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(imuGyrBias_NV.data(), 3, 1);
            imuGravity_N = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(imuGravity_NV.data(), 3, 1);
        }

        q_sensor_body = Eigen::Quaterniond(extRPY);
        t_sensor_body = extTrans;
        q_body_sensor = q_sensor_body.inverse();
        t_body_sensor = -(q_sensor_body.inverse() * t_sensor_body);

        nh.param<float>("lio_sam_6axis/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("lio_sam_6axis/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("lio_sam_6axis/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lio_sam_6axis/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("lio_sam_6axis/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("lio_sam_6axis/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lio_sam_6axis/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("lio_sam_6axis/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lio_sam_6axis/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("lio_sam_6axis/numberOfCores", numberOfCores, 2);
        nh.param<double>("lio_sam_6axis/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("lio_sam_6axis/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold,
                        1.0);
        nh.param<float>("lio_sam_6axis/surroundingkeyframeAddingAngleThreshold",
                        surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("lio_sam_6axis/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("lio_sam_6axis/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("lio_sam_6axis/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("lio_sam_6axis/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("lio_sam_6axis/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("lio_sam_6axis/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("lio_sam_6axis/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("lio_sam_6axis/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("lio_sam_6axis/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("lio_sam_6axis/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("lio_sam_6axis/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("lio_sam_6axis/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        nh.param<float>("lio_sam_6axis/globalMapLeafSize", globalMapLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
                                  imu_in.orientation.z);
        Eigen::Quaterniond q_final;
        if (imuType == 0) {
            q_final = extQRPY;
        } else if (imuType == 1)
            q_final = q_from * extQRPY;
        else
            std::cout << "pls set your imu_type, 0 for 6axis and 1 for 9axis" << std::endl;

        q_final.normalize();
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(
                q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() +
                q_final.w() * q_final.w())
            < 0.1) {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }

};


template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub,
                                      const T &thisCloud,
                                      ros::Time thisStamp,
                                      std::string thisFrame) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg) {
    return msg->header.stamp.toSec();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z) {
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

float pointDistance(PointType p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

#endif
