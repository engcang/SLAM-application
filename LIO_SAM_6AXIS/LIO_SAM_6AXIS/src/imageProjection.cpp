#include "utility.h"
#include "lio_sam_6axis/cloud_info.h"
#include "pcl/filters/impl/filter.hpp"

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)
struct Velodyne_M1600PointXYZIRT {
   PCL_ADD_POINT4D;
  uint8_t intensity;
  uint8_t ring;
  uint32_t timestampSec;
  uint32_t timestampNsec;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    Velodyne_M1600PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        uint8_t, ring, ring)(uint32_t, timestampSec, timestampSec)(uint32_t, timestampNsec, timestampNsec))


struct PandarPointXYZIRT {
    PCL_ADD_POINT4D

    float intensity;
    double timestamp;
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PandarPointXYZIRT,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (double, timestamp, timestamp)
                                          (uint16_t, ring, ring)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
//  uint32_t time;
    uint16_t reflectivity;
    uint8_t ring;
    std::uint16_t ambient;  // additional property of p.ouster
    float time;
    uint16_t noise;
    uint32_t range;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                          (uint16_t, reflectivity, reflectivity)
                                          (uint8_t, ring, ring)
                                          (std::uint16_t, ambient, ambient)
                                          (float, time, time)
                                          (uint16_t, noise, noise)
                                          (uint32_t, range, range)
)

struct MulranPointXYZIRT { // from the file player's topic https://github.com/irapkaist/file_player_mulran, see https://github.com/irapkaist/file_player_mulran/blob/17da0cb6ef66b4971ec943ab8d234aa25da33e7e/src/ROSThread.cpp#L7
     PCL_ADD_POINT4D;
     float intensity;
     uint32_t t;
     int ring;

     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 }EIGEN_ALIGN16;
 POINT_CLOUD_REGISTER_POINT_STRUCT (MulranPointXYZIRT,
     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
     (uint32_t, t, t) (int, ring, ring)
 )


//struct OusterPointXYZIRT {
//    PCL_ADD_POINT4D;
//    float intensity;
//    uint32_t t;
//    uint16_t reflectivity;
//    uint8_t ring;
//    uint16_t ambient;
//    uint32_t range;
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//} EIGEN_ALIGN16;
//
//POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
//                                  (float, x, x)
//                                          (float, y, y)
//                                          (float, z, z)
//                                          (float, intensity, intensity)
//                                          // use std::uint32_t to avoid conflicting with pcl::uint32_t
//                                          (std::uint32_t, t, t)
//                                          (std::uint16_t, reflectivity, reflectivity)
//                                          (std::uint8_t, ring, ring)
//                                          (std::uint16_t, ambient, ambient)
//                                          (std::uint32_t, range, range))
//

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer {
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubFullCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn;
    pcl::PointCloud<PandarPointXYZIRT>::Ptr tmpPandarCloudIn;
    pcl::PointCloud<Velodyne_M1600PointXYZIRT>::Ptr tmpM1600CloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam_6axis::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;

public:
    ImageProjection() :
            deskewFlag(0) {
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic,
                                                2000,
                                                &ImageProjection::imuHandler,
                                                this,
                                                ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental",
                                                   2000,
                                                   &ImageProjection::odometryHandler,
                                                   this,
                                                   ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic,
                                                               5,
                                                               &ImageProjection::cloudHandler,
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_6axis/deskew/cloud_deskewed", 1);
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_6axis/deskew/cloud_full_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam_6axis::cloud_info>("lio_sam_6axis/deskew/cloud_info", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory() {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        tmpPandarCloudIn.reset(new pcl::PointCloud<PandarPointXYZIRT>());
        tmpM1600CloudIn.reset(new pcl::PointCloud<Velodyne_M1600PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters() {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i) {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection() {}

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg) {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        if (debugImu) {
            // debug IMU data
            cout << std::setprecision(6);
            cout << "IMU acc: " << endl;
            cout << "x: " << thisImu.linear_acceleration.x <<
                 ", y: " << thisImu.linear_acceleration.y <<
                 ", z: " << thisImu.linear_acceleration.z << endl;
            cout << "IMU gyro: " << endl;
            cout << "x: " << thisImu.angular_velocity.x <<
                 ", y: " << thisImu.angular_velocity.y <<
                 ", z: " << thisImu.angular_velocity.z << endl;
            double imuRoll, imuPitch, imuYaw;
            tf::Quaternion orientation;
            tf::quaternionMsgToTF(thisImu.orientation, orientation);
            tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
            cout << "IMU roll pitch yaw: " << endl;
            cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
        }
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg) {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX) {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        } else if (sensor == SensorType::OUSTER) {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++) {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                //dst.time = src.t * 1e-9f;
//                dst.time = src.t;
                dst.time = src.time;
//                dst.time = (i % 2048) / 20480.0;
            }
        } else if (sensor == SensorType::MULRAN){
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++)
            {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = float(src.t);
            }
        } else if (sensor == SensorType::VELODYNE_M1600) {
               
            pcl::moveFromROSMsg(currentCloudMsg, *tmpM1600CloudIn);
            laserCloudIn->points.resize(tmpM1600CloudIn->size());
            laserCloudIn->is_dense = tmpM1600CloudIn->is_dense;
            double time_begins = tmpM1600CloudIn->points[0].timestampSec;
            double time_beginns = tmpM1600CloudIn->points[0].timestampNsec;
            double time_begin = time_begins + (time_beginns * 1e-9);
            for (size_t i = 0; i < tmpM1600CloudIn->size(); i++) {
                   auto &src = tmpM1600CloudIn->points[i];
                   auto &dst = laserCloudIn->points[i];
                   dst.x = src.x;
                   dst.y = src.y*-1;
                   dst.z = src.z*-1;
                   dst.intensity = static_cast<float>(src.intensity);
                   dst.ring = src.ring;
                   double point_time = src.timestampSec + (src.timestampNsec * 1e-9);
                   dst.time = point_time-time_begin;
               }

         }else if (sensor == SensorType::HESAI) {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpPandarCloudIn);
            laserCloudIn->points.resize(tmpPandarCloudIn->size());
            laserCloudIn->is_dense = tmpPandarCloudIn->is_dense;
            double time_begin = tmpPandarCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpPandarCloudIn->size(); i++) {
                auto &src = tmpPandarCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.y * -1;
                dst.y = src.x;
                //        dst.x = src.x;
                //        dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                //dst.tiSme = src.t * 1e-9f;
                dst.time = src.timestamp - time_begin; // s
            }
        } else {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        // timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        if (debugLidarTimestamp) {
            std::cout << std::fixed << std::setprecision(12) << "end time from pcd and size: "
                      << laserCloudIn->points.back().time
                      << ", " << laserCloudIn->points.size() << std::endl;
        }
	vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // check dense flag
        if (laserCloudIn->is_dense == false) {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0) {
            ringFlag = -1;
            for (int i = 0; i < (int) currentCloudMsg.fields.size(); ++i) {
                if (currentCloudMsg.fields[i].name == "ring") {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1) {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time
        if (deskewFlag == 0) {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields) {
                if (field.name == "time" || field.name == "t" || field.name == "timestamp" || field.name == "timestampSec") {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN(
                        "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo() {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur
            || imuQueue.back().header.stamp.toSec() < timeScanEnd) {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo() {

        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty()) {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int) imuQueue.size(); ++i) {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0) {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

//    std::cout << "diff:" << imuRotX[imuPointerCur - 1] << ", " << imuRotY[imuPointerCur - 1]
//              << ", " << imuRotZ[imuPointerCur - 1] << std::endl;

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;

    }

    void odomDeskewInfo() {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty()) {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int) odomQueue.size(); ++i) {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int) odomQueue.size(); ++i) {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x,
                                                            startOdomMsg.pose.pose.position.y,
                                                            startOdomMsg.pose.pose.position.z,
                                                            roll,
                                                            pitch,
                                                            yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x,
                                                          endOdomMsg.pose.pose.position.y,
                                                          endOdomMsg.pose.pose.position.z,
                                                          roll,
                                                          pitch,
                                                          yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur) {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur) {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront =
                    (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack =
                    (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur) {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime) {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true) {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur,
                                                        rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud() {
        int cloudSize = laserCloudIn->points.size();
//    std::cout << "point size raw: " << cloudSize << std::endl;
        // range image projection
        for (int i = 0; i < cloudSize; ++i) {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (sensor == SensorType::MULRAN)
            {
                rowIdn = (i % 64) + 1 ; // for MulRan dataset, Ouster OS1-64 .bin file,  giseop 
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER || sensor == SensorType::HESAI) {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0 / float(Horizon_SCAN);
                columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            } else if (sensor == SensorType::LIVOX) {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            } else if (sensor == SensorType::VELODYNE_M1600) {
                
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                
                float ang_res_x = 0.29; // or 0.33 for the extended model, as per the specifications
                
                columnIdn = round((horizonAngle / ang_res_x) + (Horizon_SCAN / 2));
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }


            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction() {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i) {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j) {
                if (rangeMat.at<float>(i, j) != FLT_MAX) {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;
        }
    }

    void publishClouds() {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);

        publishCloud(pubFullCloud, fullCloud, cloudHeader.stamp, lidarFrame);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lio_sam_6axis");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
