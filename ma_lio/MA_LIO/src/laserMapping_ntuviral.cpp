#include <omp.h>
#include <mutex>
#include <iomanip>
#include <math.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ikd-Tree/ikd_Tree.h>
#include "associate_uct.hpp"
#include "parameters.h"
#include "preprocess.h"
#include "IMU_Processing.hpp"

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

struct ValueIndexPair
{
    double value;
    int index;
    bool operator<(const ValueIndexPair &other) const
    {
        return value < other.value;
    }
};

mutex mtx_buffer;
condition_variable sig_buffer;

float res_last[100000] = {0.0};
bool point_selected_surf[100000] = {0};
const float MOV_THRESHOLD = 1.5f;
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

std::vector<double> cov_plane;
std::vector<BoxPointType> cub_needrm;
std::vector<vector<int>> pointSearchInd_surf;
std::vector<PointVector> Nearest_Points;
std::vector<PointCloudXYZI::Ptr> feats_undistort_vec;
std::vector<PointCloudXYZI::Ptr> feats_down_vec;
std::vector<vector<Pose>> pose_unc;

std::vector<Eigen::Quaterniond> ext_q;
std::vector<V3D> ext_t;
std::vector<Pose> extrinsic;
std::vector<V3D> extrinsic_trans;
std::vector<Eigen::Quaterniond> extrinsic_quat;
std::vector<int> last_indices;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, lidar_end_time = 0, first_lidar_time = 0.0;
deque<double> time_start_buffer;
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
int effct_feat_num = 0;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);

MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
V3D prev_pos;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<ImuProcess> p_imu(new ImuProcess());
double total_distance = 0;
double opt_cnt = 0;
double curr_time = 0;
double total_computation = 0;

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void pointBodyToWorld(const Matrix<float, 3, 1> &pi, Matrix<float, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (extrinsic_quat[0] * p_body + extrinsic_trans[0]) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global;
    int lid_idx = pi->intensity;
    if (lid_idx == 0)
        p_global = state_point.rot * (extrinsic_quat[lid_idx] * p_body + extrinsic_trans[lid_idx]) + state_point.pos;
    else
        p_global = state_point.rot * (kf.temporal_comp[lid_idx - 1].q_ * (extrinsic_quat[lid_idx] * p_body + extrinsic_trans[lid_idx]) + kf.temporal_comp[lid_idx - 1].t_) + state_point.pos;
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global;
    int lid_idx = pi->intensity;
    if (lid_idx == 0)
        p_global = state_point.rot * (extrinsic_quat[lid_idx] * p_body + extrinsic_trans[lid_idx]) + state_point.pos;
    else
        p_global = state_point.rot * (kf.temporal_comp[lid_idx - 1].q_ * (extrinsic_quat[lid_idx] * p_body + extrinsic_trans[lid_idx]) + kf.temporal_comp[lid_idx - 1].t_) + state_point.pos;

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->normal_y;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
}

void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg, int num_lidar)
{
    mtx_buffer.lock();
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, num_lidar);
    lidar_buffer.push_back(ptr);
    time_start_buffer.push_back(msg->header.stamp.toSec());
    time_buffer.push_back(msg->header.stamp.toSec() + p_pre->maximum_time / double(1000));
    last_timestamp_lidar = msg->header.stamp.toSec();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg, int num_lidar)
{
    mtx_buffer.lock();
    last_timestamp_lidar = msg->header.stamp.toSec();
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, num_lidar);
    lidar_buffer.push_back(ptr);
    time_start_buffer.push_back(msg->header.stamp.toSec());
    time_buffer.push_back(last_timestamp_lidar + p_pre->maximum_time / double(1000));
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    double timestamp = msg->header.stamp.toSec();
    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// 3. Change the lidar_cbk_.
// Firstly, change the input with your lidar. In example case, there are two LiDARs. Therefore, it should be changed as
// (const your_msg1, const your_msg2)
// Secondly, change the inside of lidar_cbk_ function.
// In example case, there should be only two cbk_ function inside of the lidar_cbk_.
// Especially, if user wants to use only 1 lidar, 
// There is only one function inside of lidar_cbk_ function with two inputs for lidar_cbk_. 
void lidar_cbk_(const sensor_msgs::PointCloud2::ConstPtr &scanMsg_,
                const sensor_msgs::PointCloud2::ConstPtr &scanMsg2_)
{
    standard_pcl_cbk(scanMsg_, 0);
    standard_pcl_cbk(scanMsg2_, 1);
}

/*** For UrbanNav Dataset (Case: LiDAR 2)***/
/*void lidar_cbk_(const sensor_msgs::PointCloud2::ConstPtr &scanMsg_,
                const sensor_msgs::PointCloud2::ConstPtr &scanMsg2_)
{
    standard_pcl_cbk(scanMsg_, 0);
    standard_pcl_cbk(scanMsg2_, 1);
}*/
void extrinsic_update()
{
    extrinsic.clear();
    extrinsic_trans.clear();
    extrinsic_quat.clear();

    extrinsic.resize(lid_num);
    extrinsic_trans.resize(lid_num);
    extrinsic_quat.resize(lid_num);
    for (int num = 0; num < lid_num; num++)
    {
        V3D extrinsic_t = *(static_cast<MTK::vect<3, double> *>(state_point.vect_state_ptr[1 + num]));
        Eigen::Quaterniond extrinsic_q = *(static_cast<MTK::SO3<double> *>(state_point.SO3_state_ptr[1 + num]));
        extrinsic_trans[num] = extrinsic_t;
        extrinsic_quat[num] = extrinsic_q;
        PoseInitial(extrinsic[num], extrinsic_trans[num], extrinsic_quat[num], kf.getExtrinsicUncertainty(num));
    }
}

bool sync_packages(MeasureGroup &meas)
{
    /*** TBD : 0.2 is a hard-coded. It actually calculate using the LiDAR and IMU frequency ***/
    if (lidar_buffer.empty() || imu_buffer.empty() || imu_buffer.back()->header.stamp.toSec() - time_buffer.front() < 0.2)
        return false;

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        for (auto i = 0; i < lid_num; i++)
        {
            for (auto j = 0; j < lidar_buffer[i]->points.size(); j++)
                lidar_buffer[i]->points[j].normal_y = i;
        }

        std::vector<ValueIndexPair> pairs(lid_num);
        std::vector<int> indices(lid_num);

        for (auto i = 0; i < lid_num; i++)
            pairs[i] = {time_buffer[i], i};

        std::sort(pairs.begin(), pairs.end());
        meas.lidar_multi.clear();
        meas.lidar_beg_time.clear();
        meas.lidar_beg_time.resize(lid_num);
        meas.lidar_end_time.resize(lid_num);

        for (auto i = 0; i < lid_num; i++)
        {
            indices[i] = pairs[i].index;
            meas.lidar_beg_time[i] = time_start_buffer[indices[i]];
            meas.lidar_end_time[i] = time_buffer[indices[i]];
            meas.lidar_multi.push_back(lidar_buffer[indices[i]]);
        }
        for (auto i = 0; i < lid_num; i++)
        {
            if (indices[i] != last_indices[i])
            {
                kf.change_ext(indices, last_indices);
                break;
            }
        }
        for (auto i = 0; i < lid_num; i++)
            last_indices[i] = indices[i];

        lidar_end_time = meas.lidar_end_time[lid_num - 1];

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
        return false;

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    meas.imu_cont.clear();

    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    if (meas.imu.size() != 0)
        meas.imu_cont.push_back(meas.imu.back());
    /*** TBD : For stabilization, 10 IMU are more used ***/
    for (auto count = 1; count < 16; count++)
    {
        if (imu_buffer.size() < count)
            break;
        meas.imu_cont.push_back(imu_buffer[count-1]);
    }

    for (auto i = 0; i < lid_num; i++)
    {
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        time_start_buffer.pop_front();
    }

    lidar_pushed = false;
    return true;
}

void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        if ((feats_down_body->points[i].normal_y > cov_threshold))
            continue;
        
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));

        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
            PointToAdd.push_back(feats_down_world->points[i]);
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

void publish_frame_world(const ros::Publisher &pubLaserCloudFull)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],&laserCloudWorld->points[i]);
        
        pcl::PCDWriter pcd_writer;
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
    }

    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
            RGBpointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);

        *pcl_wait_save += *laserCloudWorld;
        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num > pcd_save_interval)
        {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time); 
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear();
    corr_normvect->clear();
    cov_plane.resize(feats_down_size);
    extrinsic_update();
    /*** Utilize the local planarity ***/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /*** transform to world frame ***/
        V3D p_body(point_body.x, point_body.y, point_body.z);
        int lid_idx = point_body.intensity;
        if (lid_idx != 0)
            p_body = extrinsic_quat[0].conjugate() * ((kf.temporal_comp[lid_idx - 1].q_ * (extrinsic_quat[lid_idx] * p_body + extrinsic_trans[lid_idx]) + kf.temporal_comp[lid_idx - 1].t_) - extrinsic_trans[0]);

        V3D p_global(s.rot * (extrinsic_quat[0] * p_body + extrinsic_trans[0]) + s.pos);

        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];
        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4) pabcd;
        double unit_cov;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, plane_th, unit_cov, cov_threshold))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.1)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                cov_plane[i] = unit_cov;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;
    double max_unit_cov = 0;
    double min_unit_cov = 1000;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            cov_plane[effct_feat_num] = cov_plane[i];
            if (cov_plane[effct_feat_num] > max_unit_cov)
                max_unit_cov = cov_plane[effct_feat_num];
            if (cov_plane[effct_feat_num] < min_unit_cov)
                min_unit_cov = cov_plane[effct_feat_num];

            effct_feat_num++;
        }
    }

    // return when no effective points
    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, (1 + lid_num) * 6);
    ekfom_data.h.resize(effct_feat_num);
    ekfom_data.R = MatrixXd::Zero(effct_feat_num, 1);

    double max_cov = 0;
    double min_cov = 9999;

    for (int i = 0; i < effct_feat_num; i++)
    {
        if (cov_plane[i] == 0)
            cov_plane[i] = 1;
        else if (max_unit_cov == min_unit_cov)
            cov_plane[i] = (plane_cov_max + plane_cov_min) / 2;
        else
            cov_plane[i] = 1 / ((plane_cov_max - plane_cov_min) * (cov_plane[i] - min_unit_cov) / (max_unit_cov - min_unit_cov) + plane_cov_min);

        M3D cov;
        PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        int lid_idx = laser_p.intensity;
        if (lid_idx != 0)
            point_this_be = extrinsic_quat[0].conjugate() * ((kf.temporal_comp[lid_idx - 1].q_ * (extrinsic_quat[lid_idx] * point_this_be + extrinsic_trans[lid_idx]) + kf.temporal_comp[lid_idx - 1].t_) - extrinsic_trans[0]);

        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = extrinsic_quat[0] * point_this_be + extrinsic_trans[0];
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        V3D B;
        ekfom_data.h_x.block<1, 6>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A);

        if (extrinsic_est_en)
        {
            if (lid_idx == 0)
                B = point_be_crossmat * extrinsic_quat[0].conjugate() * C; 
            else
            {
                V3D point_ori(laser_p.x, laser_p.y, laser_p.z);
                point_be_crossmat << SKEW_SYM_MATRX(point_ori);
                C = kf.temporal_comp[lid_idx - 1].q_.conjugate() * C;
                B = point_be_crossmat * extrinsic_quat[lid_idx].conjugate() * C;
            }
            ekfom_data.h_x.block<1, 3>(i, 6 + 3 * lid_idx) << VEC_FROM_ARRAY(B);
            ekfom_data.h_x.block<1, 3>(i, 6 + 3 * (lid_num + lid_idx)) << VEC_FROM_ARRAY(C);
            int uncertain = int(laser_p.normal_x);
            if (uncertain >= pose_unc[lid_idx].size())
                uncertain = pose_unc[lid_idx].size() - 2;
            evalPointUncertainty(laser_p, cov, pose_unc[lid_idx][uncertain]);
            ekfom_data.R(i, 0) = cov(0, 0) + cov(1, 1) + cov(2, 2);
            laser_p.normal_y = cov(0, 0) + cov(1, 1) + cov(2, 2);
            if (max_cov < ekfom_data.R(i, 0))
                max_cov = ekfom_data.R(i, 0);
            if (min_cov > ekfom_data.R(i, 0))
                min_cov = ekfom_data.R(i, 0);
        }

        /*** Measuremnt residual value ***/
        ekfom_data.h(i) = (-1) * norm_p.intensity;
    }

    /*** Uncertainty Calculation using FIC ***/
    double cov_diff = max_cov - min_cov;
    for (int i = 0; i < effct_feat_num; i++)
    {
        ekfom_data.h_x.block<1, Eigen::Dynamic>(i, 0, 1, 6 * (lid_num + 1)) = ekfom_data.h_x.block<1, Eigen::Dynamic>(i, 0, 1, 6 * (lid_num + 1)) * cov_plane[i];
        ekfom_data.h(i) = ekfom_data.h(i) * cov_plane[i];
        if (ekfom_data.R(i, 0) < min_cov + (max_cov - min_cov) * range_min)
            ekfom_data.R(i, 0) = point_cov_min;
        else if (ekfom_data.R(i, 0) > min_cov + (max_cov - min_cov) * range_max)
            ekfom_data.R(i, 0) = point_cov_max;
        else
            ekfom_data.R(i, 0) = (point_cov_max - point_cov_min) * (ekfom_data.R(i, 0) - (min_cov + (max_cov - min_cov) * range_min)) / ((range_max - range_min) * (max_cov - min_cov)) + point_cov_min;
    }

    /*** Uncertainty given to non-planar point ***/
    int k = 0;
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            feats_down_body->points[i].normal_y = laserCloudOri->points[k].normal_y;
            k++;
        }
        else
        {
            M3D cov;
            float which_lidar = feats_down_body->points[i].intensity;
            int imu_idx = int(feats_down_body->points[i].normal_x);
            if (imu_idx >= pose_unc[which_lidar].size() - 1)
                imu_idx = pose_unc[which_lidar].size() - 2;
            evalPointUncertainty(feats_down_body->points[i], cov, pose_unc[which_lidar][imu_idx]);
            feats_down_body->points[i].normal_y = cov(0, 0) + cov(1, 1) + cov(2, 2);
        }
    }
    /*** Localization weight calculation ***/
    Eigen::MatrixXd svd_mat = ekfom_data.h_x.topLeftCorner(effct_feat_num, 3);
    Eigen::JacobiSVD<MatrixXd> svd(svd_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    MatrixXd singular_value = svd.singularValues();

    double weight = singular_value(2, 0) / singular_value(0, 0);

    if (weight > localize_thresh_max)
        weight = localize_cov_max;
    else if (weight < localize_thresh_min)
        weight = localize_cov_min;
    else
        weight = (localize_cov_max - localize_cov_min) * (weight - localize_thresh_min) / (localize_thresh_max - localize_thresh_min) + localize_cov_min;

    ekfom_data.h_x = ekfom_data.h_x * weight;
    ekfom_data.h = ekfom_data.h * weight;
}

void visualize_state()
{
    std::ifstream stat_stream("/proc/self/statm", std::ios_base::in);
    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
    unsigned long vm = 0, rss = 0;
    stat_stream >> vm >> rss; 
    stat_stream.close();
    double vm_usage     = vm * page_size_kb / 1024.0;
    double resident_set = rss * page_size_kb / 1024.0;
    total_computation += (omp_get_wtime() - curr_time);
    opt_cnt += 1;
    std::system("clear");
    
    std::cout << "\033[1;36m" << "**** Multiple Asynchronous LiDAR Inertial Odometry (MA-LIO) ****" << "\033[0m" << std::endl;
    std::cout << std::endl;
    std::cout.precision(20);
    std::cout << "\033[1;33m" << "[Timestamp of Current State]: " << "\033[0m" << "\033[1;32m" << Measures.lidar_end_time[2] << " secs" "\033[0m" << std::endl;
    std::cout.precision(5);
    std::cout << "\033[1;33m" << "[Position]: " << "\033[0m" << "\033[1;32m" << "\n\t[x]: " << state_point.pos(0) << " meter"
              << "\n\t[y]: " << state_point.pos(1) << " meter"
              << "\n\t[z]: " << state_point.pos(2) << " meter" << "\033[0m" << std::endl;

    std::cout << "\033[1;33m" << "[Orientation]: " << "\033[0m" << "\033[1;32m" 
              << "\n\t[qx]: " << geoQuat.x 
              << "\n\t[qy]: " << geoQuat.y 
              << "\n\t[qz]: " << geoQuat.z 
              << "\n\t[qw]: " << geoQuat.w << "\033[0m" << std::endl;
    
    std::cout << "\033[1;33m" << "[Velocity]: " << "\033[0m" << "\033[1;32m" 
              << "\n\t[x]: " << state_point.vel(0) << " m/s"
              << "\n\t[y]: " << state_point.vel(1) << " m/s"
              << "\n\t[z]: " << state_point.vel(2) << " m/s" << "\033[0m" << std::endl;

    for (int num = 0; num < lid_num; num++)
    {
        V3D extrinsic_t = *(static_cast<MTK::vect<3, double> *>(state_point.vect_state_ptr[1 + num]));
        Eigen::Quaterniond extrinsic_q = *(static_cast<MTK::SO3<double> *>(state_point.SO3_state_ptr[1 + num]));
        std::cout.precision(20);
        std::cout << "\033[1;33m" << "[LiDAR-IMU Extrinsic " << num << "]: " << "\033[0m"
                  << "\033[1;32m" << "\n\tTimestamp for the Lastest Point: " 
                  << Measures.lidar_end_time[lid_num - 1 - num] << " secs";
        std::cout.precision(5);          
        std::cout << "\033[1;32m" << "\n\tTranslation: " 
                  << "\n\t\t[x]: " << extrinsic_t(0) << " meter"
                  << "\n\t\t[y]: " << extrinsic_t(1) << " meter"
                  << "\n\t\t[z]: " << extrinsic_t(2) << " meter"
                  << "\n\tRotation: " 
                  << "\n\t\t[qx]: " << extrinsic_q.x() 
                  << "\n\t\t[qy]: " << extrinsic_q.y() 
                  << "\n\t\t[qz]: " << extrinsic_q.z() 
                  << "\n\t\t[qw]: " << extrinsic_q.w() << "\033[0m" << std::endl;
    }

    for(int num = 0; num < lid_num - 1; num++)
    {
        std::cout << "\033[1;33m" << "[Temporal discrepancy between LiDAR " << num << " and " << num+1 << "]: " << "\033[0m" 
                << "\033[1;32m" << (Measures.lidar_end_time[lid_num - 1 - num] - Measures.lidar_end_time[lid_num - 1 - (num + 1)]) * 1000 << " msecs" << "\033[0m" << std::endl;
    }
    
    total_distance += sqrt((state_point.pos(0) - prev_pos(0)) * (state_point.pos(0) - prev_pos(0)) + (state_point.pos(1) - prev_pos(1)) * (state_point.pos(1) - prev_pos(1)) + (state_point.pos(2) - prev_pos(2)) * (state_point.pos(2) - prev_pos(2)));
    std::cout << "\033[1;33m" << "[Total Distance]: " << "\033[0m" << "\033[1;32m" << total_distance << " meter" << "\033[0m"<< std::endl;
    std::cout << "\033[1;33m" << "[Duration]: " << "\033[0m" << "\033[1;32m" << Measures.lidar_end_time[lid_num-1] - first_lidar_time << " secs" << "\033[0m" << std::endl;
    std::cout << "\033[1;33m" << "[Map Point Count (Downsample w/ "<<  filter_size_map_min << ")]: " << "\033[0m" << "\033[1;32m" << ikdtree.size() << "\033[0m"<< std::endl;
    std::cout << "\033[1;33m" << "[Computation Time]: " << "\033[0m" << "\033[1;32m" << (omp_get_wtime() - curr_time) * 1000 << " msecs" << "\033[0m" << std::endl;
    std::cout << "\033[1;33m" << "[Computation Time (Avg)]: " << "\033[0m" << "\033[1;32m" << total_computation / opt_cnt * 1000 << " msecs" << "\033[0m" << std::endl;    
    std::cout << "\033[1;33m" << "[RAM Allocation]: " << "\033[0m" << "\033[1;32m" << resident_set << " MB" << "\033[0m" << std::endl;
    std::cout << "\033[1;33m" << "[VSZ]: " << "\033[0m" << "\033[1;32m" << vm_usage << " MB" << "\033[0m" << "\n" << std::endl;
}   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    readParameters(nh);

    last_indices.resize(lid_num);
    for (int num = 0; num < lid_num; num++)
    {
        V3D extrinsic_Tn(extrinT.at(num * 3), extrinT.at(num * 3 + 1), extrinT.at(num * 3 + 2));
        Eigen::Quaterniond extrinsic_Rn(extrinR.at(num * 4), extrinR.at(num * 4 + 1), extrinR.at(num * 4 + 2), extrinR.at(num * 4 + 3));
        ext_t.push_back(extrinsic_Tn);
        ext_q.push_back(extrinsic_Rn);

        p_pre->point_filter_num.push_back(point_filter_num.at(num));
        p_pre->N_SCANS.push_back(N_SCANS.at(num));
        p_pre->lid_type.push_back(lid_type.at(num));

        last_indices[num] = lid_num - num - 1;
    }
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS);
    kf.extrinsicInit(ext_t, ext_q);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    /*** trajectory record ***/
    ofstream fout_out;
    fout_out.open(DEBUG_FILE_DIR("trajectory.txt"), ios::out);

    if (fout_out)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    typedef sensor_msgs::PointCloud2 LidarMsgType;
    typedef livox_ros_driver::CustomMsg LivoxMsgType;
    typedef message_filters::Subscriber<LidarMsgType> LidarSubType;
    typedef message_filters::Subscriber<LivoxMsgType> LivoxSubType;

    for (int num = 0; num < lid_num; num++)
    {
        if (lid_type.at(num) == 1)
            livox_num++;
        else
            spin_num++;
    }

    std::vector<LidarSubType *> sub_spin(spin_num);
    std::vector<LivoxSubType *> sub_livox(livox_num);

    for (int num = 0; num < lid_num - livox_num; num++)
        sub_spin[num] = new LidarSubType(nh, lid_topic.at(num), 2000);
    for (int num = 0; num < livox_num; num++)
        sub_livox[num] = new LivoxSubType(nh, lid_topic.at(num + spin_num), 2000);

    // 1. Change the sync_policies using the SubType.
    // For example, if users want to use two LiDARs, rewrite below as
    // typedef message_filters::sync_policies::ApproximateTime<Your Msg1, Your Msg2> LidarSyncPolicy;
    // Especially, if you want to use only 1 lidar, define the ApproximateTime as
    // typedef message_filters::sync_policies::ApproximateTime<Your Msg1, Your Msg1> LidarSyncPolicy
    typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType> LidarSyncPolicy;
    typedef message_filters::Synchronizer<LidarSyncPolicy> Sync;

    // 2. Change the synchronizer based on sync_policies
    // LidarSyncPolicy(10), *Your LiDAR[0], *Your LiDAR[1];
    // Don't forget to change the registerCallback as (lidar_cbk_, _1, _2).
    // Especially, if you want to use only 1 lidar, Synchronizer should be written as
    // LidarSyncPolicy(10), *Your LiDAR[0], *Your LiDAR[0];
    message_filters::Synchronizer<LidarSyncPolicy> *sync =
        new message_filters::Synchronizer<LidarSyncPolicy>(
            LidarSyncPolicy(10), *sub_spin[0], *sub_spin[1]);
    sync->registerCallback(boost::bind(&lidar_cbk_, _1, _2));

    /*** For UrbanNav Dataset (Case: LiDAR 2)***/
    /*typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType> LidarSyncPolicy;
    typedef message_filters::Synchronizer<LidarSyncPolicy> Sync;

    message_filters::Synchronizer<LidarSyncPolicy> *sync =
        new message_filters::Synchronizer<LidarSyncPolicy>(
            LidarSyncPolicy(10), *sub_spin[0], *sub_spin[1]);
    sync->registerCallback(boost::bind(&lidar_cbk_, _1, _2));*/

    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
    //------------------------------------------------------------------------------------------------------//
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit)
            break;
        ros::spinOnce();

        if (sync_packages(Measures))
        {   /*** Initialize ***/
            curr_time = omp_get_wtime();
            state_point = kf.get_x();
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time[0];
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }
            extrinsic_update();
            feats_undistort->clear();
            feats_down_body->clear();
            feats_undistort_vec.resize(lid_num);
            feats_down_vec.resize(lid_num);

            for (int num = 0; num < lid_num; num++)
            {
                PointCloudXYZI::Ptr feats_distort(new PointCloudXYZI());
                PointCloudXYZI::Ptr feats_down(new PointCloudXYZI());
                feats_undistort_vec[num] = feats_distort;
                feats_down_vec[num] = feats_down;
            }
            /*** Undistortion ***/
            p_imu->Process(Measures, kf, feats_undistort_vec);
            /*** Downsampling ***/
            for (int num = 0; num < lid_num; num++)
            {
                downSizeFilterSurf.setInputCloud(feats_undistort_vec[num]);
                downSizeFilterSurf.filter(*feats_down_vec[num]);
                for (auto i = 0; i < feats_down_vec[num]->points.size(); i++)
                {
                    feats_down_vec[num]->points[i].normal_x = feats_down_vec[num]->points[i].intensity;
                    feats_down_vec[num]->points[i].intensity = num;
                }
                for (auto i = 0; i < feats_undistort_vec[num]->points.size(); i++)
                {
                    feats_undistort_vec[num]->points[i].intensity = num;
                }
                *feats_undistort = *feats_undistort + *feats_undistort_vec[num];
                *feats_down_body = *feats_down_body + *feats_down_vec[num];
            }

            feats_down_size = feats_down_body->points.size();
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * extrinsic_trans[0];

            flg_EKF_inited = (Measures.lidar_beg_time[0] - first_lidar_time) < INIT_TIME ? false : true;
            
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** initialize the tree ***/
            if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                        feats_down_world->points[i].normal_y = 0.001;
                    }

                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            V3D ext_euler = SO3ToEuler(extrinsic_quat[0]);

            if (0) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);

            /*** Calculate each LiDAR state uncertainty ***/
            pose_unc.clear();
            pose_unc.resize(lid_num);
            Pose pose_point;
            for (int num = 0; num < lid_num; num++)
            {
                if (num == 0)
                {
                    for (int i = 0; i < (int)kf.lidar_uncertainty[num].size(); i++)
                        pose_unc[num].push_back(kf.lidar_uncertainty[num][i]);
                }
                else
                {
                    for (int i = 0; i < (int)kf.lidar_uncertainty[num].size(); i++)
                    {
                        compoundPoseWithCov(extrinsic[num], extrinsic[num].cov_, kf.lidar_uncertainty[num][i], kf.lidar_uncertainty[num][i].cov_, pose_point, pose_point.cov_, 2);
                        compoundPoseWithCov(kf.temporal_comp[num - 1], kf.temporal_comp[num - 1].cov_, pose_point, pose_point.cov_, pose_point, pose_point.cov_, 2);
                        compoundInvPoseWithCov(extrinsic[0], extrinsic[0].cov_, pose_point, pose_point.cov_, pose_point, pose_point.cov_, 2);
                        pose_unc[num].push_back(pose_point);
                    }
                }
            }

            double solve_H_time = 0;
            /*** iterated state estimation ***/
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            extrinsic_update();
            state_point = kf.get_x();

            pos_lid = state_point.pos + state_point.rot * extrinsic_trans[0];
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            map_incremental();

            visualize_state();

            prev_pos(0) = state_point.pos(0); prev_pos(1) = state_point.pos(1); prev_pos(2) = state_point.pos(2);
            fout_out.precision(20);
            fout_out << Measures.lidar_end_time[2] << " " << state_point.pos(0) << " " << state_point.pos(1) << " " << state_point.pos(2) << " " << geoQuat.x << " " << geoQuat.y << " " << geoQuat.z << " " << geoQuat.w << std::endl; 
            
            /******* Publish points *******/
            if (path_en)
                publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull);
        }

        status = ros::ok();
        rate.sleep();
    }
    /**************** save map ****************/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("result.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();

    return 0;
}
