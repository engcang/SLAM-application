/**
 * This file is part of SLICT.
 *
 * Copyright (C) 2020 Thien-Minh Nguyen <thienminh.npn at ieee dot org>,
 * Division of RPL, KTH Royal Institute of Technology
 *
 * For more information please see <https://britsknguyen.github.io>.
 * or <https://github.com/brytsknguyen/slict>.
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * SLICT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SLICT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SLICT.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// Created by Thien-Minh Nguyen on 01/08/22.
//

#include "utility.h"

#include <livox_ros_driver/CustomMsg.h>

// const int queueLength = 2000;

using namespace std;
using namespace Eigen;
using namespace pcl;

class LivoxToOuster
{
private:
    // Node handler
    ros::NodeHandlePtr nh_ptr;

    ros::Subscriber livoxCloudSub;
    ros::Publisher ousterCloudPub;

    double intensityConvCoef = -1;

    int NUM_CORE;

    // bool remove_human_body = true;

public:
    // Destructor
    ~LivoxToOuster() {}

    LivoxToOuster(ros::NodeHandlePtr &nh_ptr_) : nh_ptr(nh_ptr_)
    {
        NUM_CORE = omp_get_max_threads();

        // Coefficient to convert the intensity from livox to ouster
        nh_ptr->param("intensityConvCoef", intensityConvCoef, 1.0);

        livoxCloudSub = nh_ptr->subscribe<livox_ros_driver::CustomMsg>("/livox/avia", 50, &LivoxToOuster::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        ousterCloudPub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/livox/lidar_ouster2", 50);
    }

    void cloudHandler(const livox_ros_driver::CustomMsg::ConstPtr &msgIn)
    {
        int cloudsize = msgIn->points.size();

        CloudOuster laserCloudOuster;
        laserCloudOuster.points.resize(cloudsize);
        laserCloudOuster.is_dense = true;

        #pragma omp parallel for num_threads(NUM_CORE)
        for (size_t i = 0; i < cloudsize; i++)
        {
            auto &src = msgIn->points[i];
            auto &dst = laserCloudOuster.points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.reflectivity * intensityConvCoef;
            dst.ring = src.line;
            dst.t = src.offset_time;
            dst.range = sqrt(src.x * src.x + src.y * src.y + src.z * src.z)*1000.0;
        }

        Util::publishCloud(ousterCloudPub, laserCloudOuster, msgIn->header.stamp, msgIn->header.frame_id);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_to_ouster");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO(KGRN "----> Livox to Ouster started" RESET);

    LivoxToOuster L2O(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}