#include <mutex>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <condition_variable>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

#include "preprocess.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

/*** Time Log Variables ***/
bool   runtime_pos_log = false;
/**************************/

string root_dir = ROOT_DIR;
string map_file_path, lid_topic;

mutex mtx_buffer;
condition_variable sig_buffer;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
int    scan_count = 0;

deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;

shared_ptr<Preprocess> p_pre(new Preprocess());

// parameters for normal estimation
int N_SCAN, Horizon_SCAN;
string ring_table_dir;
bool check_normal;

ros::Publisher pub_cloud;
ros::Publisher pub_lines;

double len = 1.0;

void visualizeNormals(const PointCloudXYZI & cloud, const std_msgs::Header & header)
{

    if (pub_lines.getNumSubscribers() != 0) {
        visualization_msgs::Marker lines;
        lines.header = header;
        lines.header.frame_id = "camera_init";
        lines.ns = "lines";
        lines.type = visualization_msgs::Marker::LINE_LIST;
        lines.action = visualization_msgs::Marker::ADD;
        lines.pose.orientation.w = 1.0;
        lines.lifetime = ros::Duration();

        //static int key_poses_id = 0;
        lines.id = 0; //key_poses_id++;
        lines.scale.x = 0.03;
        lines.scale.y = 0.03;
        lines.scale.z = 0.03;
        lines.color.g = 1.0;
        lines.color.a = 1.0;

        for (int i = 0; i < p_pre->pl_surf.points.size(); ++i) {
            geometry_msgs::Point p;
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            lines.points.push_back(p);

            p.x += cloud.points[i].normal_x * len;
            p.y += cloud.points[i].normal_y * len;
            p.z += cloud.points[i].normal_z * len;
            lines.points.push_back(p);
        }
        pub_lines.publish(lines);
    }
}

void visualizeCloud(const PointCloudXYZI & cloud, const std_msgs::Header & header)
{
    if (pub_cloud.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(p_pre->pl_surf, laserCloudmsg);
        laserCloudmsg.header = header;
        laserCloudmsg.header.frame_id = "camera_init";
        pub_cloud.publish(laserCloudmsg);
    }
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    scan_count ++;
//    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    visualizeCloud(p_pre->pl_surf, msg->header);

    visualizeNormals(p_pre->pl_surf, msg->header);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Ring FALS");
    ros::NodeHandle nh;

    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/velodyne_points");
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/scan_line", N_SCAN, 16);
    nh.param<int>("preprocess/Horizon_SCAN", Horizon_SCAN, 1800);
    nh.param<int>("preprocess/Horizon_SCAN", p_pre->Horizon_SCAN, 1800);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);

    // ring Fals Normal Estimation parameters
    nh.param<bool>("normal/compute_table", p_pre->compute_table, false);
    nh.param<bool>("normal/ringfals_en", p_pre->ringfals_en, false);
    nh.param<bool>("normal/check_normal", check_normal, true);
    nh.param<string>("normal/ring_table_dir", ring_table_dir, "/tmp");
    std::string PROJECT_NAME = "ring_fals";
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);
    ring_table_dir = pkg_path + ring_table_dir;
    p_pre->ring_table_dir = ring_table_dir;
    p_pre->runtime_log = runtime_pos_log;
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    p_pre->initNormalEstimator();


    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 1000, standard_pcl_cbk);

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_normal", 1000);

    pub_lines = nh.advertise<visualization_msgs::Marker> ("normals", 10);
//------------------------------------------------------------------------------------------------------
    ros::Rate rate(5000);
    bool status = ros::ok();
    ros::spin();

    if (p_pre->compute_table) {
        printf(".....Computing M inverse....\n");
        p_pre->range_image.computeMInverse();
        printf("Computing M inverse matrix.\n");
        printf(".....Saving range image lookup table....\n");
        p_pre->range_image.saveLookupTable(p_pre->ring_table_dir, "ring" + std::to_string(N_SCAN));
    }

    return 0;
}
