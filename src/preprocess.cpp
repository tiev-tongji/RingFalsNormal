#include "preprocess.h"
#include "point_type.h"
#include "tic_toc.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS   = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B?
  p2l_ratio = 225;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}


void Preprocess::saveNormalPCD(const std::string& file_name, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, const cv::Mat& normals_mat)
{
    ROS_ASSERT(rangeMat.size == normals_mat.size);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZINormal>);
    for (int i = 0; i < normals_mat.rows; ++i) {
        for (int j = 0; j < normals_mat.cols; ++j) {
            if (rangeMat.at<float>(i, j) == FLT_MAX) //
                continue;

////            int index = j  + i * normals_mat.cols; //点在深度图像中的索引
            const int index = image2cloud[j + i * image_cols];
            const PointXYZIRT& thisPoint = cloud->points[index];
//            int index = j  + i * normals_mat.cols; //
//            const PointType& thisPoint = fullCloud->points[index];

            pcl::PointXYZINormal pn;
            pn.x = thisPoint.x;
            pn.y = thisPoint.y;
            pn.z = thisPoint.z;

            const cv::Vec3f & n_cv = normals_mat.at<cv::Vec3f>(i, j);
            pn.normal_x = n_cv(0);
            pn.normal_y = n_cv(1);
            pn.normal_z = n_cv(2);

            cloud_normal->push_back(pn);
        }
    }
    if (cloud_normal->size() > 0)
        pcl::io::savePCDFile(file_name, *cloud_normal);
}

void Preprocess::projectPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
//    if (!compute_table)
        rangeMat = cv::Mat(N_SCANS, image_cols, CV_32F, cv::Scalar::all(FLT_MAX));
    image2cloud.clear();
    image2cloud.resize(N_SCANS * image_cols, -1);

    TicToc t_range_image;
    int cloudSize = (int)cloud->points.size();
    cloud2image.clear();
    cloud2image.resize(cloudSize, -1);

    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        const PointXYZIRT& thisPoint = cloud->points[i];

        int rowIdn, columnIdn;
        if (!pointInImage(thisPoint, rowIdn, columnIdn))
            continue;

        float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);//  sqrt(x^2 + y^2 + z^2)

//        if (range < 1.0)
//            continue;

//        if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
//            continue;

        rangeMat.at<float>(rowIdn, columnIdn) = range; //record range after correcting

        int index = columnIdn  + rowIdn * image_cols; //index
        image2cloud[index] = i;
        cloud2image[i] = index;
    }
}

void Preprocess::computeRingNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    // not test yet
    if (compute_table) {
        TicToc t_range_image;
//        int cloudSize = (int)cloud->points.size();
//        pcl::PointCloud<PointXYZIRT>::Ptr cloud_tmp(new  pcl::PointCloud<PointXYZIRT>());
//        cloud_tmp->resize(cloudSize);
//        // range image projection
//        for (int i = 0; i < cloudSize; ++i) {
//            PointXYZIRT &thisPoint = cloud_tmp->points[i];
//            thisPoint.x = cloud->points[i].x;
//            thisPoint.y = cloud->points[i].y;
//            thisPoint.z = cloud->points[i].z;
//            thisPoint.intensity = cloud->points[i].intensity;
//            thisPoint.ring = static_cast<uint16_t>(cloud->points[i].normal_x);  // ring，
//        }
//        range_image.createFromRings(N_SCANS, image_cols, laserCloudIn);
//        range_image.createAndBuildTableFromRings(laserCloudIn);
        range_image.buildTableFromRings(cloud);
        ROS_WARN("build range image from rings cost: %fms", t_range_image.toc());
        return;
    }

//        range_image.createFromRings(laserCloudIn);
//        range_image.saveRangeImage("/tmp/range_image.jpg");

    TicToc t_normal;
//        range_image.computeNormals(rangeMat, normals);
    range_image.computeNormals(rangeMat, normals, plane_residual); ///output: normalized normals
//        ROS_WARN("normals %d x %d", (int)normals.rows, (int)normals.image_cols);
//    ROS_WARN("LiDAR FALS normal computation cost: %fms", t_normal.toc());
}



void Preprocess::flipNormalsTowardCenterAndNormalized(const cv::Vec3f& center, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, cv::Mat& image_normals)
{
    ROS_ASSERT(rangeMat.size == image_normals.size);
    for (int i = 0; i < N_SCANS; ++i)
        for (int j = 0; j < image_cols; ++j)
            if (rangeMat.at<float>(i, j) != FLT_MAX)
            {
                const int point_id = image2cloud[j + i * image_cols];
                const PointXYZIRT &thisPoint = cloud->points[point_id];
//                const PointType &thisPoint = fullCloud->points[j + i * image_cols];

                // vector: from center to point
                cv::Vec3f vc2p(thisPoint.x - center(0), thisPoint.y - center(1), thisPoint.z - center(2));
                vc2p /= norm(vc2p);

                cv::Vec3f &n = image_normals.at<cv::Vec3f>(i, j); ///already normalized
                if (vc2p.dot(n) > 0)
                    n *= -1;
            }
}

void Preprocess::NormalizeNormals(cv::Mat& image_normals)
{
    ROS_ASSERT(rangeMat.size == image_normals.size);
    for (int i = 0; i < N_SCANS; ++i)//
        for (int j = 0; j < image_cols; ++j)
            if (rangeMat.at<float>(i, j) != FLT_MAX)//
            {
                cv::Vec3f &n = image_normals.at<cv::Vec3f>(i, j);
                n /= norm(n);
            }
}

void Preprocess::extractCloudAndNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    TicToc t_flip;
    cv::Vec3f lidar(0, 0, 0);
    flipNormalsTowardCenterAndNormalized(lidar, cloud, normals);
//    ROS_WARN("flip normals cost: %fms", t_flip.toc());
//    saveNormalPCD("/tmp/normal_flipped.pcd", cloud, normals);

    // smooth normals after flipping
    TicToc t_smo;
//    cv::Mat normal_smoothed;
//        cv::GaussianBlur(normals, normal_smoothed, cv::Size(3, 3), 0);
    cv::medianBlur(normals, normals, 5);
//    saveNormalPCD("/tmp/normal_median_blur.pcd", cloud, normals);
//        TicToc t_nor;
    NormalizeNormals(normals);
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
  case OUSTER:
      ouster_handler(msg);
    break;

  case VELODYNE:
    velodyne_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }

//  ROS_WARN("cloud_with_normal size: %d", (int)cloud_with_normal->size());
//    if (pl_surf.size() > 0)
//        pcl::io::savePCDFile("/tmp/pl_surf.pcd", pl_surf);
//  pcl_out = cloud_with_normal;
    *pcl_out = pl_surf;

}


void Preprocess::estimateNormals(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    TicToc t_1;
    projectPointCloud(cloud);
    proj_time = t_1.toc();
    TicToc t_2;
    computeRingNormals(cloud);
    compu_time = t_2.toc();
    TicToc t_3;
    if (!compute_table)
    {
        extractCloudAndNormals(cloud);
    }
    smooth_time = t_3.toc();
}

void Preprocess::ouster2velodyne(const pcl::PointCloud<ouster_ros::Point>& cloud_in, const pcl::PointCloud<PointXYZIRT>::Ptr& cloud_out)
{
    int cloud_size = cloud_in.size();
    cloud_out->resize(cloud_size);
    for (int i = 0; i < cloud_size; ++i) {
        const ouster_ros::Point& p_in = cloud_in[i];
        PointXYZIRT& p_out = cloud_out->points[i];
        p_out.getVector4fMap() = p_in.getVector4fMap();
        p_out.ring = p_in.ring;
    }
}

void Preprocess::ouster_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  pcl::PointCloud<PointXYZIRT>::Ptr cloud_tmp(new pcl::PointCloud<PointXYZIRT>());
  ouster2velodyne(pl_orig, cloud_tmp);
    if (compute_normal)
    {
        TicToc t_nor;
        estimateNormals(cloud_tmp); // get cloud_with_normal
        if (runtime_log)
        {
            num_scans++;
            aver_normal_time = aver_normal_time * (num_scans - 1) / num_scans + t_nor.toc() / num_scans;
            aver_proj_time = aver_proj_time * (num_scans - 1) / num_scans + proj_time / num_scans;
            aver_compu_time = aver_compu_time * (num_scans - 1) / num_scans + compu_time / num_scans;
            aver_smooth_time = aver_smooth_time * (num_scans - 1) / num_scans + smooth_time / num_scans;
            ROS_INFO("[Ring FALS] mean project %0.3fms, compute %0.3fms, smooth %0.3fms, total %0.3fms",
                     aver_proj_time, aver_compu_time, aver_smooth_time, aver_normal_time);
        }
    }

  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);



    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

        const int & index =  cloud2image[i];
        if (compute_normal && index > 0)
        {
            int rowIdn, columnIdn;
            rowIdn = index / image_cols;
            columnIdn = index - rowIdn * image_cols;
            const cv::Vec3f &n_cv = normals.at<cv::Vec3f>(rowIdn, columnIdn);
            if (!std::isfinite(n_cv(0)) || !std::isfinite(n_cv(1)) || !std::isfinite(n_cv(2)))
                continue;
            added_pt.normal_x = n_cv(0);
            added_pt.normal_y = n_cv(1);
            added_pt.normal_z = n_cv(2);
        }

      pl_surf.points.push_back(added_pt);
    }

//    pl_surf.resize(pl_surf.points.size());
//    if (pl_surf.size() > 0)
//        pcl::io::savePCDFile("/tmp/pl_surf.pcd", pl_surf);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<PointXYZIRT>::Ptr pl_orig(new pcl::PointCloud<PointXYZIRT>());
//    pcl::PointCloud<PointXYZIRT>::Ptr pl_orig;
    pcl::fromROSMsg(*msg, *pl_orig);
    int plsize = pl_orig->points.size();
//    ROS_INFO("cloud input size: %d", plsize);

    if (compute_normal)
    {
        TicToc t_nor;
        estimateNormals(pl_orig); // get cloud_with_normal
        if (runtime_log)
        {
            num_scans++;
            aver_normal_time = aver_normal_time * (num_scans - 1) / num_scans + t_nor.toc() / num_scans;
            aver_proj_time = aver_proj_time * (num_scans - 1) / num_scans + proj_time / num_scans;
            aver_compu_time = aver_compu_time * (num_scans - 1) / num_scans + compu_time / num_scans;
            aver_smooth_time = aver_smooth_time * (num_scans - 1) / num_scans + smooth_time / num_scans;
            ROS_INFO("[Ring FALS] mean project %0.3fms, compute %0.3fms, smooth %0.3fms, total %0.3fms",
                     aver_proj_time, aver_compu_time, aver_smooth_time, aver_normal_time);
        }
    }

    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig->points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig->points[0].y, pl_orig->points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig->points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig->points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig->points[i].y, pl_orig->points[i].x) * 57.29578;
          break;
        }
      }
    }

      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        const PointXYZIRT& orig_pt = pl_orig->points[i];
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        added_pt.x = orig_pt.x;
        added_pt.y = orig_pt.y;
        added_pt.z = orig_pt.z;
        added_pt.intensity = orig_pt.intensity;
        added_pt.curvature = orig_pt.time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

        if (!given_offset_time)
        {
          int layer = orig_pt.ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        if (i % point_filter_num == 0)
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
          {
//              if (!pointInImage(orig_pt, rowIdn, columnIdn))
//                continue;
              const int & index =  cloud2image[i];
              if (compute_normal && index > 0)
              {
                  int rowIdn, columnIdn;
                  rowIdn = index / image_cols;
                  columnIdn = index - rowIdn * image_cols;
                  const cv::Vec3f &n_cv = normals.at<cv::Vec3f>(rowIdn, columnIdn);
                  if (!std::isfinite(n_cv(0)) || !std::isfinite(n_cv(1)) || !std::isfinite(n_cv(2)))
                      continue;
                  added_pt.normal_x = n_cv(0);
                  added_pt.normal_y = n_cv(1);
                  added_pt.normal_z = n_cv(2);
              }
              pl_surf.points.push_back(added_pt);
          }
        }
      }

//    pl_surf.resize(pl_surf.points.size());
//    if (pl_surf.size() > 0)
//        pcl::io::savePCDFile("/tmp/pl_surf.pcd", pl_surf);
}

bool Preprocess::pointInImage(const PointXYZIRT& point, int& rowIdn, int& columnIdn)
{
    rowIdn = (int)point.ring;
    if (rowIdn < 0 || rowIdn >= N_SCANS)
        return false;
    float horizonAngle = atan2(point.y, -point.x);
    if (horizonAngle < 0)
        horizonAngle += 2 * M_PI;
    horizonAngle = horizonAngle  * 180 / M_PI;
    columnIdn = round(horizonAngle/ ang_res_x);
    if (columnIdn < 0 || columnIdn >= image_cols)
        return false;
    return true;
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

void Preprocess::initNormalEstimator() {
//    image_cols = Horizon_SCAN / point_filter_num;
    image_cols = Horizon_SCAN;
    range_image = RangeImage(N_SCANS, image_cols);
    ang_res_x = 360.0/float(image_cols); //resolution 360 / 1800 = 0.2
    if (!compute_table) {
        if (!range_image.loadLookupTable(ring_table_dir, "ring" + std::to_string(N_SCANS))) {
            ROS_ERROR("Wrong path to ring normal M file, EXIT.");
        }
//            range_image.saveLookupTable("/tmp/table", "ring32");
    }
    rangeMat = cv::Mat(N_SCANS, image_cols, CV_32F, cv::Scalar::all(FLT_MAX));
}
