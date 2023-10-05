//
// Created by hk on 12/8/22.
//

#pragma once
#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#ifndef LIO_POINT_TYPE_H
#define LIO_POINT_TYPE_H


struct ikdTree_PointType
{
    PCL_ADD_POINT4D
    PCL_ADD_NORMAL4D;
    union {
        struct {
            float intensity;
            float curvature;
            float s_xx;
            float s_xy;
        };
        float data_c[4];
    };
//    Eigen::Matrix3f S; // summation p * p^t
//    Eigen::Vector3f mean; //

    union {
        struct {
            float s_xz;
            float s_yy;
            float s_yz;
            float s_zz;
        };
        float data_s[4];
    };

    union {
        struct {
            int num_hit;
            float mean_x;
            float mean_y;
            float mean_z;
        };
        float data_num[4];
    };
//
//    //todo using s_xx as point cov?
//    union {
//        struct {
//            float cov_xx;
//            float cov_yy;
//            float cov_zz;
//            float cov_xy;
//        };
//        float data_cov_1[4];
//    };
//
//    union {
//        struct {
//            float cov_xz;
//            float cov_yz;
//            float n_cov_xx;
//            float n_cov_xy;
//        };
//        float data_cov_2[4];
//    };
//
//    union {
//        struct {
//            float n_cov_xz;
//            float n_cov_yz;
//            float n_cov_zz;
//            float n_cov_yy;
//        };
//        float data_cov_3[4];
//    };
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//    void pointCovMatrix(Eigen::Matrix3f & m )
//    {
//        m(0, 0) = cov_xx; m(0, 1) = cov_xy; m(0, 2) = cov_xz;
//        m(1, 0) = cov_xy; m(1, 1) = cov_yy; m(1, 2) = cov_yz;
//        m(2, 0) = cov_xz; m(2, 1) = cov_yz; m(2, 2) = cov_zz;
//    }
//    void normalCovMatrix(Eigen::Matrix3f & m )
//    {
//        m(0, 0) = n_cov_xx; m(0, 1) = n_cov_xy; m(0, 2) = n_cov_xz;
//        m(1, 0) = n_cov_xy; m(1, 1) = n_cov_yy; m(1, 2) = n_cov_yz;
//        m(2, 0) = n_cov_xz; m(2, 1) = n_cov_yz; m(2, 2) = n_cov_zz;
//    }
//
//    void recordPointCovFromMatrix(const Eigen::Matrix3f & m)
//    {
//        cov_xx = m(0, 0); cov_xy = m(0, 1); cov_xz = m(0, 2);
//        cov_yy = m(1, 1); cov_yz = m(1, 2); cov_zz = m(2, 2);
//    }
//    void recordNormalCovFromMatrix(const Eigen::Matrix3f & m)
//    {
//        n_cov_xx = m(0, 0); n_cov_xy = m(0, 1); n_cov_xz = m(0, 2);
//        n_cov_yy = m(1, 1); n_cov_yz = m(1, 2); n_cov_zz = m(2, 2);
//    }
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (ikdTree_PointType,
   (float, x, x) (float, y, y) (float, z, z)
   (float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z)
   (float, intensity, intensity) (float, curvature, curvature) (float, s_xx, s_xx) (float, s_xy, s_xy)
   (float, s_xz, s_xz) (float, s_yy, s_yy) (float, s_yz, s_yz) (float, s_zz, s_zz)
   (int, num_hit, num_hit) (float, mean_x, mean_x) (float, mean_y, mean_y) (float, mean_z, mean_z)
)

// Velodyne
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (float, time, time)
)

// Ouster
struct ousterPointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(ousterPointXYZIRT,
      (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
      (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
      (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)


#endif //LIO_POINT_TYPE_H
