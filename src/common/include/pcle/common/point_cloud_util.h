#ifndef PCLE_COMMON_POINT_CLOUD_UTIL_H
#define PCLE_COMMON_POINT_CLOUD_UTIL_H

#include <string>

#include "pcle/common/types.h"
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

namespace pcle {
namespace common {

template <class _Point>
bool pcl_save(const std::string &file_name, pcl::PointCloud<_Point> &cloud) {
    pcl::PLYWriter w;
    return (w.write (file_name, cloud, false, true));
}

PointCloudNT::Ptr pcl_transformPointCloudWithNormal(PointCloudNT::Ptr& src, Transform& transform);

double computerCloudResolution(PointCloudNT::ConstPtr cloud);
double computerCloudResolution(PointCloudT::ConstPtr  cloud);

template <typename _Point = PointT>
typename pcl::PointCloud<_Point>::Ptr
pcl_copyPointCloud(const pcl::PointCloud<_Point> &cloud) {
    typename pcl::PointCloud<_Point>::Ptr newCloud(new pcl::PointCloud<_Point>);
    pcl::copyPointCloud(cloud, *newCloud);
    return newCloud;
}

std::vector<int> pcl_kdtree_search(PointCloudT::Ptr cloud, const PointT &point, int k);

PointCloudNT::Ptr pcl_normalEstimation(const PointCloudT::Ptr& cloud);

PointCloudNT::Ptr pcl_downSample(const PointCloudNT::Ptr& cloud, double leafSize);
PointCloudT::Ptr  pcl_downSample(const PointCloudT::Ptr&  cloud, double leafSize);

void pcl_StatisticalOutlier_self(const PointCloudT::Ptr&  cloud, int meanK, double std_mul);
void pcl_StatisticalOutlier_self(const PointCloudNT::Ptr& cloud, int meanK, double std_mul);

void pcl_remove_plane(PointCloudT::Ptr  cloud, double distanceThreshold);
void pcl_remove_plane(PointCloudNT::Ptr cloud, double distanceThreshold);

PointCloudNT::Ptr pcl_concatenate(const PointCloudT::Ptr& cloud, 
                                const PointCloudN::Ptr& normals);

PointCloudN::Ptr pcl_nt_2_n(const PointCloudNT::Ptr &cloud);
PointCloudT::Ptr pcl_nt_2_t(const PointCloudNT::Ptr &cloud);

BoundingBox pcl_boundingBox(const PointCloudT::Ptr&  cloud);
BoundingBox pcl_boundingBox(const PointCloudNT::Ptr& cloud);

float pcl_diameter(const PointCloudT::Ptr&  cloud);
float pcl_diameter(const PointCloudNT::Ptr& cloud);

PointCloudNT::Ptr pcl_filterByNormal(const PointCloudNT &cloud, PointN normal,
                                     double angleThresInDegree);

PointCloudT::Ptr pcl_filterByBox(PointCloudT::Ptr in, PointT min, PointT max);

PointCloudNT::Ptr pcl_loadPointCloudNT(std::string modelPath);
PointCloudT::Ptr  pcl_loadPointCloudT(std::string modelPath);

} // namespace common
} // namespace pcle

#endif // PCLE_COMMON_POINT_CLOUD_UTIL_H
