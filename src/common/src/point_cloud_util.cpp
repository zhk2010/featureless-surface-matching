#include "pcle/common/point_cloud_util.h"

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>

#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/registration/transformation_estimation_svd.h>

#include <fstream>

namespace pcle {
namespace common {

PointCloudNT::Ptr pcl_transformPointCloudWithNormal(PointCloudNT::Ptr& src, Transform& trans){
    PointCloudNT::Ptr dst(new PointCloudNT);
    pcl::transformPointCloudWithNormals(*src, *dst, trans.matrix());
    return dst;
}

double computerCloudResolution(PointCloudNT::ConstPtr cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<PointNT> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!std::isfinite((*cloud)[i].x)) {
            continue;
        }
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2) {
            res += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0) {
        res /= n_points;
    }
    return res;
}

double computerCloudResolution(PointCloudT::ConstPtr cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!std::isfinite((*cloud)[i].x)) {
            continue;
        }
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2) {
            res += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0) {
        res /= n_points;
    }
    return res;
}

std::vector<int> pcl_kdtree_search(PointCloudT::Ptr cloud, const PointT &point,
                                   int k) {
    pcl::search::KdTree<PointT> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> indices;
    std::vector<float> distances;
    int size = kdtree.nearestKSearch(point, k, indices, distances);
    std::vector<int> indices_solve;
    for (int i = 0; i < size; i++) {
        int index = indices[i];
        indices_solve.push_back(index);
    }
    return indices_solve;
}

PointCloudNT::Ptr pcl_normalEstimation(const PointCloudT::Ptr& cloud) {
    PointCloudN::Ptr normals(new PointCloudN);
    pcl::NormalEstimationOMP<PointT, PointN> n;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    return pcl_concatenate(cloud, normals);
}

PointCloudNT::Ptr pcl_downSample(const PointCloudNT::Ptr& cloud, double leafSize) {
    PointCloudNT::Ptr downSampledCloud(new PointCloudNT());
    pcl::VoxelGrid<PointNT> sor_scene;
    sor_scene.setInputCloud(cloud);
    sor_scene.setLeafSize(leafSize, leafSize, leafSize);
    sor_scene.filter(*downSampledCloud);
    return downSampledCloud;
}

PointCloudT::Ptr pcl_downSample(const PointCloudT::Ptr& cloud, double leafSize) {
    PointCloudT::Ptr downSampledCloud(new PointCloudT());
    pcl::VoxelGrid<PointT> sor_scene;
    sor_scene.setInputCloud(cloud);
    sor_scene.setLeafSize(leafSize, leafSize, leafSize);
    sor_scene.filter(*downSampledCloud);
    return downSampledCloud;
}

void pcl_StatisticalOutlier_self(PointCloudT::Ptr cloud, int meanK,
                                 double std_mul) {
    pcl::StatisticalOutlierRemoval<PointT> Statistical;
    Statistical.setInputCloud(cloud);
    Statistical.setMeanK(meanK);
    Statistical.setStddevMulThresh(std_mul);
    Statistical.filter(*cloud);
}

void pcl_StatisticalOutlier_self(PointCloudNT::Ptr cloud, int meanK,
                                 double std_mul) {
    pcl::StatisticalOutlierRemoval<PointNT> Statistical;
    Statistical.setInputCloud(cloud);
    Statistical.setMeanK(meanK);
    Statistical.setStddevMulThresh(std_mul);
    Statistical.filter(*cloud);
}

void pcl_remove_plane(PointCloudT::Ptr cloud, double distanceThreshold) {
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_indices(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(45.0f / 180 * M_PI);
    seg.setInputCloud(cloud);
    seg.segment(*inliers_indices, *coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_indices);
    extract.setNegative(false);
    extract.filter(*cloud);
}

void pcl_remove_plane(PointCloudNT::Ptr cloud, double distanceThreshold) {
    pcl::SACSegmentation<PointNT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_indices(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers_indices, *coefficients);

    pcl::ExtractIndices<PointNT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_indices);
    extract.setNegative(false);
    extract.filter(*cloud);
}

PointCloudNT::Ptr pcl_concatenate(const PointCloudT::Ptr& cloud,
                                  const PointCloudN::Ptr& normals) {
    PointCloudNT::Ptr cloudnt(new PointCloudNT);
    pcl::concatenateFields(*cloud, *normals, *cloudnt);
    return cloudnt;
}

PointCloudN::Ptr pcl_nt_2_n(const PointCloudNT::Ptr &cloud) {
    PointCloudN::Ptr dst(new PointCloudN());
    pcl::PCLPointCloud2::Ptr m(new pcl::PCLPointCloud2());

    pcl::toPCLPointCloud2(*cloud, *m);
    pcl::fromPCLPointCloud2(*m, *dst);
    return dst;
}

PointCloudT::Ptr pcl_nt_2_t(const PointCloudNT::Ptr &cloud) {
    PointCloudT::Ptr dst(new PointCloudT());
    pcl::PCLPointCloud2::Ptr m(new pcl::PCLPointCloud2());

    pcl::toPCLPointCloud2(*cloud, *m);
    pcl::fromPCLPointCloud2(*m, *dst);
    return dst;
}

BoundingBox pcl_boundingBox(const PointCloudT::Ptr& cloud) {
    PointT _min;
    PointT _max;
    pcl::getMinMax3D(*cloud, _min, _max);
    return {_min, _max};
}

BoundingBox pcl_boundingBox(const PointCloudNT::Ptr& cloud) {
    PointCloudT::Ptr cloudT = pcl_nt_2_t(cloud);
    return pcl_boundingBox(cloudT);
}

float pcl_diameter(const PointCloudT::Ptr& cloud){
    float model_diameter = 0;
    for(PointT& p1 : *cloud){
        for(PointT& p2 : *cloud){
            model_diameter = std::max(model_diameter, pcl::euclideanDistance(p1, p2));
        }    
    }
    return model_diameter;
}

float pcl_diameter(const PointCloudNT::Ptr& cloud){
    float model_diameter = 0;
    for(PointNT& p1 : *cloud){
        for(PointNT& p2 : *cloud){
            model_diameter = std::max(model_diameter, pcl::euclideanDistance(p1, p2));
        }    
    }
    return model_diameter;
}

PointCloudNT::Ptr pcl_filterByNormal(const PointCloudNT &cloud, PointN normal,
                                     double angleThresInDegree) {
    PointCloudNT::Ptr cloudOut(new PointCloudNT);
    for (auto p : cloud) {
        double angle = pcl::getAngle3D(
            Eigen::Vector3f(p.normal_x, p.normal_y, p.normal_z),
            Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z),
            true);
        if (angle < angleThresInDegree) {
            cloudOut->points.push_back(p);
        }
    }
    return cloudOut;
}

void pcl_filterByBox(PointCloudT::Ptr in, PointCloudT::Ptr out, PointT min,
                     PointT max) {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min.z, max.z);
    pass.filter(*out);

    pass.setInputCloud(out);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min.y, max.y);
    pass.filter(*out);

    pass.setInputCloud(out);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min.x, max.x);
    pass.filter(*out);
}

PointCloudT::Ptr pcl_filterByBox(PointCloudT::Ptr in, PointT min, PointT max) {
    PointCloudT::Ptr out(new PointCloudT());
    pcl_filterByBox(in, out, min, max);
    return out;
}

bool implyContainsStr(std::string inputStr, std::string containStr) {
    std::string::size_type idx = inputStr.find(containStr);
    if (idx != std::string::npos) {
        return true;
    } else {
        return false;
    }
}

PointCloudNT::Ptr pcl_loadPointCloudNT(std::string modelPath) {
    PointCloudNT::Ptr modelWithNormal(new PointCloudNT());
    if (implyContainsStr(modelPath, ".pcd")) {
        if (pcl::io::loadPCDFile(modelPath, *modelWithNormal) == -1) {
            PCL_ERROR("Couldn't read pcd file\n");
        }
    } else if (implyContainsStr(modelPath, ".ply")) {
        if (pcl::io::loadPLYFile(modelPath, *modelWithNormal) == -1) {
            PCL_ERROR("Couldn't read ply file\n");
        }
    } else if (implyContainsStr(modelPath, ".obj")) {
        if (pcl::io::loadOBJFile(modelPath, *modelWithNormal) == -1) {
            PCL_ERROR("Couldn't read obj file\n");
        }
    } else {
        PCL_ERROR("Read file is wrong!\n");
    }
    return modelWithNormal;
}

PointCloudT::Ptr pcl_loadPointCloudT(std::string modelPath) {
    PointCloudT::Ptr modelWithoutNormal(new PointCloudT());
    if (implyContainsStr(modelPath, ".pcd")) {
        if (pcl::io::loadPCDFile(modelPath, *modelWithoutNormal) == -1) {
            PCL_ERROR("Couldn't read pcd file\n");
        }
    } else if (implyContainsStr(modelPath, ".ply")) {
        if (pcl::io::loadPLYFile(modelPath, *modelWithoutNormal) == -1) {
            PCL_ERROR("Couldn't read ply file\n");
        }
    } else if (implyContainsStr(modelPath, ".obj")) {
        if (pcl::io::loadOBJFile(modelPath, *modelWithoutNormal) == -1) {
            PCL_ERROR("Couldn't read obj file\n");
        }
    } else {
        std::cout << "Read file is wrong!" << std::endl;
    }
    return modelWithoutNormal;
}

pcl::PolygonMesh pointCloudToPolygonMesh(PointCloudNT::Ptr inputCloud) {
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
        new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(inputCloud);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(3.0);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees
    gp3.setMinimumAngle(M_PI / 36);       // 10 degrees
    gp3.setMaximumAngle(5 * M_PI / 6);    // 120 degrees
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(inputCloud);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    std::cout << "parts.size(): " << parts.size() << std::endl;
    std::cout << "states.size(): " << states.size() << std::endl;

    std::cout << "parts[0]: " << parts[0] << std::endl;
    std::cout << "states[0]: " << states[0] << std::endl;

    return triangles;
}

} // namespace common
} // namespace pcle