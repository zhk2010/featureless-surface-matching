#ifndef TFTECH_PCLE_MATCHER_MATCHER_NOFEATURE_H
#define TFTECH_PCLE_MATCHER_MATCHER_NOFEATURE_H

#include <flann/flann.hpp>
#include <pcl/features/ppf.h>
#include <pcl/common/distances.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/registration/registration.h>

#include <map>
#include <mutex>

#include "pcle/common/utils.h"

using namespace pcl;

namespace pcle {
namespace matcher {

class MatcherNoFeature {
  public:

    void setModel(const PointCloudNT::Ptr model);
    std::vector<Cluster> match(PointCloudNT::Ptr& scene);
  private:
    void clusterPoses(std::vector<Transform> &transformVec, std::vector<Cluster>& result);
    bool posesWithinErrorBounds(Transform &pose1, Transform &pose2);

  public:
    int   scene_reference_point_interval = 1;
    float clustering_position_diff_threshold;
    float clustering_rotation_diff_threshold;
    int max_clusters = 32;
    int angleNums = 36;

    float test_overlap = 0.2;

    float leaf_size;
    float model_diameter;

    float cos_clustering_rotation_diff_threshold;

    bool debug = false;
  public:
    PointCloudNT::Ptr model;
};

} // namespace matcher
} // namespace pcle

#endif // TFTECH_PCLE_MATCHER_MATCHER_NOFEATURE_H
