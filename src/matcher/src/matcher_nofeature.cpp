

#include "pcle/matcher/matcher_nofeature.h"

#include "pcle/common/utils.h"
#include "pcle/osg/osg_viewer.h"

#include <random>
#include <map>

#include "glog/logging.h"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace pcle {
namespace matcher {

void MatcherNoFeature::setModel(const PointCloudNT::Ptr model) {
    this->model = model;

    model_diameter = pcle::common::pcl_diameter(model);
    cos_clustering_rotation_diff_threshold = cosf(clustering_rotation_diff_threshold);
}


inline 
float length(Eigen::Vector3f& n){
    return sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
}

inline 
Eigen::Vector3f norm(Eigen::Vector3f& n){
    float len = length(n); //(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    Eigen::Vector3f nn = {n[0]/len, n[1]/len, n[2]/len};
    return nn;
}

inline 
Eigen::Vector3f cross(Eigen::Vector3f& n1, Eigen::Vector3f& n2){
    Eigen::Vector3f k = {
                n1[1] * n2[2] - n1[2] * n2[1],
                n1[2] * n2[0] - n1[0] * n2[2],
                n1[0] * n2[1] - n1[1] * n2[0]};
    return k;
}

inline 
float dot(const Eigen::Vector3f& n1, const Eigen::Vector3f& n2){
    return n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];
}


Eigen::Matrix3f calcRotateTrans(Eigen::Vector3f kn, float beta){
    Eigen::Matrix3f I;
    I << 1,0,0,0,1,0,0,0,1;

    Eigen::Matrix3f K;
    K <<     0, -kn[2],  kn[1], 
         kn[2],      0, -kn[0],
        -kn[1],  kn[0],      0;
    
    Eigen::Matrix3f K2 = K * K;

    float sina = sin(beta);
    float cosa = cos(beta);

    Eigen::Matrix3f R = I + sina * K + (1-cosa)*K2;
    return R;
}

Eigen::Matrix3f calcRotateTrans(Eigen::Vector3f n1, Eigen::Vector3f n2){
    float l1 = sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]);
    float l2 = sqrt(n2[0] * n2[0] + n2[1] * n2[1] + n2[2] * n2[2]);

    Eigen::Vector3f n1n = norm(n1);
    Eigen::Vector3f n2n = norm(n2);

    float beta = acos(n1n[0] * n2n[0] + n1n[1] * n2n[1] + n1n[2] * n2n[2]); 

    Eigen::Vector3f k = cross(n1n, n2n);
    Eigen::Vector3f kn = norm(k);

    return calcRotateTrans(kn, beta);
}


std::vector<Cluster> MatcherNoFeature::match(PointCloudNT::Ptr& scene){

    pcle::common::TimeCount timeCountAll;  
    pcle::common::TimeCount timeCountUnit;

    if(debug) LOG(INFO) << "model.size = " << model->size();
    if(debug) LOG(INFO) << "scene.size = " << scene->size();

    pcl::KdTreeFLANN<PointNT>::Ptr scene_search_tree_(new pcl::KdTreeFLANN<PointNT>);
    scene_search_tree_->setInputCloud(scene);
    
    int modelSize = model->size();
    int sceneSize = scene->size();

    int sceneDataSize = sceneSize / scene_reference_point_interval;
    std::vector<Transform> transformVec(sceneDataSize * modelSize * angleNums);

    #pragma omp parallel for
    for (int sceneDataIdx = 0; sceneDataIdx < sceneDataSize; sceneDataIdx ++) {
        int sceneIdx = sceneDataIdx * scene_reference_point_interval;
        LOG(INFO) << "sceneIdx " << sceneIdx << "/" << sceneSize;
        PointNT& pointS = (*scene)[sceneIdx];
        Eigen::Vector3f posS{pointS.x, pointS.y, pointS.z};
        Eigen::Vector3f normalS{pointS.normal_x, pointS.normal_y, pointS.normal_z};
        for(int modelIdx = 0; modelIdx < modelSize; modelIdx++){

            PointNT& pointM = (*model)[modelIdx];
            Eigen::Vector3f posM{pointM.x, pointM.y, pointM.z};
            Eigen::Vector3f normalM{pointM.normal_x, pointM.normal_y, pointM.normal_z};
  
            float l2 = dot(-posM, normalM);

            Eigen::Vector3f o1d = l2 * normalM; 
            Eigen::Vector3f Tm = o1d + posM;

            float lenPM = length(Tm);

            Eigen::Vector3f o2d = l2 * normalS;
            Eigen::Vector3f pS = posS + o2d;

            Eigen::Vector3f o2r{ lenPM * pointS.normal_y / sqrt(pointS.normal_x * pointS.normal_x + pointS.normal_y * pointS.normal_y),
                        -lenPM * pointS.normal_x / sqrt(pointS.normal_x * pointS.normal_x + pointS.normal_y * pointS.normal_y),
                        0};

            for(int angleIdx = 0; angleIdx < angleNums; angleIdx++){
                Eigen::Matrix3f R = calcRotateTrans(normalS, M_PI * 2  * angleIdx / angleNums);
                Eigen::Vector3f v = R * o2r;
                Eigen::Vector3f oS = v + pS;
                
                Eigen::Matrix3f rot  = calcRotateTrans(Tm, pS - oS);
                Transform trans;
                trans.setTranslation(oS);
                trans.setRotation(rot);

                Eigen::Vector3f posMtrans = trans * posM;
                Eigen::Vector3f pMtrans = trans * Tm;
                Eigen::Matrix3f rot2 = calcRotateTrans(posMtrans - pMtrans, posS - pS);
                Transform trans2;
                trans2.setRotation(rot2);

                // Transform transAll = Transform::from(rot2 * rot, oS);
                transformVec[sceneDataIdx * modelSize * angleNums + modelIdx * angleNums + angleIdx] = Transform::from(rot2 * rot, oS);
            }
        }
    }

    std::vector<Cluster> results;
    clusterPoses(transformVec, results);
    return results;
}


void pcle::matcher::MatcherNoFeature::clusterPoses(std::vector<Transform> &transformVec, std::vector<Cluster>& result) {

    pcle::common::TimeCount time_count_;
    time_count_.start();

    std::map<int, std::vector<int>> hashClusters;
    std::vector<Cluster> clusters;

    int MAX_CLUSTERS = 100000;

    int transSize = transformVec.size();
    int i = 0;
    for(Transform trans : transformVec){
        i++;
        if(i % 1000000 == 0){
            printf("clusterPoses %d/%d\n", i, transSize);
        }

        Eigen::Vector3f t = trans.translation();
        auto r = trans.axis();
        int x = round(t[0] / clustering_position_diff_threshold + 10240);
        int y = round(t[1] / clustering_position_diff_threshold + 10240);
        int z = round(t[2] / clustering_position_diff_threshold + 10240);
        x = x % 1024;
        y = y % 1024;
        z = z % 1024;
        int hash = ((x & 0xFFF) << 24) | ((y & 0xFFF) << 12) | ((z & 0xFFF));

        auto vvv = trans.quaternion().coeffs();
        if(std::isnan(vvv[0])){
            continue;
        }

        if(hashClusters.count(hash) < 1){
            if(clusters.size() >= MAX_CLUSTERS - 1) continue;
            hashClusters[hash] = std::vector<int>();
        }
        std::vector<int>& cs = hashClusters[hash];

        bool found_cluster = false;
        for(int clusterId : cs){
            Cluster& cluster = clusters[clusterId];
            if (posesWithinErrorBounds(trans, cluster.transform)) {
                found_cluster = true;
                
                cluster.votes += 1;
                cluster.translation_sum += trans.translation();
                cluster.rotation_sum += trans.quaternion().coeffs();

                break;
            }
        }
        if(!found_cluster){
            if(clusters.size() >= MAX_CLUSTERS - 1) continue;

            Cluster cluster;
            cluster.transform = trans;
            cluster.votes = 1;
            cluster.translation_sum = trans.translation();
            cluster.rotation_sum = trans.quaternion().coeffs();

            clusters.push_back(cluster);
            cs.push_back(clusters.size()-1);

            if(clusters.size() % 1000 == 0){
                LOG(INFO) << "clusters.size = " << clusters.size();
            }
        }
    }

    if(debug) LOG(INFO) << "cluster pose: " << time_count_.countMilliSecond() << "ms";
    time_count_.start();

    std::sort(clusters.begin(), clusters.end(), 
            [](const Cluster &a, const Cluster &b) {
                return (a.votes > b.votes);
            }
    );

    if(debug) LOG(INFO) << "sort cluster: " << time_count_.countMilliSecond() << "ms";
    time_count_.start();

    for (std::size_t cluster_i = 0; cluster_i < clusters.size(); ++cluster_i) {
        Cluster cluster = clusters[cluster_i];

        Eigen::Vector3f translation_average = cluster.translation_sum / cluster.votes;
        Eigen::Vector4f rotation_average = cluster.rotation_sum / cluster.votes;

        Eigen::Affine3f transform_average;
        transform_average.translation().matrix() = translation_average;
        transform_average.linear().matrix() = Eigen::Quaternionf(rotation_average).normalized().toRotationMatrix();

        cluster.transform = Transform(transform_average);

        result.push_back(cluster);

        if (result.size() >= max_clusters)
            break;
    }
}



bool pcle::matcher::MatcherNoFeature::posesWithinErrorBounds(Transform &pose1,
                                                         Transform &pose2) {
    // float position_diff = (pose1.translation() - pose2.translation()).norm();
    // if (position_diff > clustering_position_diff_threshold)
    //     return false;


    if(pose1.quaternion().dot(pose2.quaternion()) < cos_clustering_rotation_diff_threshold){
        return false;
    }
    return true;
}

} // namespace matcher
} // namespace pcle
