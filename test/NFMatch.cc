#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "pcle/common/utils.h"
#include "pcle/osg/utils.h"
#include "pcle/matcher/utils.h"

int main(int argc, char* argv[]){
  
  std::string model_path    = argv[1];
  std::string scene_path    = argv[2];

  LOG(INFO) << "model: " << model_path;
  LOG(INFO) << "scene: " << scene_path;

  PointCloudNT::Ptr sceneOri = pcle::common::pcl_loadPointCloudNT(scene_path);
  PointCloudNT::Ptr modelOri = pcle::common::pcl_loadPointCloudNT(model_path);

  PointCloudNT::Ptr scene = sceneOri;
  PointCloudNT::Ptr model = modelOri;

  // downsample
  float leaf_size = pcle::common::computerCloudResolution(model);
  while(model->size() > 1300){
      leaf_size *= 1.5;
      model = pcle::common::pcl_downSample(modelOri, leaf_size);
  }
  scene = pcle::common::pcl_downSample(sceneOri, leaf_size);
  
  int modelSize = model->size();
  int sceneSize = scene->size();

  {
    pcle::osgplus::OsgViewer viewer;
    viewer.addPoints(modelOri, 3);
    viewer.addPoints(sceneOri, 3, pcle::osgplus::RED);
    viewer.show();
  }

  pcle::matcher::MatcherNoFeature matcher;
  matcher.debug = true;
  matcher.clustering_rotation_diff_threshold = 5 / 180.0f * float(M_PI);
  matcher.clustering_position_diff_threshold = leaf_size * 2;
  matcher.scene_reference_point_interval = 5;

  matcher.setModel(model);
  std::vector<Cluster> allResults = matcher.match(scene);
  
  for (int i = 0; i < 6 && i < allResults.size(); i++) {
    Cluster& cluster = allResults[i];

    Transform transform     = cluster.transform;
    float votesForCluster   = cluster.votes;
   
    if (abs(transform.translation()[0]) > 99000) { continue; }

    LOG(INFO) << "[" << i+1 << "]: " << "votes " << cluster.votes;
    PointCloudNT::Ptr modelTrans = pcle::common::pcl_transformPointCloudWithNormal(model, transform);

    {
      pcle::osgplus::OsgViewer viewer;
      viewer.addPoints(scene, 3);
      viewer.addPoints(modelTrans, 3, pcle::osgplus::RED);
      viewer.show();
    }


    PointCloudNT::Ptr modelOriTrans = pcle::common::pcl_transformPointCloudWithNormal(modelOri, transform);
    pcle::common::pcl_save("modelOriTrans.ply", *modelOriTrans);
  }

  return 0;
}
