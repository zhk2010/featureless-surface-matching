#ifndef PCLE_OSGPLUS_OSGVIEWER_H
#define PCLE_OSGPLUS_OSGVIEWER_H

#include <osg/Array>
#include <osg/AutoTransform>
#include <osg/BufferIndexBinding>
#include <osg/BufferObject>
#include <osg/ComputeBoundsVisitor>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osg/ref_ptr>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgManipulator/Selection>
#include <osgManipulator/TranslateAxisDragger>
#include <osgText/Text>
#include <osgUtil/Optimizer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>

#include "glog/logging.h"
#include "pcle/osg/osg_utils.h"

namespace pcle {
namespace osgplus {

class OsgViewer {
  private:
    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osgViewer::Viewer> _viewer;

    bool _showOnce = false;
  public:
    OsgViewer();

    void clear();
    void show(); 
    void showInThread();
    void remove(int nodeIdx);

    void addPoints(PointCloudT::Ptr  cloud, double pointSize = 1, osg::Vec4 color = pcle::osgplus::WHITE);
    void addPoints(PointCloudNT::Ptr cloud, double pointSize = 1, osg::Vec4 color = pcle::osgplus::WHITE);
    void addPoints(PointCloudT::Ptr  cloud, double pointSize, std::vector<osg::Vec4> colors);
    void addPoints(PointCloudNT::Ptr cloud, double pointSize, std::vector<osg::Vec4> colors);
    void addPointsWithNormal(PointCloudNT::Ptr cloud, double pointSize = 1,
                             osg::Vec4 pointColor = WHITE,
                             double normalLength = 7.0,
                             double normalWidth = 1.0,
                             osg::Vec4 normalColor = RED);
    void addNode(osg::ref_ptr<osg::Node> node);
    void addText(std::string text, osg::Vec3 position,
                 osg::Vec4 color = pcle::osgplus::WHITE);

    void addLines(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2,
                  std::vector<std::pair<int, int>> correspondences,
                  double lineWidth = 1,
                  osg::Vec4 lineColor = pcle::osgplus::WHITE);
    void addLines(PointCloudNT::Ptr cloud1, PointCloudNT::Ptr cloud2,
                  std::vector<std::pair<int, int>> correspondences,
                  double lineWidth = 1,
                  osg::Vec4 lineColor = pcle::osgplus::WHITE);
};

} // namespace osgplus
} // namespace pcle
#endif // PCLE_OSGPLUS_OSGVIEWER_H