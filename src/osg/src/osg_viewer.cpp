#include "pcle/osg/osg_viewer.h"

namespace pcle {
namespace osgplus {

OsgViewer::OsgViewer() {
    _root = new osg::Group;
    _viewer = osg_createViewer();
    _viewer->setKeyEventSetsDone(0);
}

void OsgViewer::show() {
    if (_showOnce) {
        return;
    }
    _showOnce = true;

    _root->getOrCreateStateSet()->setMode(
        GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

    osgUtil::Optimizer optimizer;
    optimizer.optimize(_root.get());
    _viewer->setSceneData(_root.get());
    _viewer->run();
}

void OsgViewer::showInThread() {
    if (_showOnce) {
        return;
    }
    auto f = [this] { show(); };
    std::thread t(f);
    t.detach();
}

void OsgViewer::addPoints(PointCloudT::Ptr cloud, double pointSize,
                          osg::Vec4 color) {
    _root->addChild(drawPoints(cloud, pointSize, color));
}

void OsgViewer::addPoints(PointCloudNT::Ptr cloud, double pointSize,
                          osg::Vec4 color) {
    _root->addChild(drawPoints(cloud, pointSize, color));
}

void OsgViewer::addPoints(PointCloudNT::Ptr cloud, double pointSize,
                          std::vector<osg::Vec4> colors) {
    _root->addChild(drawPoints(cloud, pointSize, colors));
}


void OsgViewer::addPointsWithNormal(PointCloudNT::Ptr cloud, double pointSize,
                                    osg::Vec4 pointColor, double normalLength,
                                    double normalWidth, osg::Vec4 normalColor) {
    _root->addChild(drawPointsWithNormal(
        cloud, pointSize, pointColor, normalLength, normalWidth, normalColor));
}

void OsgViewer::addNode(osg::ref_ptr<osg::Node> node) {
    _root->addChild(node.get());
}

void OsgViewer::addText(std::string text, osg::Vec3 position, osg::Vec4 color) {

    osg::ref_ptr<osgText::Text> osgText = drawText(text, position, color);
    _root->addChild(osgText);
}

void OsgViewer::addLines(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2,
                         std::vector<std::pair<int, int>> correspondences,
                         double lineWidth, osg::Vec4 lineColor) {
    osg::ref_ptr<osg::Vec3Array> vertices1(new osg::Vec3Array);
    osg::ref_ptr<osg::Vec3Array> vertices2(new osg::Vec3Array);

    for (int k = 0; k < correspondences.size(); k++) {
        int i = correspondences[k].first;
        int j = correspondences[k].second;
        PointT point1 = cloud1->points[i];
        PointT point2 = cloud2->points[j];
        vertices1->push_back({point1.x, point1.y, point1.z});
        vertices2->push_back({point2.x, point2.y, point2.z});
    }
    _root->addChild(drawLines(vertices1, vertices2, lineColor, lineWidth));
}

void OsgViewer::addLines(PointCloudNT::Ptr cloud1, PointCloudNT::Ptr cloud2,
                         std::vector<std::pair<int, int>> correspondences,
                         double lineWidth, osg::Vec4 lineColor) {
    osg::ref_ptr<osg::Vec3Array> vertices1(new osg::Vec3Array);
    osg::ref_ptr<osg::Vec3Array> vertices2(new osg::Vec3Array);

    for (int k = 0; k < correspondences.size(); k++) {
        int i = correspondences[k].first;
        int j = correspondences[k].second;
        PointNT point1 = cloud1->points[i];
        PointNT point2 = cloud2->points[j];
        vertices1->push_back({point1.x, point1.y, point1.z});
        vertices2->push_back({point2.x, point2.y, point2.z});
    }
    _root->addChild(drawLines(vertices1, vertices2, lineColor, lineWidth));
}

void OsgViewer::clear() {
    int childrenNum = _root->getNumChildren();
    _root->removeChildren(0, childrenNum);
}

void OsgViewer::remove(int nodeIdx) { _root->removeChildren(nodeIdx, 1); }

} // namespace osgplus
} // namespace pcle