#ifndef PCLE_OSGPLUS_OSGUTILS_H
#define PCLE_OSGPLUS_OSGUTILS_H

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

#include "glog/logging.h"

#include "pcle/common/types.h"

namespace pcle {
namespace osgplus {

static int osg_viewer_width = 1800;
static int osg_viewer_height = 1600;
static int osg_viewer_left = 1000;
static int osg_viewer_top = 200;
static int osg_point_size = 8;

static osg::Vec4 RED = osg::Vec4(1, 0, 0, 1);
static osg::Vec4 GREEN = osg::Vec4(0, 1, 0, 1);
static osg::Vec4 BLUE = osg::Vec4(0, 0, 1, 1);
static osg::Vec4 YELLOW = osg::Vec4(1, 1, 0, 1);
static osg::Vec4 PINK = osg::Vec4(1, 0, 1, 1);
static osg::Vec4 ORANGE = osg::Vec4(1, 0.5, 0, 1);
static osg::Vec4 WHITE = osg::Vec4(1, 1, 1, 1);

static std::default_random_engine random_engine;

osg::ref_ptr<osgViewer::Viewer> osg_createViewer(int left, int top, int width,
                                                 int height);
osg::ref_ptr<osgViewer::Viewer> osg_createViewer();
void osg_show(osg::ref_ptr<osg::Group> root);
void osg_show_geode(osg::ref_ptr<osg::Geode> root);
void osg_show_geometry(osg::ref_ptr<osg::Geometry> root);

osg::Vec4 osg_colorRandom();
osg::Vec4 osg_color(std::string c);

void osg_add_line(osg::ref_ptr<osg::Geode> m_geode, osg::Vec3 n1, osg::Vec3 n2,
                  osg::Vec4 color1 = osg::Vec4(1.0f, 1.0f, 1.0f, 0.2f),
                  osg::Vec4 color2 = osg::Vec4(1.0f, 1.0f, 1.0f, 0.2f));
void osg_add_virtualBox(osg::ref_ptr<osg::Geode> m_geode, osg::Vec3 center,
                        double length, double width, double height);

osg::ref_ptr<osg::Group>
osg_draw_dragger(osg::Vec3 center, float length,
                 osg::Vec4 color = osg::Vec4(1.0f, 1.0f, 1.0f, 0.2f));

osg::ref_ptr<osg::Group> osg_draw_coor(osg::Vec3 center, float length);

osg::BoundingBox osg_boundingBox(osg::Node *node);
osg::BoundingBox osg_boundingBox(osg::ref_ptr<osg::Node> node);

osg::ref_ptr<osg::Node> cretateBoundingBox(osg::Node *node);

void osg_readPCD(std::string filePath, osg::ref_ptr<osg::Vec3Array> &coords,
                 osg::ref_ptr<osg::Vec4Array> &color, int &num, int scale);

osg::ref_ptr<osg::Node> osg_loadFile(std::string filePath);
void osg_save(const osg::Node &node, std::string filePath);

void osg_QuatToHPR(osg::Quat q, double &heading, double &pitch, double &roll);
osg::Quat osg_HPRToQuat(double heading, double pitch, double roll);

enum RotSeq { zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy, xzx };
void twoaxisrot(double r11, double r12, double r21, double r31, double r32,
                double res[]);
void threeaxisrot(double r11, double r12, double r21, double r31, double r32,
                  double res[]);
void quaternion2Euler(osg::Quat q, double res[], RotSeq rotSeq);

osg::ref_ptr<osg::ShapeDrawable>
drawCylinder(int radius, int height,
             osg::Vec4 color = osg::Vec4(1.0f, 1.0f, 1.0f, 0.2f));
osg::ref_ptr<osg::ShapeDrawable>
drawBox(osg::Vec3 center, osg::Vec3 halfLength,
        osg::Vec4 color = osg::Vec4(1.0f, 1.0f, 1.0f, 0.2f));

osg::ref_ptr<osgText::Text> drawText(const std::string &txt, osg::Vec3 pos,
                                     osg::Vec4 color = WHITE);
osg::ref_ptr<osg::Geometry> 
drawPoints(osg::ref_ptr<osg::Vec3Array> vertices, double pointSize = 1, osg::Vec4 color = WHITE);
osg::ref_ptr<osg::Geometry> 
drawPoints(osg::ref_ptr<osg::Vec3Array> vertices, double pointSize, std::vector<osg::Vec4> colors);

osg::ref_ptr<osg::Geometry> 
drawPoints(PointCloudNT::Ptr cloud, double pointSize = 1, osg::Vec4 color = WHITE);
osg::ref_ptr<osg::Geometry> 
drawPoints(PointCloudT::Ptr cloud, double pointSize = 1, osg::Vec4 color = WHITE);
osg::ref_ptr<osg::Geometry> 
drawPoints(PointCloudNT::Ptr cloud, double pointSize, std::vector<osg::Vec4> colors);
osg::ref_ptr<osg::Geometry> 
drawPoints(PointCloudT::Ptr cloud, double pointSize, std::vector<osg::Vec4> colors);

osg::ref_ptr<osg::Geometry> drawLine(osg::Vec3 n1, osg::Vec3 n2,
                                     osg::Vec4 color1 = WHITE,
                                     osg::Vec4 color2 = WHITE,
                                     double lineWidth = 1.0);
osg::ref_ptr<osg::Geometry> drawLines(osg::ref_ptr<osg::Vec3Array> vertices,
                                      osg::ref_ptr<osg::Vec4Array> colors,
                                      double lineWidth = 1.0);
osg::ref_ptr<osg::Geometry> drawLines(osg::ref_ptr<osg::Vec3Array> vertices,
                                      osg::Vec4 lineColor = WHITE,
                                      double lineWidth = 1);
osg::ref_ptr<osg::Geometry> drawLines(osg::ref_ptr<osg::Vec3Array> vertices1,
                                      osg::ref_ptr<osg::Vec3Array> vertices2,
                                      osg::Vec4 lineColor = WHITE,
                                      double lineWidth = 1);
osg::ref_ptr<osg::Geode>
drawPointsWithNormal(PointCloudNT::Ptr cloud, double pointSize = 1,
                     osg::Vec4 pointColor = WHITE, double normalLength = 7.0,
                     double normalWidth = 1.0, osg::Vec4 normalColor = RED);
                     
} // namespace osgplus
} // namespace pcle

#endif // PCLE_OSGPLUS_OSGUTILS_H