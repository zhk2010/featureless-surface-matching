#include "pcle/osg/osg_utils.h"

#include "pcle/common/point_cloud_util.h"

namespace pcle {
namespace osgplus {

osg::ref_ptr<osgViewer::Viewer> osg_createViewer() {
    return osg_createViewer(osg_viewer_left, osg_viewer_top, osg_viewer_width,
                            osg_viewer_height);
}
osg::ref_ptr<osgViewer::Viewer> osg_createViewer(int left, int top, int width, int height) {

    osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
    osg::ref_ptr<osg::GraphicsContext::Traits> traits =
        new osg::GraphicsContext::Traits;
    traits->x = left;
    traits->y = top;
    traits->width = width;
    traits->height = height;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    traits->readDISPLAY();
    traits->setUndefinedScreenDetailsToDefaultScreen();

    osg::ref_ptr<osg::GraphicsContext> gc =
        osg::GraphicsContext::createGraphicsContext(traits.get());
    if (gc.valid()) {
        osg::notify(osg::INFO)
            << "  GraphicsWindow has been created successfully." << std::endl;
        gc->setClearColor(osg::Vec4f(0.2f, 0.2f, 0.6f, 1.0f));
        gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    } else {
        osg::notify(osg::NOTICE)
            << "  GraphicsWindow has not been created successfully."
            << std::endl;
    }

    osg::ref_ptr<osg::Camera> camera = viewer->getCamera();

    double fovy, aspectRatio, zNear, zFar;
    camera->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
    double newAspectRatio = double(traits->width) / double(traits->height);
    double aspectRatioChange = newAspectRatio / aspectRatio;

    camera->setViewport(new osg::Viewport(0, 0, width, height));
    camera->setGraphicsContext(gc.get());

    GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
    camera->setDrawBuffer(buffer);
    camera->setReadBuffer(buffer);

    viewer->addEventHandler(new osgGA::StateSetManipulator(
        viewer->getCamera()->getOrCreateStateSet()));

    return viewer;
}

void osg_show(osg::ref_ptr<osg::Group> root) {
    root->getOrCreateStateSet()->setAttribute(new osg::Point(osg_point_size),
                                              osg::StateAttribute::OVERRIDE);
                                              
    root->getOrCreateStateSet()->setMode(
        GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

    osgUtil::Optimizer optimizer;
    optimizer.optimize(root.get());

    osg::ref_ptr<osgViewer::Viewer> viewer = osg_createViewer();
    viewer->setSceneData(root.get());

    viewer->run();
}

void osg_show_geometry(osg::ref_ptr<osg::Geometry> node) {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(node);
    osg_show(root);
}

void osg_show_geode(osg::ref_ptr<osg::Geode> node) {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(node);
    osg_show(root);
}

osg::Vec4 osg_colorRandom() {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    float v0 = u(random_engine);
    float v1 = u(random_engine);
    float v2 = u(random_engine);
    float sum = v0 + v1 + v2;
    v0 = v0 / sum;
    v1 = v1 / sum;
    v2 = v2 / sum;

    float alpha = 1; // 1-u(random_engine) * 0.5;
    return {v0, v1, v2, alpha};
}

osg::Vec4 osg_color(std::string c) {
    return {0,0,0,0};
}

void osg_add_line(osg::ref_ptr<osg::Geode> m_geode, osg::Vec3 n1, osg::Vec3 n2,
                  osg::Vec4 color1, osg::Vec4 color2) {
    m_geode->addDrawable(drawLine(n1, n2, color1, color2));
}

void osg_add_virtualBox(osg::ref_ptr<osg::Geode> m_geode, osg::Vec3 center,
                        double length, double width, double height) {
    double halfL = length / 2;
    double halfW = width / 2;
    double halfH = height / 2;

    osg_add_line(
        m_geode,
        osg::Vec3(center.x() + halfL, center.y() + halfW, center.z() + halfH),
        osg::Vec3(center.x() + halfL, center.y() + halfW, center.z() + halfH));
    osg_add_line(
        m_geode,
        osg::Vec3(center.x() + halfL, center.y() + halfW, center.z() - halfH),
        osg::Vec3(center.x() + halfL, center.y() + halfW, center.z() - halfH));
    osg_add_line(
        m_geode,
        osg::Vec3(center.x() + halfL, center.y() - halfW, center.z() + halfH),
        osg::Vec3(center.x() + halfL, center.y() - halfW, center.z() + halfH));
    osg_add_line(
        m_geode,
        osg::Vec3(center.x() + halfL, center.y() - halfW, center.z() - halfH),
        osg::Vec3(center.x() + halfL, center.y() - halfW, center.z() - halfH));
    osg_add_line(
        m_geode,
        osg::Vec3(center.x() - halfL, center.y() + halfW, center.z() + halfH),
        osg::Vec3(center.x() - halfL, center.y() + halfW, center.z() + halfH));
    osg_add_line(
        m_geode,
        osg::Vec3(center.x() - halfL, center.y() + halfW, center.z() - halfH),
        osg::Vec3(center.x() - halfL, center.y() + halfW, center.z() - halfH));
    osg_add_line(
        m_geode,
        osg::Vec3(center.x() - halfL, center.y() - halfW, center.z() + halfH),
        osg::Vec3(center.x() - halfL, center.y() - halfW, center.z() + halfH));
    osg_add_line(
        m_geode,
        osg::Vec3(center.x() - halfL, center.y() - halfW, center.z() - halfH),
        osg::Vec3(center.x() - halfL, center.y() - halfW, center.z() - halfH));
}

osg::ref_ptr<osg::Geometry> drawLine(osg::Vec3 n1, osg::Vec3 n2,
                                     osg::Vec4 color1, osg::Vec4 color2,
                                     double lineWidth) {
                                        
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(n1);
    vertices->push_back(n2);

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(color1);
    colors->push_back(color2);

    return drawLines(vertices, colors, lineWidth);
}

osg::ref_ptr<osg::Geometry> drawLines(osg::ref_ptr<osg::Vec3Array> vertices,
                                      osg::Vec4 lineColor, double lineWidth) {
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    for (int i = 0; i < vertices->size(); i++) {
        colors->push_back(lineColor);
    }

    return drawLines(vertices, colors, lineWidth);
}

osg::ref_ptr<osg::Geometry> drawLines(osg::ref_ptr<osg::Vec3Array> vertices,
                                      osg::ref_ptr<osg::Vec4Array> colors,
                                      double lineWidth) {
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setVertexArray(vertices);

    geometry->setColorArray(colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->getOrCreateStateSet()->setAttribute(
        new osg::LineWidth(lineWidth), osg::StateAttribute::OVERRIDE);

    geometry->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));
    return geometry;
}

osg::ref_ptr<osg::Geometry> drawLines(osg::ref_ptr<osg::Vec3Array> vertices1,
                                      osg::ref_ptr<osg::Vec3Array> vertices2,
                                      osg::Vec4 lineColor, double lineWidth) {
    osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array);
    for (int i = 0; i < vertices1->size(); i++) {
        vertices->push_back(vertices1->at(i));
        vertices->push_back(vertices2->at(i));
    }
    return drawLines(vertices, lineColor, lineWidth);
}

osg::ref_ptr<osg::ShapeDrawable> drawCylinder(int radius, int height,
                                              osg::Vec4 color) {
    auto cy = new osg::Cylinder;
    cy->setHeight(height);
    cy->setRadius(radius);
    cy->setCenter(osg::Vec3(0.0, 0.0, height / 2.0));

    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints();
    hints->setDetailRatio(10);

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(cy);
    sd->setTessellationHints(hints);
    sd->setColor(color);

    return sd;
}

osg::ref_ptr<osg::Node> cretateBoundingBox(osg::Node *node) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::BoundingBox boundingBox = osg_boundingBox(node);

    osg::notify(osg::ALWAYS) << "bouding box info" << std::endl;
    osg::notify(osg::ALWAYS) << "xMax: " << boundingBox.xMax() << std::endl;
    osg::notify(osg::ALWAYS) << "xMin: " << boundingBox.xMin() << std::endl;
    osg::notify(osg::ALWAYS) << "yMax: " << boundingBox.yMax() << std::endl;
    osg::notify(osg::ALWAYS) << "yMin: " << boundingBox.yMin() << std::endl;
    osg::notify(osg::ALWAYS) << "zMax: " << boundingBox.zMax() << std::endl;
    osg::notify(osg::ALWAYS) << "zMin: " << boundingBox.zMin() << std::endl;
    osg::notify(osg::ALWAYS) << "center: x=" << boundingBox.center().x()
                             << ",y=" << boundingBox.center().y()
                             << ",z=" << boundingBox.center().z() << std::endl;

    float length = boundingBox.xMax() - boundingBox.xMin();
    float width = boundingBox.yMax() - boundingBox.yMin();
    float height = boundingBox.zMax() - boundingBox.zMin();
    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(
        new osg::Box(boundingBox.center(), length, width, height));
    drawable->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));

    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
    stateset = drawable->getOrCreateStateSet();
    osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode(
        osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    stateset->setAttributeAndModes(polygonMode);

    osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth(3.0);
    stateset->setAttribute(linewidth);

    geode->addDrawable(drawable);
    return geode;
}

osg::ref_ptr<osg::Geometry> drawPoints(PointCloudNT::Ptr cloud, double pointSize,
                                       std::vector<osg::Vec4> pointColors) {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    for (auto &p : cloud->points) {
        if(abs(p.z) < 0.001){
            continue;
        }
        vertices->push_back({p.x, p.y, p.z});
    }
    return drawPoints(vertices, pointSize, pointColors);
}

osg::ref_ptr<osg::Geometry> drawPoints(PointCloudNT::Ptr cloud,
                                       double pointSize, osg::Vec4 color) {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    for (auto &p : cloud->points) {
        if(abs(p.z) < 0.001){
            continue;
        }
        vertices->push_back({p.x, p.y, p.z});
    }
    return drawPoints(vertices, pointSize, color);
}

osg::ref_ptr<osg::Geometry> drawPoints(PointCloudT::Ptr cloud, double pointSize,
                                       osg::Vec4 color) {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    for (const PointT &p : cloud->points) {
        if(abs(p.z) < 0.001){
            continue;
        }
        vertices->push_back({p.x, p.y, p.z});
    }
    return drawPoints(vertices, pointSize, color);
}

osg::ref_ptr<osg::Geometry> drawPoints(PointCloudT::Ptr cloud, double pointSize,
                                       std::vector<osg::Vec4> colors) {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    for (const PointT &p : cloud->points) {
        if(abs(p.z) < 0.001){
            continue;
        }
        vertices->push_back({p.x, p.y, p.z});
    }
    return drawPoints(vertices, pointSize, colors);
}

osg::ref_ptr<osg::Geometry> drawPoints(osg::ref_ptr<osg::Vec3Array> vertices,
                                       double pointSize, osg::Vec4 pointColor) {
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    for (int i = 0; i < vertices->size(); i++) {
        colors->push_back(pointColor);
    }

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setVertexArray(vertices.get());
    geometry->setColorArray(colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->getOrCreateStateSet()->setAttribute(
        new osg::Point(pointSize), osg::StateAttribute::OVERRIDE);

    osg::Vec3Array *normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
    geometry->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

    return geometry;
}


osg::ref_ptr<osg::Geometry> drawPoints(osg::ref_ptr<osg::Vec3Array> vertices,
                                       double pointSize, std::vector<osg::Vec4> pointColors) {
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    for (int i = 0; i < vertices->size(); i++) {
        colors->push_back(pointColors[i]);
    }

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setVertexArray(vertices.get());
    geometry->setColorArray(colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->getOrCreateStateSet()->setAttribute(
        new osg::Point(pointSize), osg::StateAttribute::OVERRIDE);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

    return geometry;
}

osg::ref_ptr<osg::Geode>
drawPointsWithNormal(PointCloudNT::Ptr cloud, double pointSize,
                     osg::Vec4 pointColor, double normalLength,
                     double normalWidth, osg::Vec4 normalColor) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geometry = drawPoints(cloud, pointSize, pointColor);
    geode->addDrawable(geometry.get());

    if (normalLength >= 1 && normalWidth >= 1) {
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
        int point_size = cloud->size();
        int step = std::max(1, point_size / 5000);
        for (int i = 0; i < point_size; i += step) {
            PointNT &p = cloud->points[i];
            if(abs(p.z) < 0.001){
                continue;
            }
            vertices->push_back({p.x, p.y, p.z});
            vertices->push_back({(float)(p.x + p.normal_x * normalLength),
                                 (float)(p.y + p.normal_y * normalLength),
                                 (float)(p.z + p.normal_z * normalLength)});
        }

        osg::ref_ptr<osg::Geometry> lines =
            drawLines(vertices, normalColor, normalWidth);
        geode->addDrawable(lines.get());
    }
    return geode;
}

osg::BoundingBox osg_boundingBox(osg::Node *node) {
    osg::ComputeBoundsVisitor boundVisitor;
    node->accept(boundVisitor);
    osg::BoundingBox boundingBox = boundVisitor.getBoundingBox();
    return boundingBox;
}

osg::BoundingBox osg_boundingBox(osg::ref_ptr<osg::Node> node) {
    if (node == nullptr) {
        LOG(ERROR) << "node == nullptr";
    }
    if (node) {
        osg::ComputeBoundsVisitor boundVisitor;
        node->accept(boundVisitor);
        osg::BoundingBox boundingBox = boundVisitor.getBoundingBox();
        return boundingBox;
    } else {
        LOG(ERROR) << "node valid";
    }
}

osg::ref_ptr<osg::ShapeDrawable> drawBox(osg::Vec3 center, osg::Vec3 halfLength,
                                         osg::Vec4 color) {
    osg::ref_ptr<osg::Box> box = new osg::Box;
    box->setCenter(center);
    box->setHalfLengths(halfLength);
    osg::ref_ptr<osg::ShapeDrawable> sd2 = new osg::ShapeDrawable(box.get());
    sd2->setColor(color);
    return sd2;
}

osg::ref_ptr<osgText::Text> drawText(const std::string &txt, osg::Vec3 pos,
                                     osg::Vec4 color) {
    osg::ref_ptr<osgText::Font> font = osgText::readFontFile("simhei.ttf");

    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setFont(font);
    text->setColor(color);
    text->setCharacterSize(8);
    text->setPosition(pos);
    text->setAlignment(osgText::Text::CENTER_CENTER);
    text->setAxisAlignment(osgText::Text::SCREEN);
    text->setText(txt, osgText::String::ENCODING_UTF8);

    return text;
}

osg::ref_ptr<osg::Group> osg_draw_dragger(osg::Vec3 center, float length,
                                          osg::Vec4 color) {
    osg::ref_ptr<osg::Cylinder> cylinderX =
        new osg::Cylinder(osg::Vec3(0, 0, 0), 0.5, length);
    osg::ref_ptr<osg::ShapeDrawable> lineX = new osg::ShapeDrawable(cylinderX);
    lineX->setColor(osg::Vec4(1, 0, 0, 1));

    osg::ref_ptr<osg::Cylinder> cylinderY =
        new osg::Cylinder(osg::Vec3(0, 0, 0), 0.5, length);
    osg::ref_ptr<osg::ShapeDrawable> lineY = new osg::ShapeDrawable(cylinderY);
    lineY->setColor(osg::Vec4(0, 1, 0, 1));

    osg::ref_ptr<osg::Cylinder> cylinderZ =
        new osg::Cylinder(osg::Vec3(0, 0, 0), 0.5, length);
    osg::ref_ptr<osg::ShapeDrawable> lineZ = new osg::ShapeDrawable(cylinderZ);
    lineZ->setColor(osg::Vec4(0, 0, 1, 1));

    osg::ref_ptr<osg::MatrixTransform> transY = new osg::MatrixTransform;
    transY->setMatrix(
        osg::Matrix::rotate(osg::DegreesToRadians(90.0), 1, 0, 0));
    transY->addChild(lineY.get());

    osg::ref_ptr<osg::MatrixTransform> transZ = new osg::MatrixTransform;
    transZ->setMatrix(
        osg::Matrix::rotate(osg::DegreesToRadians(90.0), 0, 1, 0));
    transZ->addChild(lineZ.get());

    osg::ref_ptr<osg::Group> geode = new osg::Group;
    geode->addChild(lineX.get());
    geode->addChild(transY.get());
    geode->addChild(transZ.get());

    osg::ref_ptr<osg::MatrixTransform> _scale2 = new osg::MatrixTransform;
    _scale2->setMatrix(
        osg::Matrix::translate(center.x(), center.y(), center.z()));
    _scale2->addChild(geode.get());

    osg::ref_ptr<osg::Group> _node = new osg::Group;

    _node->addChild(_scale2.get());

    return _node;
}

osg::ref_ptr<osg::Group> osg_draw_coor(osg::Vec3 center, float length) {
    osg::ref_ptr<osg::Geode> groupZ = new osg::Geode;
    {
        osg::ref_ptr<osg::Cylinder> cylinderZ =
            new osg::Cylinder(osg::Vec3(0, 0, 0), 0.5, length);
        osg::ref_ptr<osg::ShapeDrawable> shapeCylinderZ =
            new osg::ShapeDrawable(cylinderZ);
        shapeCylinderZ->setColor(osg::Vec4(0, 0, 1, 1));

        osg::ref_ptr<osg::Cone> coneZ =
            new osg::Cone(osg::Vec3(0.0f, 0.0f, length / 2), 2, 6);
        osg::ref_ptr<osg::ShapeDrawable> shapeConeZ =
            new osg::ShapeDrawable(coneZ.get());
        shapeConeZ->setColor(osg::Vec4(0, 0, 1, 1));

        groupZ->addChild(shapeCylinderZ.get());
        groupZ->addChild(shapeConeZ.get());
    }

    osg::ref_ptr<osg::Geode> groupY = new osg::Geode;
    {
        osg::ref_ptr<osg::Cylinder> cylinderY =
            new osg::Cylinder(osg::Vec3(0, 0, 0), 0.5, length);
        osg::ref_ptr<osg::ShapeDrawable> shapeCylinderY =
            new osg::ShapeDrawable(cylinderY);
        shapeCylinderY->setColor(osg::Vec4(0, 1, 0, 1));

        osg::ref_ptr<osg::Cone> coneY =
            new osg::Cone(osg::Vec3(0.0f, 0.0f, length / 2), 2, 6);
        osg::ref_ptr<osg::ShapeDrawable> shapeConeY =
            new osg::ShapeDrawable(coneY.get());
        shapeConeY->setColor(osg::Vec4(0, 1, 0, 1));

        groupY->addChild(shapeCylinderY.get());
        groupY->addChild(shapeConeY.get());
    }

    osg::ref_ptr<osg::Geode> groupX = new osg::Geode;
    {
        osg::ref_ptr<osg::Cylinder> cylinderX =
            new osg::Cylinder(osg::Vec3(0, 0, 0), 0.5, length);
        osg::ref_ptr<osg::ShapeDrawable> shapeCylinderX =
            new osg::ShapeDrawable(cylinderX);
        shapeCylinderX->setColor(osg::Vec4(1, 0, 0, 1));

        osg::ref_ptr<osg::Cone> coneX =
            new osg::Cone(osg::Vec3(0.0f, 0.0f, length / 2), 2, 6);
        osg::ref_ptr<osg::ShapeDrawable> shapeConeX =
            new osg::ShapeDrawable(coneX.get());
        shapeConeX->setColor(osg::Vec4(1, 0, 0, 1));

        groupX->addChild(shapeCylinderX.get());
        groupX->addChild(shapeConeX.get());
    }

    osg::ref_ptr<osg::MatrixTransform> transZ = new osg::MatrixTransform;
    transZ->setMatrix(osg::Matrix::translate(0, 0, length / 2));
    transZ->addChild(groupZ.get());

    osg::ref_ptr<osg::MatrixTransform> transY = new osg::MatrixTransform;
    transY->setMatrix(
        osg::Matrix::rotate(-osg::DegreesToRadians(90.0), 1, 0, 0) *
        osg::Matrix::translate(0, length / 2, 0));
    transY->addChild(groupY.get());

    osg::ref_ptr<osg::MatrixTransform> transX = new osg::MatrixTransform;
    transX->setMatrix(
        osg::Matrix::rotate(osg::DegreesToRadians(90.0), 0, 1, 0) *
        osg::Matrix::translate(length / 2, 0, 0));
    transX->addChild(groupX.get());

    osg::ref_ptr<osg::Group> geode = new osg::Group;
    geode->addChild(transZ.get());
    geode->addChild(transY.get());
    geode->addChild(transX.get());

    osg::ref_ptr<osg::MatrixTransform> _scale2 = new osg::MatrixTransform;
    _scale2->setMatrix(
        osg::Matrix::translate(center.x(), center.y(), center.z()));
    _scale2->addChild(geode.get());

    osg::ref_ptr<osg::Group> _node = new osg::Group;

    _node->addChild(_scale2.get());

    return _node;
}

void osg_readPCD(std::string filePath, osg::ref_ptr<osg::Vec3Array> &coords,
                 osg::ref_ptr<osg::Vec4Array> &color, int &num, int scale) {

    std::ifstream fileIn(filePath.c_str(), std::ios::in);
    std::string line;
    std::string token;
    float xyz[3], fa = 0.6, fb = 0.13, fc = 0.13;
    for (int i = 0; i < 10; i++)
        getline(fileIn, line);
    while (!fileIn.eof()) {
        getline(fileIn, line);
        std::stringstream sstr(line);
        for (int i = 0; i < 3; i++) {
            getline(sstr, token, ' ');
            xyz[i] = atof(token.c_str()) / scale;
        }
        coords->push_back(osg::Vec3(xyz[0], xyz[1], xyz[2]));
        color->push_back(osg::Vec4(fa, fb, fc, 1.0f));
        num++;
    }
}

osg::ref_ptr<osg::Node> osg_loadFile(std::string fileName) {
    osg::ref_ptr<osgDB::Options> options = new osgDB::Options("noRotation");
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(fileName, options);

#if !defined(OSG_GLES2_AVAILABLE)
    node->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
#endif
    return node;
}

void osg_save(const osg::Node &node, std::string filePath) {
    osgDB::writeNodeFile(node, filePath);
}

void osg_QuatToHPR(osg::Quat q, double &heading, double &pitch, double &roll) {
    double tmp[3];
    quaternion2Euler(q, tmp, RotSeq::zyx);
    heading = tmp[0];
    pitch = tmp[1];
    roll = tmp[2];
}

osg::Quat osg_HPRToQuat(double heading, double pitch, double roll) {
    osg::Quat q(roll, osg::Vec3d(0.0, 1.0, 0.0), pitch,
                osg::Vec3d(1.0, 0.0, 0.0), heading, osg::Vec3d(0.0, 0.0, 1.0));
    return q;
}

void twoaxisrot(double r11, double r12, double r21, double r31, double r32,
                double res[]) {
    res[0] = atan2(r11, r12);
    res[1] = acos(r21);
    res[2] = atan2(r31, r32);
}

void threeaxisrot(double r11, double r12, double r21, double r31, double r32,
                  double res[]) {
    res[0] = atan2(r31, r32);
    res[1] = asin(r21);
    res[2] = atan2(r11, r12);
}

void quaternion2Euler(osg::Quat q, double res[], RotSeq rotSeq) {
    switch (rotSeq) {
    case zyx:
        threeaxisrot(
            2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
            -2 * (q.x() * q.z() - q.w() * q.y()),
            2 * (q.y() * q.z() + q.w() * q.x()),
            q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(), res);
        break;

    case zyz:
        twoaxisrot(2 * (q.y() * q.z() - q.w() * q.x()),
                   2 * (q.x() * q.z() + q.w() * q.y()),
                   q.w() * q.w() - q.x() * q.x() - q.y() * q.y() +
                       q.z() * q.z(),
                   2 * (q.y() * q.z() + q.w() * q.x()),
                   -2 * (q.x() * q.z() - q.w() * q.y()), res);
        break;

    case zxy:
        threeaxisrot(
            -2 * (q.x() * q.y() - q.w() * q.z()),
            q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
            2 * (q.y() * q.z() + q.w() * q.x()),
            -2 * (q.x() * q.z() - q.w() * q.y()),
            q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(), res);
        break;

    case zxz:
        twoaxisrot(2 * (q.x() * q.z() + q.w() * q.y()),
                   -2 * (q.y() * q.z() - q.w() * q.x()),
                   q.w() * q.w() - q.x() * q.x() - q.y() * q.y() +
                       q.z() * q.z(),
                   2 * (q.x() * q.z() - q.w() * q.y()),
                   2 * (q.y() * q.z() + q.w() * q.x()), res);
        break;

    case yxz:
        threeaxisrot(
            2 * (q.x() * q.z() + q.w() * q.y()),
            q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
            -2 * (q.y() * q.z() - q.w() * q.x()),
            2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(), res);
        break;

    case yxy:
        twoaxisrot(2 * (q.x() * q.y() - q.w() * q.z()),
                   2 * (q.y() * q.z() + q.w() * q.x()),
                   q.w() * q.w() - q.x() * q.x() + q.y() * q.y() -
                       q.z() * q.z(),
                   2 * (q.x() * q.y() + q.w() * q.z()),
                   -2 * (q.y() * q.z() - q.w() * q.x()), res);
        break;

    case yzx:
        threeaxisrot(
            -2 * (q.x() * q.z() - q.w() * q.y()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
            2 * (q.x() * q.y() + q.w() * q.z()),
            -2 * (q.y() * q.z() - q.w() * q.x()),
            q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(), res);
        break;

    case yzy:
        twoaxisrot(2 * (q.y() * q.z() + q.w() * q.x()),
                   -2 * (q.x() * q.y() - q.w() * q.z()),
                   q.w() * q.w() - q.x() * q.x() + q.y() * q.y() -
                       q.z() * q.z(),
                   2 * (q.y() * q.z() - q.w() * q.x()),
                   2 * (q.x() * q.y() + q.w() * q.z()), res);
        break;

    case xyz:
        threeaxisrot(
            -2 * (q.y() * q.z() - q.w() * q.x()),
            q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
            2 * (q.x() * q.z() + q.w() * q.y()),
            -2 * (q.x() * q.y() - q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(), res);
        break;

    case xyx:
        twoaxisrot(2 * (q.x() * q.y() + q.w() * q.z()),
                   -2 * (q.x() * q.z() - q.w() * q.y()),
                   q.w() * q.w() + q.x() * q.x() - q.y() * q.y() -
                       q.z() * q.z(),
                   2 * (q.x() * q.y() - q.w() * q.z()),
                   2 * (q.x() * q.z() + q.w() * q.y()), res);
        break;

    case xzy:
        threeaxisrot(
            2 * (q.y() * q.z() + q.w() * q.x()),
            q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
            -2 * (q.x() * q.y() - q.w() * q.z()),
            2 * (q.x() * q.z() + q.w() * q.y()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(), res);
        break;

    case xzx:
        twoaxisrot(2 * (q.x() * q.z() - q.w() * q.y()),
                   2 * (q.x() * q.y() + q.w() * q.z()),
                   q.w() * q.w() + q.x() * q.x() - q.y() * q.y() -
                       q.z() * q.z(),
                   2 * (q.x() * q.z() + q.w() * q.y()),
                   -2 * (q.x() * q.y() - q.w() * q.z()), res);
        break;
    default:
        std::cout << "Unknown rotation sequence" << std::endl;
        break;
    }
}

} // namespace osgplus
} // namespace pcle