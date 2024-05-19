#ifndef PCLE_COMMON_TYPE_H
#define PCLE_COMMON_TYPE_H

#define PCLE_DEVICE_FUNC

#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <pcl/types.h>

#include <opencv2/core/core.hpp>

#include <set>
#include <vector>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB RGBPointT;
typedef pcl::Normal NormalT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<RGBPointT> RGBPointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

typedef pcl::Normal PointN;
typedef pcl::PointCloud<NormalT> PointCloudN;

class Transform {
 public:
  using M_     = Eigen::Matrix4f;
  using R_     = Eigen::Matrix3f;
  using T_     = Eigen::Vector3f;
  using A_     = Eigen::Affine3f;
  using Q_     = Eigen::Quaternionf;
  using Point_ = Eigen::Vector3f;

  M_ mat;

  PCLE_DEVICE_FUNC inline
  static Transform zero()
  {
    Transform pose;
    pose.mat << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    return pose;
  }

  PCLE_DEVICE_FUNC inline
  static Transform from(const M_& m)
  {
    Transform pose(m);
    return pose;
  }

  PCLE_DEVICE_FUNC inline
  static Transform fromMatrix(const Eigen::Matrix4d& m)
  {
    Transform pose(m.cast<float>());
    return pose;
  }

  PCLE_DEVICE_FUNC inline
  static Transform fromMatrix(const Eigen::Matrix4f& m)
  {
    Transform pose(m);
    return pose;
  }

  PCLE_DEVICE_FUNC inline
  static Transform from(const R_& r, const T_& t)
  {
    Transform pose;
    pose.mat << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    pose.mat.block<3, 3>(0, 0) = r;
    pose.mat.block<3, 1>(0, 3) = t;
    return pose;
  }

  PCLE_DEVICE_FUNC inline
  static Transform from(const float* r, const float* t)
  {
    Transform pose;
    pose.mat << r[0], r[1], r[2], t[0], 
                r[3], r[4], r[5], t[1], 
                r[6], r[7], r[8], t[2], 
                0, 0, 0, 1;
    return pose;
  }

  PCLE_DEVICE_FUNC inline
  Transform() { mat << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; }

  PCLE_DEVICE_FUNC inline
  Transform(const A_& a) { mat = a.matrix(); }

  PCLE_DEVICE_FUNC inline Transform(const M_& m) : mat(m) {}

  PCLE_DEVICE_FUNC inline
  Transform(const Eigen::AngleAxisf& ax)
  {
    mat.block<3, 3>(0, 0) = ax.matrix();
    mat.block<3, 1>(0, 3) << 0, 0, 0;
    mat.block<1, 4>(3, 0) << 0, 0, 0, 1;
  }

  PCLE_DEVICE_FUNC inline
  Transform(float x, float y, float z, float rx, float ry, float rz)
  {
    Eigen::Vector3f r_vec(rx, ry, rz);
    Eigen::AngleAxisf r(r_vec.norm(), r_vec.normalized());

    mat.block<3, 3>(0, 0) = r.matrix();
    mat.block<3, 1>(0, 3) << x, y, z;
    mat.block<1, 4>(3, 0) << 0, 0, 0, 1;
  }

  PCLE_DEVICE_FUNC inline
  Transform(const R_& r, const T_& t)
  {
    mat.block<3, 3>(0, 0) = r;
    mat.block<3, 1>(0, 3) = t;
    mat.block<1, 4>(3, 0) << 0, 0, 0, 1;
  }

  PCLE_DEVICE_FUNC inline 
  Transform(const Eigen::Quaternionf& quaternion, const T_& t)
  {
    R_ r = quaternion.toRotationMatrix();
    setRotation(r);
    setTranslation(t);
  }

  PCLE_DEVICE_FUNC inline 
  void setRotation(const R_& r) { mat.block<3, 3>(0, 0) = r; }

  PCLE_DEVICE_FUNC inline 
  void setRotationD(const cv::Mat& r) { 
    mat.block<3, 3>(0, 0) << 
      r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2),
      r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2),
      r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2); 
  }
  
  PCLE_DEVICE_FUNC inline 
  void setRotation(const std::vector<float>& r) { 
    mat.block<3, 3>(0, 0) <<  r[0], r[1], r[2], 
                              r[3], r[4], r[5],
                              r[6], r[7], r[8]; 
  }

  PCLE_DEVICE_FUNC inline void setRotation(Eigen::AngleAxisf angleAxis)
  {
    R_ r = angleAxis.toRotationMatrix();
    setRotation(r);
  }

  PCLE_DEVICE_FUNC inline void setRotation(Eigen::Quaternionf quaternion)
  {
    R_ r = quaternion.toRotationMatrix();
    setRotation(r);
  }

  PCLE_DEVICE_FUNC inline void setTranslation(const std::vector<float>& r) { 
    mat.block<3, 1>(0, 3) <<  r[0], r[1], r[2]; 
  }
  PCLE_DEVICE_FUNC inline void setTranslation(float tx, float ty, float tz) { 
    mat.block<3, 1>(0, 3) <<  tx, ty, tz; 
  }

  PCLE_DEVICE_FUNC inline void setTranslation(const T_& t) { mat.block<3, 1>(0, 3) = t; }

  PCLE_DEVICE_FUNC inline void init()
  {
    mat << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  }

  PCLE_DEVICE_FUNC inline void init(const R_& r, T_& t)
  {
    mat.block<3, 3>(0, 0) = r;
    mat.block<3, 1>(0, 3) = t;
    mat.block<1, 4>(3, 0) << 0, 0, 0, 1;
  }

  PCLE_DEVICE_FUNC inline void init(const std::vector<float>& r, const std::vector<float>& t)
  {
    setRotation(r);
    setTranslation(t);
    mat.block<1, 4>(3, 0) << 0, 0, 0, 1;
  }


  PCLE_DEVICE_FUNC inline const A_ affine3()
  {
    A_ a;
    a.matrix() = mat;
    return a;
  }

  PCLE_DEVICE_FUNC inline const M_& matrix() { return mat; }

  PCLE_DEVICE_FUNC inline R_ rotation() { return mat.block<3, 3>(0, 0); }

  PCLE_DEVICE_FUNC inline
  Eigen::Vector3f axis() {
    Eigen::AngleAxisf aa(rotation());
    return aa.axis();
  }

  PCLE_DEVICE_FUNC inline Transform inverse()
  {
    return affine3().inverse();
  }

  PCLE_DEVICE_FUNC inline Eigen::Quaternionf quaternion() { return Eigen::Quaternionf(this->rotation()); }

  PCLE_DEVICE_FUNC inline T_ translation() { return mat.block<3, 1>(0, 3); }

  PCLE_DEVICE_FUNC inline float& operator()(int i, int j) { return mat(i, j); }

  PCLE_DEVICE_FUNC inline Transform operator*(const Transform& pose) { return {mat * pose.mat}; }

  PCLE_DEVICE_FUNC inline Transform operator*(const Eigen::Affine3f& a) { return {mat * a.matrix()}; }

  PCLE_DEVICE_FUNC inline Transform operator*(const Eigen::AngleAxisf& a) { return {mat * (Transform(a).mat)}; }

  PCLE_DEVICE_FUNC inline Point_ operator*(const Point_& point) { return affine3() * point; }

  PCLE_DEVICE_FUNC inline void print()
  {
    printf("%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n",
           mat(0, 0),
           mat(0, 1),
           mat(0, 2),
           mat(0, 3),
           mat(1, 0),
           mat(1, 1),
           mat(1, 2),
           mat(1, 3),
           mat(2, 0),
           mat(2, 1),
           mat(2, 2),
           mat(2, 3),
           mat(3, 0),
           mat(3, 1),
           mat(3, 2),
           mat(3, 3));
  }

};

class Cluster {
public:
  Transform transform;

  Eigen::Vector3f translation_sum;
  Eigen::Vector4f rotation_sum;
  float votes;

  PCLE_DEVICE_FUNC inline
  Cluster()
  {
    votes           = 0;
    translation_sum = {0.0, 0.0, 0.0};
    rotation_sum    = {0.0, 0.0, 0.0, 0.0};
  }
};


// namespace pcle {

// using Correspondences    = std::vector<std::pair<int, int>>;
// using CorrespondencesPtr = std::shared_ptr<Correspondences>;
using BoundingBox = std::pair<PointT, PointT>;

// }  // namespace pcle


#endif  // PCLE_COMMON_TYPE_H
