
#ifndef ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H
#define ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/DescriberConfig.h>
#include <sensor_msgs/PointCloud2.h>

namespace ros_recognizer {

struct Local3dDescription
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  pcl::PointCloud<pcl::SHOT1344>::Ptr descriptors_;
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr ref_frames_;

  boost::shared_ptr<Eigen::Matrix4f> pose_;
  double input_resolution_ = .0;

  Local3dDescription() { reset(); }
  void reset() {
    input_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    keypoints_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    normals_.reset(new pcl::PointCloud<pcl::Normal>());
    descriptors_.reset(new pcl::PointCloud<pcl::SHOT1344>());
    ref_frames_.reset(new pcl::PointCloud<pcl::ReferenceFrame>());
    pose_.reset(new Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
    input_resolution_ = .0;
  }
};

class Local3dDescriber : public Reconfigurable<ros_recognizer::DescriberConfig>
{
public:
  Local3dDescription describe(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
  void computeNormals(Local3dDescription& data);
  void computeKeypoints(Local3dDescription& data);
  void computeDescriptors(Local3dDescription& data);
  void computeRefFrames(Local3dDescription& data);
  void computeResolution(Local3dDescription& data);
};

}

#endif //ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H
