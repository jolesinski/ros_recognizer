
#ifndef ROS_RECOGNIZER_VISUALIZER_H
#define ROS_RECOGNIZER_VISUALIZER_H

#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/preprocessing/local_3d_description.h>
#include <ros_recognizer/VisualizerConfig.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace ros_recognizer
{

class Visualizer : public Reconfigurable<VisualizerConfig>
{
public:
  static constexpr auto SPIN_MS = 100;

  Visualizer();

  void setModel(const Local3dDescription& model);
  void setScene(const Local3dDescription& scene);

  void render();

private:
  Local3dDescription model_, scene_;
  pcl::visualization::PCLVisualizer vis_;

  void showDescription(const ros_recognizer::Local3dDescription& descr,
                       const std::string& id_prefix);

  void showInput(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                 const std::string &id);
  void showNormals(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input,
                   const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                   const std::string& id);
  void showKeypoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                     const std::string &id);

  Local3dDescription shiftModel(const Local3dDescription& model);
};

}

#endif //ROS_RECOGNIZER_VISUALIZER_H
