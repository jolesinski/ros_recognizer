
#ifndef ROS_RECOGNIZER_VISUALIZER_H
#define ROS_RECOGNIZER_VISUALIZER_H

#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/preprocessing/local_3d_description.h>
#include <ros_recognizer/matching/hypotheses.h>
#include <ros_recognizer/VisualizerConfig.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace ros_recognizer
{

class Visualizer : public Reconfigurable<VisualizerConfig>
{
public:
  static constexpr auto SPIN_MS = 100;

  void setModel(const Local3dDescription& model);
  void setScene(const Local3dDescription& scene);
  void setCorrespondences(const pcl::Correspondences& corrs);
  void setClusters(const std::vector<pcl::Correspondences>& clusters);
  void setHypotheses(const Hypotheses& hyps, bool valid);

  void render(bool force_redrawal);

private:
  Local3dDescription model_, scene_;
  pcl::Correspondences corrs_;
  std::vector<pcl::Correspondences> clusters_;
  Hypotheses true_hypotheses_, false_hypotheses_;
  bool needs_redrawal_ = false;
  std::unique_ptr<pcl::visualization::PCLVisualizer> vis_;

  // Delayed vis initialization, nodelet will mess this up in onInit
  void initVis();

  void showDescription(const ros_recognizer::Local3dDescription& descr,
                       const std::string& id_prefix);

  void showInput(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                 const std::string &id);
  void showNormals(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input,
                   const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                   const std::string& id);
  void showKeypoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                     const std::string &id);
  void showCorrespondences();
  void showHypotheses(const Hypotheses& hypotheses, bool valid);
  void clearResults();

  Local3dDescription shiftModel(const Local3dDescription& model);
};

}

#endif //ROS_RECOGNIZER_VISUALIZER_H
