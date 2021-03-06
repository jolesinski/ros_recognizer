
#ifndef ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H
#define ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H

#include <sensor_msgs/PointCloud2.h>

#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/description/local_3d_description.h>
#include <ros_recognizer/DescriberConfig.h>

namespace ros_recognizer
{

class Local3dDescriber : public Reconfigurable<DescriberConfig>
{
public:
  Local3dDescription operator()(const sensor_msgs::PointCloud2& cloud_msg);

private:
  void computeNormals(Local3dDescription& data);
  void computeKeypoints(Local3dDescription& data);
  void computeDescriptors(Local3dDescription& data);
  void computeRefFrames(Local3dDescription& data);
  void computeResolution(Local3dDescription& data);
};

}

#endif //ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H
