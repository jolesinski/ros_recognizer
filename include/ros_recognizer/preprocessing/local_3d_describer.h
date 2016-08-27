
#ifndef ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H
#define ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H

#include <ros_recognizer/DescriberConfig.h>
#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/preprocessing/local_3d_description.h>
#include <sensor_msgs/PointCloud2.h>

namespace ros_recognizer
{

class Local3dDescriber : public Reconfigurable<DescriberConfig>
{
public:
  Local3dDescription describe(const sensor_msgs::PointCloud2& cloud_msg);

private:
  void computeNormals(Local3dDescription& data);
  void computeKeypoints(Local3dDescription& data);
  void computeDescriptors(Local3dDescription& data);
  void computeRefFrames(Local3dDescription& data);
  void computeResolution(Local3dDescription& data);
};

}

#endif //ROS_RECOGNIZER_LOCAL_3D_PREPROCESSOR_H
