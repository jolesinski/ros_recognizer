#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros_recognizer/preprocessing/local_3d_describer.h>

sensor_msgs::PointCloud2Ptr loadPCD(const std::string& pcd_path)
{
  system("pwd");
  pcl::PCLPointCloud2 pcl_cloud;
  if (pcl::io::loadPCDFile(pcd_path, pcl_cloud) == -1 || pcl_cloud.width == 0)
    throw std::runtime_error("Failed to load from " + pcd_path);

  sensor_msgs::PointCloud2Ptr ros_cloud(new sensor_msgs::PointCloud2);
  pcl_conversions::fromPCL(pcl_cloud, *ros_cloud);
  return ros_cloud;
}

TEST (Preprocessing, modelLocal3dDescriber)
{
  ros_recognizer::Local3dDescriber describer;

  auto model_cloud = loadPCD("data/model.pcd");

  ASSERT_NE(model_cloud, nullptr);

  auto model_description = describer.describe(model_cloud);

  ASSERT_NE(model_description.input_, nullptr);
  ASSERT_NE(model_description.normals_, nullptr);
  ASSERT_NE(model_description.keypoints_, nullptr);
  ASSERT_NE(model_description.ref_frames_, nullptr);
  ASSERT_NE(model_description.descriptors_, nullptr);

  EXPECT_GT(model_description.input_resolution_, .0);
  EXPECT_NE(model_description.input_->size(), 0);
  EXPECT_NE(model_description.normals_->size(), 0);
  EXPECT_NE(model_description.keypoints_->size(), 0);
  EXPECT_NE(model_description.ref_frames_->size(), 0);
  EXPECT_NE(model_description.descriptors_->size(), 0);

  EXPECT_GE(model_description.input_->size(),
            model_description.keypoints_->size());
}

TEST (Preprocessing, sceneLocal3dDescriber)
{
  ros_recognizer::Local3dDescriber describer;

  auto scene_cloud = loadPCD("data/scene.pcd");

  ASSERT_NE(scene_cloud, nullptr);

  auto scene_description = describer.describe(scene_cloud);

  ASSERT_NE(scene_description.input_, nullptr);
  ASSERT_NE(scene_description.normals_, nullptr);
  ASSERT_NE(scene_description.keypoints_, nullptr);
  ASSERT_NE(scene_description.ref_frames_, nullptr);
  ASSERT_NE(scene_description.descriptors_, nullptr);

  EXPECT_GT(scene_description.input_resolution_, .0);
  EXPECT_NE(scene_description.input_->size(), 0);
  EXPECT_NE(scene_description.normals_->size(), 0);
  EXPECT_NE(scene_description.keypoints_->size(), 0);
  EXPECT_NE(scene_description.ref_frames_->size(), 0);
  EXPECT_NE(scene_description.descriptors_->size(), 0);

  EXPECT_GE(scene_description.input_->size(),
            scene_description.keypoints_->size());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}