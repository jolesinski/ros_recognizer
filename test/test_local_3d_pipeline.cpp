#define ROS_RECOGNIZER_OUTSIDE_ROS_NODE

#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros_recognizer/matching/local_matcher.h>
#include <ros_recognizer/preprocessing/local_3d_describer.h>
#include <ros_recognizer/verification/verifier.h>

struct Local3dPipeline : testing::Test {
  sensor_msgs::PointCloud2Ptr model_input;
  sensor_msgs::PointCloud2Ptr scene_with_single_model_input;
  std::shared_ptr<Eigen::Matrix4f> ground_truth_pose;

  Local3dPipeline() {
    model_input = loadPCD("data/model.pcd");
    scene_with_single_model_input = loadPCD("data/scene_with_single_instance.pcd");
    ground_truth_pose = loadGroundTruth("data/ground_truth_pose.pcd");
  }

  sensor_msgs::PointCloud2Ptr loadPCD(const std::string& pcd_path)
  {
    pcl::PCLPointCloud2 pcl_cloud;
    if (pcl::io::loadPCDFile(pcd_path, pcl_cloud) == -1 || pcl_cloud.width == 0)
      throw std::runtime_error("Failed to load from " + pcd_path);

    sensor_msgs::PointCloud2Ptr ros_cloud(new sensor_msgs::PointCloud2);
    pcl_conversions::fromPCL(pcl_cloud, *ros_cloud);
    return ros_cloud;
  }

  std::shared_ptr<Eigen::Matrix4f> loadGroundTruth(const std::string& pose_path)
  {
    std::shared_ptr<Eigen::Matrix4f> pose(new Eigen::Matrix4f);
    std::ifstream file(pose_path);
    float pose_elem;
    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    for (int idx = 0; file >> pose_elem; idx++)
    {
      auto row = idx / 4;
      auto col = idx % 4;

      if (row == 3)
        break;

      if (col == 3)
        translation[row] = pose_elem;
      else
        rotation(row, col) = pose_elem;
    }
    //Affine inverse
    pose->block(0,0,3,3) << rotation.transpose();
    pose->block(0,3,3,1) << -rotation.transpose()*translation;

    return pose;
  }

};

TEST_F (Local3dPipeline, modelLocal3dDescriber)
{
  ros_recognizer::Local3dDescriber describer;
  auto model_description = describer.describe(*model_input);

  ASSERT_NE(model_description.input_, nullptr);
  ASSERT_NE(model_description.normals_, nullptr);
  ASSERT_NE(model_description.keypoints_, nullptr);
  ASSERT_NE(model_description.ref_frames_, nullptr);
  ASSERT_NE(model_description.descriptors_, nullptr);

  //EXPECT_GT(model_description.input_resolution_, .0);
  EXPECT_NE(model_description.input_->size(), 0);
  EXPECT_NE(model_description.normals_->size(), 0);
  EXPECT_NE(model_description.keypoints_->size(), 0);
  EXPECT_NE(model_description.ref_frames_->size(), 0);
  EXPECT_NE(model_description.descriptors_->size(), 0);

  EXPECT_GE(model_description.input_->size(),
            model_description.keypoints_->size());
  EXPECT_EQ(model_description.ref_frames_->size(),
            model_description.descriptors_->size());
}

TEST_F (Local3dPipeline, sceneLocal3dDescriber)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  ros_recognizer::Local3dDescriber describer;
  auto scene_description = describer.describe(*scene_with_single_model_input);

  ASSERT_NE(scene_description.input_, nullptr);
  ASSERT_NE(scene_description.normals_, nullptr);
  ASSERT_NE(scene_description.keypoints_, nullptr);
  ASSERT_NE(scene_description.ref_frames_, nullptr);
  ASSERT_NE(scene_description.descriptors_, nullptr);

  //EXPECT_GT(scene_description.input_resolution_, .0);
  EXPECT_NE(scene_description.input_->size(), 0);
  EXPECT_NE(scene_description.normals_->size(), 0);
  EXPECT_NE(scene_description.keypoints_->size(), 0);
  EXPECT_NE(scene_description.ref_frames_->size(), 0);
  EXPECT_NE(scene_description.descriptors_->size(), 0);

  EXPECT_GE(scene_description.input_->size(),
            scene_description.keypoints_->size());
  EXPECT_EQ(scene_description.keypoints_->size(),
            scene_description.descriptors_->size());
  EXPECT_EQ(scene_description.ref_frames_->size(),
            scene_description.descriptors_->size());
}

TEST_F (Local3dPipeline, localMatcher)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  ros_recognizer::Local3dDescriber describer;
  auto model_description = describer.describe(*model_input);
  auto scene_description = describer.describe(*scene_with_single_model_input);

  ros_recognizer::LocalMatcher matcher;
  auto hypotheses = matcher.match(model_description, scene_description);
  EXPECT_GT(hypotheses.size(), 0);
  std::cout << "Hypotheses: " << hypotheses.size() << std::endl;
}

TEST_F (Local3dPipeline, singleInstanceVerification)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  ros_recognizer::Local3dDescriber describer;
  auto model_description = describer.describe(*model_input);
  auto scene_description = describer.describe(*scene_with_single_model_input);

  ros_recognizer::LocalMatcher matcher;
  auto hypotheses = matcher.match(model_description, scene_description);

  ros_recognizer::Verifier verifier;
  hypotheses = verifier.verify(hypotheses, scene_description.input_);

  auto idx = 0u;
  for(const auto& hyp : hypotheses)
    if (hyp.is_valid_)
      std::cout << "Instance " << idx++ << " is GOOD! <---" << std::endl;
    else
      std::cout << "Instance " << idx++ << " is bad!" << std::endl;

  ASSERT_EQ(hypotheses.size(), 1);

  float translation_error = (ground_truth_pose->block(0,3,3,1)
                             - hypotheses.front().pose_.block(0,3,3,1)).norm();
  EXPECT_LE(translation_error, 0.01);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}