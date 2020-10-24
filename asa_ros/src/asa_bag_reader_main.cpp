#include <gflags/gflags.h>
#include <glog/logging.h>

#include "asa_ros/asa_bag_reader.h"

DEFINE_string(bag, "", "Path to the ROS bag file.");
DEFINE_bool(create, false,
            "Whether to create a new anchor. This or query should be set. "
            "Otherwise won't do much.");
DEFINE_string(query, "", "The anchor ID of the spatial anchor to query.");
DEFINE_string(account_domain, "mixedreality.azure.com",
              "Which account domain to use for the Azure Spatial Anchors "
              "account. Can be found in Azure Portal.");
DEFINE_string(account_id, "", "ASA account ID.");
DEFINE_string(account_key, "", "ASA account key.");

// Bag params.
DEFINE_string(image_topic, "/camera/fisheye1_rect/image",
              "ROS image topic to subscribe to.");
DEFINE_string(camera_info_topic, "/camera/fisheye1_rect/camera_info",
              "ROS camera info topic to subscribe to.");
DEFINE_bool(use_tf, true,
            "Whether to use TF (true) or transform/pose topics (false).");
DEFINE_string(world_frame_id, "camera_odom_frame",
              "TF frame for the odometry/world/global coordinate frame.");
DEFINE_string(camera_frame_id, "camera_fisheye1_optical_frame",
              "TF frame for the camera optical frame, z into image plane.");
DEFINE_string(transform_topic, "",
              "Transform topic to subscribe to if --use_tf is false.");
DEFINE_string(pose_topic, "",
              "Pose topic to subscribe to if --use_tf is false.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  asa_ros::AsaBagReaderConfig bag_config;
  bag_config.image_topic = FLAGS_image_topic;
  bag_config.camera_info_topic = FLAGS_camera_info_topic;
  bag_config.use_tf = FLAGS_use_tf;
  bag_config.world_frame_id = FLAGS_world_frame_id;
  bag_config.camera_frame_id = FLAGS_camera_frame_id;
  bag_config.transform_topic = FLAGS_transform_topic;
  bag_config.pose_topic = FLAGS_pose_topic;

  asa_ros::AsaRosConfig asa_config;
  asa_config.account_id = FLAGS_account_id;
  asa_config.account_key = FLAGS_account_key;
  asa_config.account_domain = FLAGS_account_domain;

  asa_ros::AsaBagReader bag_reader(bag_config, asa_config);

  std::string bag_path = FLAGS_bag;
  if (bag_path.empty()) {
    std::cout << "No bag file specified. Usage: \"asa_bag_reader --bag "
                 "path_to_bag\""
              << std::endl;
    return 1;
  }

  LOG(INFO) << "[ASA Bag Reader] Now running.";
  LOG(INFO) << "[ASA Bag Reader] Loading bag: " << bag_path;

  bag_reader.loadBag(bag_path);

  LOG(INFO) << "[ASA Bag Reader] Loaded the whole bag!";

  if (FLAGS_create) {
    Eigen::Affine3d anchor_in_world_frame = Eigen::Affine3d::Identity();
    std::string anchor_id;
    bag_reader.createAnchor(anchor_in_world_frame, &anchor_id);

    LOG(INFO) << "[ASA Bag Reader] Created an anchor! " << anchor_id;
  }
  if (!FLAGS_query.empty()) {
    Eigen::Affine3d anchor_in_world_frame = Eigen::Affine3d::Identity();
    if (bag_reader.queryAnchor(FLAGS_query, &anchor_in_world_frame)) {
      LOG(INFO) << "[ASA Bag Reader] Queried an anchor! " << FLAGS_query
                << std::endl
                << anchor_in_world_frame.matrix();
    } else {
      LOG(INFO) << "[ASA Bag Reader] Could not find anchor! " << FLAGS_query;
    }
  }
  return 0;
}
