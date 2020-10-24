#include "asa_ros/asa_interface.h"

namespace asa_ros {

struct AsaBagReaderConfig {
  // Required:
  // TODO: support for multiple image topics? This would be fairly easy to do.
  std::string image_topic;
  std::string camera_info_topic;
  // For topic-based pose input:
  std::string transform_topic;
  std::string pose_topic;
  // For TF input:
  bool use_tf = true;
  std::string world_frame_id = "world";
  std::string camera_frame_id = "camera";
};

class AsaBagReader {
 public:
  // Sets up the bag reader.
  AsaBagReader(const AsaBagReaderConfig& bag_config,
                  const AsaRosConfig& asa_config);

  // Reads the entire bag into ASA.
  // This should be done before querying or creating.
  void loadBag(const std::string& bag_path);

  // Two possible options:
  // Create an anchor at a given pose relative TO THE WORLD COORDINATE FRAME.
  bool createAnchor(const Eigen::Affine3d& anchor_in_world_frame, std::string* anchor_id);

  bool queryAnchor(const std::string& anchor_id, Eigen::Affine3d* anchor_in_world_frame);

 private:
  // Cache the configs:
  AsaBagReaderConfig bag_config_;

  // Create an ASA interface object.
  AzureSpatialAnchorsInterface interface_;
};

}  // namespace asa_ros
