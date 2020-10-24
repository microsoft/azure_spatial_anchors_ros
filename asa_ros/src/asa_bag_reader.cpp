#include <glog/logging.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "asa_ros/asa_bag_reader.h"
#include "asa_ros/asa_interface.h"

namespace asa_ros {

AsaBagReader::AsaBagReader(const AsaBagReaderConfig& bag_config,
                           const AsaRosConfig& asa_config)
    : bag_config_(bag_config), interface_(asa_config) {}

// Reads the entire bag into ASA.
// This should be done before querying or creating.
void AsaBagReader::loadBag(const std::string& bag_path) {
  const std::string kDefaultAuthority = "default_authority";
  const std::string kTfStaticTopic = "/tf_static";
  const std::string kTfTopic = "/tf";

  // Start the ASA session.
  interface_.start();

  // Load the bag.
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  if (!bag.isOpen()) {
    LOG(ERROR) << "Couldn't open bag. :(";
    return;
  }

  std::vector<std::string> topics;
  // We should only have one transform source.
  if (bag_config_.use_tf) {
    topics.push_back(kTfTopic);
    topics.push_back(kTfStaticTopic);
  } else {
    if (!bag_config_.transform_topic.empty()) {
      topics.push_back(bag_config_.transform_topic);
    } else if (!bag_config_.pose_topic.empty()) {
      topics.push_back(bag_config_.pose_topic);
    }
  }

  // We first iterate over the bag to fill a full map of transforms over time.
  // For consistency, we just populate a tf::Buffer with the whole bag.
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // This sets a default cache time of 5000 seconds.
  tf2_ros::Buffer buffer(ros::Duration(5000.0));

  for (rosbag::MessageInstance const m : view) {
    tf2_msgs::TFMessage::ConstPtr tf2_msg =
        m.instantiate<tf2_msgs::TFMessage>();
    if (tf2_msg != nullptr) {
      bool is_static = (m.getTopic() == kTfStaticTopic);
      for (const geometry_msgs::TransformStamped& tf_stamped :
           tf2_msg->transforms) {
        buffer.setTransform(tf_stamped, kDefaultAuthority, is_static);
      }
      continue;
    }

    geometry_msgs::TransformStamped::ConstPtr tf_msg =
        m.instantiate<geometry_msgs::TransformStamped>();
    if (tf_msg != nullptr) {
      // Make a copy, we're overwriting some stuff.
      geometry_msgs::TransformStamped tf_copy = *tf_msg;
      tf_copy.header.frame_id = bag_config_.world_frame_id;
      tf_copy.child_frame_id = bag_config_.camera_frame_id;
      buffer.setTransform(tf_copy, kDefaultAuthority, false);
      continue;
    }

    geometry_msgs::PoseStamped::ConstPtr pose_msg =
        m.instantiate<geometry_msgs::PoseStamped>();
    if (pose_msg != nullptr) {
      // Make a copy, we're overwriting some stuff.
      geometry_msgs::TransformStamped tf_copy;
      tf_copy.header.frame_id = bag_config_.world_frame_id;
      tf_copy.child_frame_id = bag_config_.camera_frame_id;

      tf_copy.transform.translation.x = pose_msg->pose.position.x;
      tf_copy.transform.translation.y = pose_msg->pose.position.y;
      tf_copy.transform.translation.z = pose_msg->pose.position.z;

      tf_copy.transform.rotation = pose_msg->pose.orientation;

      buffer.setTransform(tf_copy, kDefaultAuthority, false);
      continue;
    }
  }

  // Next, we iterate over all the images.
  std::vector<std::string> image_topics;
  image_topics.push_back(bag_config_.camera_info_topic);
  image_topics.push_back(bag_config_.image_topic);
  rosbag::View image_view(bag, rosbag::TopicQuery(image_topics));

  // We cache the last CameraInfo message.
  sensor_msgs::CameraInfo camera_info;

  for (rosbag::MessageInstance const m : image_view) {
    // Retrieve the image!
    sensor_msgs::CameraInfo::ConstPtr cam_info_msg =
        m.instantiate<sensor_msgs::CameraInfo>();
    if (cam_info_msg != nullptr) {
      camera_info = *cam_info_msg;
      continue;
    }

    sensor_msgs::Image::ConstPtr image_msg =
        m.instantiate<sensor_msgs::Image>();
    // If we haven't received any camera infos yet, just skip.
    // We *MIGHT* drop 1 frame as a result!!!!
    if (image_msg == nullptr || camera_info.height == 0) {
      LOG(ERROR) << "No valid camera info or no valid image.\n";
      continue;
    }

    // Look up its pose.
    if (buffer.canTransform(bag_config_.world_frame_id,
                            bag_config_.camera_frame_id,
                            image_msg->header.stamp)) {
      geometry_msgs::TransformStamped transform = buffer.lookupTransform(
          bag_config_.world_frame_id, bag_config_.camera_frame_id,
          image_msg->header.stamp);

      // Finally and only in this case can we actually add the frame.
      interface_.addFrame(*image_msg, camera_info, transform);
      LOG(INFO) << "Added a new frame.\n";
    } else {
      LOG(WARNING) << "Couldn't look up transform!\n";
      continue;
    }
  }
}

// Two possible options:
// Create an anchor at a given pose relative TO THE WORLD COORDINATE FRAME.
bool AsaBagReader::createAnchor(const Eigen::Affine3d& anchor_in_world_frame,
                                std::string* anchor_id) {
  return interface_.createAnchor(anchor_in_world_frame, anchor_id);
}

// Query an anchor, returning the transform between the current world coordinate
// frame and the anchor coordinate frame.
bool AsaBagReader::queryAnchor(const std::string& anchor_id,
                               Eigen::Affine3d* anchor_in_world_frame) {
  return interface_.queryAnchor(anchor_id, anchor_in_world_frame);
}

}  // namespace asa_ros
