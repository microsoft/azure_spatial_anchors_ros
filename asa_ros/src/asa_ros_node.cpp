#include <cstdlib>
#include <fstream>
#include <sstream>

#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

#include "asa_ros/asa_ros_node.h"

namespace asa_ros {

                                            
AsaRosNode::AsaRosNode(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      tf_buffer_(ros::Duration(120.0)),
      tf_listener_(tf_buffer_),
      world_frame_id_("world"),
      camera_frame_id_(""),
      anchor_frame_id_(""),
      broadcast_anchor_tf_(true),
      tf_lookup_timeout_(0.1),
      prev_frame_timestamp_() {
  initFromRosParams();
}

AsaRosNode::~AsaRosNode() {}


void AsaRosNode::initFromRosParams() {

  nh_private_.param("subscriber_queue_size", queue_size_, 1);
  nh_private_.param("use_approx_sync_policy", use_approx_sync_policy_, false);
  nh_private_.param("activate_interface_level_logging",
                    activate_interface_level_logging_,
                    false);
  nh_private_.param("query_last_anchor_id_from_cache",
                    query_last_anchor_id_from_cache_,
                    false);

  // Set anchor id cache path
  std::string last_anchor_cache_path_default;
  // $HOME should be defined, but if not, put it in /tmp
  const char *home_path = getenv("HOME");
  if(home_path) {
    last_anchor_cache_path_default = std::string(home_path) +
                                     "/.ros/last_anchor_id";
  }
  else {
    last_anchor_cache_path_default = "/tmp/last_anchor_id";
  }
  nh_private_.param<std::string>("last_anchor_cache_path",
                                 last_anchor_cache_path_,
                                 last_anchor_cache_path_default);

  if(use_approx_sync_policy_) {
    ROS_INFO_STREAM("Starting image and info subscribers with approximate " <<
                    "time sync, where queue size is " << queue_size_);
  }

  // Subscribe to the camera images.
  image_sub_.subscribe(nh_, "image", queue_size_);
  info_sub_.subscribe(nh_, "camera_info", queue_size_);
  
  if(use_approx_sync_policy_) {
    image_info_approx_sync_.reset(new message_filters::Synchronizer<CameraSyncPolicy>(CameraSyncPolicy(queue_size_), image_sub_, info_sub_));
    image_info_approx_sync_->registerCallback(boost::bind(&AsaRosNode::imageAndInfoCallback, this, _1, _2));
  } else {
    image_info_sync_.reset(
        new message_filters::TimeSynchronizer<sensor_msgs::Image,
                                              sensor_msgs::CameraInfo>(image_sub_, info_sub_, 10));
    image_info_sync_->registerCallback(
        boost::bind(&AsaRosNode::imageAndInfoCallback, this, _1, _2));
  }

  // Subscribe to transform topics, if any.
  transform_sub_ =
      nh_.subscribe("transform", 1, &AsaRosNode::transformCallback, this);

  // Publishers.
  found_anchor_pub_ =
      nh_private_.advertise<asa_ros_msgs::FoundAnchor>("found_anchor", 1, true);
  created_anchor_pub_ = nh_private_.advertise<asa_ros_msgs::CreatedAnchor>(
      "created_anchor", 1, true);
  feedback_pub_ = nh_private_.advertise<asa_ros_msgs::CreateAnchorFeedback>(
      "create_anchor_feedback", 1, true);

  // Services.
  create_anchor_srv_ = nh_private_.advertiseService(
      "create_anchor", &AsaRosNode::createAnchorCallback, this);
  find_anchor_srv_ = nh_private_.advertiseService(
      "find_anchor", &AsaRosNode::findAnchorCallback, this);
  reset_srv_ =
      nh_private_.advertiseService("reset", &AsaRosNode::resetCallback, this);
  reset_completely_srv_ = nh_private_.advertiseService(
      "reset_completely", &AsaRosNode::resetCompletelyCallback, this);

  // Transform settings.
  nh_private_.param("world_frame_id", world_frame_id_, world_frame_id_);
  nh_private_.param("camera_frame_id", camera_frame_id_, camera_frame_id_);
  nh_private_.param("anchor_frame_id", anchor_frame_id_, anchor_frame_id_);
  nh_private_.param("broadcast_anchor_tf", broadcast_anchor_tf_, broadcast_anchor_tf_);
  nh_private_.param("tf_lookup_timeout", tf_lookup_timeout_,
                    tf_lookup_timeout_);

  // Load ASA config and set it up.
  AsaRosConfig asa_config;

  // Set a small queue size to save RAMs.
  asa_config.max_queue_size = 50;

  // Set up the account IDs.
  nh_private_.param("account_id", asa_config.account_id, asa_config.account_id);
  nh_private_.param("account_key", asa_config.account_key,
                    asa_config.account_key);
  nh_private_.param("account_domain", asa_config.account_domain,
                    asa_config.account_domain);

  ROS_INFO_STREAM("Account domain: " << asa_config.account_domain
                                     << " account ID: " << asa_config.account_id
                                     << " account key: "
                                     << asa_config.account_key);

  interface_.reset(new AzureSpatialAnchorsInterface(asa_config));
  interface_->setCreateAnchorFeedbackCallback(
    std::bind(&AsaRosNode::createAnchorFeedbackCallback, this,
              std::placeholders::_1, std::placeholders::_2,
              std::placeholders::_3));
  if(activate_interface_level_logging_){
    interface_->ActivateInterfaceLevelLogging();
  }
  interface_->start();

  std::string anchor_id;
  nh_private_.param("anchor_id", anchor_id, anchor_id);
  if (!anchor_id.empty()) {
    // Start looking for an anchor immediately if one is given as a param.
    interface_->queryAnchorWithCallback(
        anchor_id, std::bind(&AsaRosNode::anchorFoundCallback, this,
                             std::placeholders::_1, std::placeholders::_2));
  }
  else if(query_last_anchor_id_from_cache_) {
    // Or look for the last created anchor if this flag is set
    const std::string cached_anchor_id = readCachedAnchorId();
    ROS_INFO_STREAM("Querying cached anchor id: " << cached_anchor_id);
    interface_->queryAnchorWithCallback(
        cached_anchor_id, std::bind(&AsaRosNode::anchorFoundCallback, this,
                             std::placeholders::_1, std::placeholders::_2));
  }
}

// Returns true if the camera intrinsics seem to be valid. Does not validate correctness
bool ValidateCameraIntrinsics(const boost::array<double, 9> K, int image_height, int image_width){
  return K.at(0) > 0 && K.at(4) > 0 && K.at(2) >= 0 && K.at(5) >= 0 && image_height > 0 && image_width > 0;
}

void AsaRosNode::imageAndInfoCallback(
    const sensor_msgs::Image::ConstPtr& image,
    const sensor_msgs::CameraInfo::ConstPtr& camera_info) {
  
  if (camera_frame_id_.empty()) {
    camera_frame_id_ = image->header.frame_id;
    ROS_INFO_STREAM("Set camera frame ID to " << camera_frame_id_);
  }

  if(!ValidateCameraIntrinsics(camera_info->K, image->height, image->width)){
    ROS_WARN_ONCE("The camera_info topic reported invalid values. The anchor creation will fail. Check the camera configuration.");
  }

  // Look up its pose.
  if (tf_buffer_.canTransform(world_frame_id_, camera_frame_id_,
                              image->header.stamp,
                              ros::Duration(tf_lookup_timeout_))) {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        world_frame_id_, camera_frame_id_, image->header.stamp);

    // Finally and only in this case can we actually add the frame.
    interface_->addFrame(*image, *camera_info, transform);
    prev_frame_timestamp_ = image->header.stamp;
    ROS_INFO_ONCE("Added first frame.");
  } else {
    ROS_WARN_STREAM("Couldn't look up transform from "
                    << world_frame_id_ << " to " << camera_frame_id_
                    << " at timestamp " << image->header.stamp
                    << " and ros::Time::now() " << ros::Time::now());
  }
}

void AsaRosNode::transformCallback(const geometry_msgs::TransformStamped& msg) {
  const std::string kDefaultAuthority = "default_authority";
  const bool kIsStatic = false;

  geometry_msgs::TransformStamped tf_copy = msg;
  tf_copy.header.frame_id = world_frame_id_;
  tf_copy.child_frame_id = camera_frame_id_;
  tf_buffer_.setTransform(tf_copy, kDefaultAuthority, kIsStatic);
}

void AsaRosNode::anchorFoundCallback(
    const std::string& anchor_id,
    const Eigen::Affine3d& anchor_in_world_frame) {
  ROS_INFO_STREAM("Found anchor: " << anchor_id << "\n"
                                   << anchor_in_world_frame.matrix());
  // Here we will just publish a TF frame.
  geometry_msgs::TransformStamped T_W_A_msg =
      tf2::eigenToTransform(anchor_in_world_frame);
  T_W_A_msg.header.frame_id = world_frame_id_;
  T_W_A_msg.header.stamp = ros::Time::now();
  T_W_A_msg.child_frame_id = anchor_frame_id_;
  // If anchor frame ID is empty, just publish the anchor ID.
  if (anchor_frame_id_.empty()) {
    T_W_A_msg.child_frame_id = anchor_id;
  }

  if (broadcast_anchor_tf_) {
    tf_broadcaster_.sendTransform(T_W_A_msg);
  }

  // Also publish this as a topic.
  asa_ros_msgs::FoundAnchor anchor_msg;
  anchor_msg.anchor_id = anchor_id;
  anchor_msg.anchor_in_world_frame = T_W_A_msg;

  found_anchor_pub_.publish(anchor_msg);
}

bool AsaRosNode::createAnchorCallback(asa_ros_msgs::CreateAnchorRequest& req,
                                      asa_ros_msgs::CreateAnchorResponse& res) {
  Eigen::Affine3d anchor_in_target_frame;
  anchor_in_target_frame = tf2::transformToEigen(req.anchor_in_target_frame);

  // Compute anchor in world frame if pose given relative to target frame
  Eigen::Affine3d anchor_in_world_frame;
  if(req.target_frame.empty()) {
    anchor_in_world_frame = anchor_in_target_frame;
  }
  else {
    if (prev_frame_timestamp_ == ros::Time(0))
    {
      ROS_WARN_STREAM("Failed to create anchor: no frames have been added.");
      return false;
    }
    if (tf_buffer_.canTransform(world_frame_id_,
                                req.target_frame,
                                prev_frame_timestamp_,
                                ros::Duration(tf_lookup_timeout_))) {

      geometry_msgs::TransformStamped target_in_world_frame_tf =
          tf_buffer_.lookupTransform(world_frame_id_,
                                     req.target_frame,
                                     prev_frame_timestamp_,
                                     ros::Duration(tf_lookup_timeout_));
      anchor_in_world_frame = tf2::transformToEigen(target_in_world_frame_tf) *
                                  anchor_in_target_frame;
    }
    else {
      ROS_WARN_STREAM("Failed to find target frame in TF tree. " <<
                       "Anchor not created.");
      return false;
    }
  }

  bool success = interface_->createAnchorWithCallback(
      anchor_in_world_frame,
      std::bind(&AsaRosNode::anchorCreatedCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  // If there is not enough "spatial data" (different perspectives on the
  // scene), then anchor creation will fail immediately. This can be fixed by
  // getting more viewpoints.
  // However, this returns true because it's not possible to pass an error
  // message back to the process calling the service, so we just print an
  // error if the create anchor call fails.
  return true;
}

bool AsaRosNode::findAnchorCallback(asa_ros_msgs::FindAnchorRequest& req,
                                    asa_ros_msgs::FindAnchorResponse& res) {
  // anchor_id is deprecated but still supported for now
  // if the client leaves it empty, use the new vector member instead
  if (req.anchor_id.empty()) {
    queryAnchors(convertIdVectorToString(req.anchor_ids));
  }
  else {
    ROS_WARN_ONCE("The anchor_id member of FindAnchor.srv is deprecated and will be "
                  "removed in a future release. Use the anchor_ids vector member instead.");
    
    queryAnchors(req.anchor_id);
  }
                                      
  return true;
}

void AsaRosNode::anchorCreatedCallback(bool success,
                                       const std::string& anchor_id,
                                       const std::string& reason) {
  asa_ros_msgs::CreatedAnchor anchor_msg;
  anchor_msg.success = success;
  anchor_msg.anchor_id = anchor_id;
  anchor_msg.failure_reason = reason;

  if (success) {
    ROS_INFO_STREAM("Successfully created an anchor with ID: " << anchor_id);
  } else {
    ROS_WARN_STREAM("Unable to create anchor. Reason: " << reason);
  }
  created_anchor_pub_.publish(anchor_msg);
  if(storeAnchorIdInCache(anchor_id)) {
    ROS_INFO_STREAM("Stored anchor id in cache.");
  }
}

std::string AsaRosNode::convertIdVectorToString(const std::vector<std::string>& vec) const
{  
  std::stringstream output;
  
  const size_t last_index = vec.size() - 1;
  const char separator = ',';
  
  for (int i = 0; i < vec.size(); ++i) {
    output << vec.at(i);
    
    // if this is not the last element, add the separator
    if (i != last_index) {
      output << separator;
    }
  }
  
  return output.str();
}

bool AsaRosNode::queryAnchors(const std::string& anchor_ids) {
  anchor_ids_ = anchor_ids;
  if (interface_->queryAnchorWithCallback(
          anchor_ids,
          std::bind(&AsaRosNode::anchorFoundCallback, this,
                    std::placeholders::_1, std::placeholders::_2))) {
    // Only update anchor ID list if it's valid.
    anchor_ids_ = anchor_ids;
    return true;
  }
  return false;
}

void AsaRosNode::createAnchorFeedbackCallback(
    const float ready_for_create_progress,
    const float recommended_for_create_progress,
    const std::string& user_feedback) {

  if(feedback_pub_.getNumSubscribers() > 0) {
    asa_ros_msgs::CreateAnchorFeedback msg;
    msg.ready_for_create_progress = ready_for_create_progress;
    msg.recommended_for_create_progress = recommended_for_create_progress;
    msg.user_feedback = user_feedback;
    feedback_pub_.publish(msg);
  }
}

void AsaRosNode::createAnchorTimerCallback(const ros::TimerEvent& e) {
  std::string id;
  Eigen::Affine3d anchor_in_world_frame = Eigen::Affine3d::Identity();
  interface_->createAnchor(anchor_in_world_frame, &id);
  ROS_INFO_STREAM("Created an anchor with ID: " << id);
}

std::string AsaRosNode::readCachedAnchorId() {
  std::string cached_anchor_id;

  // Try to read cache file
  std::ifstream cache_file(last_anchor_cache_path_.c_str());
  if(cache_file) {
    std::ostringstream string_stream;
    string_stream << cache_file.rdbuf();
    cached_anchor_id = string_stream.str();
    ROS_INFO_STREAM("Read anchor id: " << cached_anchor_id << " from cache");
  }
  else {
    ROS_ERROR_STREAM("Could not read from anchor id cache file: " <<
                      last_anchor_cache_path_);
  }

  return cached_anchor_id;
}

bool AsaRosNode::storeAnchorIdInCache(const std::string& created_anchor_id) {
  std::ofstream cache_file(last_anchor_cache_path_.c_str());
  if(cache_file) {
    cache_file << created_anchor_id;
    cache_file.close();
    return true;
  }
  else {
    ROS_ERROR_STREAM("Could not open anchor cache file for writing: " <<
                     last_anchor_cache_path_);
    return false;
  }
}

// Does a "soft" reset: continues to try to find any anchors that are
// currently being tracked.
bool AsaRosNode::resetCallback(std_srvs::EmptyRequest& req,
                               std_srvs::EmptyResponse& res) {
  interface_->reset();
  // Also re-query last set of anchors.
  queryAnchors(anchor_ids_);
  return true;
}

// Does a "hard" reset: stops tracking any anchors, wipes any session
// information.
bool AsaRosNode::resetCompletelyCallback(std_srvs::EmptyRequest& req,
                                         std_srvs::EmptyResponse& res) {
  interface_->reset();
  return true;
}

}  // namespace asa_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = 1;

  ros::init(argc, argv, "asa_ros");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  asa_ros::AsaRosNode node(nh, nh_private);
  ROS_INFO("[Azure Spatial Anchors ROS] Now running.");

  ros::spin();
  return 0;
}
