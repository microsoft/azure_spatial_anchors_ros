#include <algorithm>
#include <functional>
#include <regex>
#include <string>

#include <AzureSpatialAnchors.h>
#include <AzureSpatialAnchorsProvider.hpp>

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <tf2_eigen/tf2_eigen.h>

#include "asa_ros/asa_interface.h"
#include "asa_ros/asa_ros_provider.h"

// For debugging only
#include "asa_ros/internal/asa_helper.h"
#include "asa_ros/internal/wait_for_value.h"

namespace asa_ros {

namespace asa = Microsoft::Azure::SpatialAnchors;

// Helper functions.
void eigenToAsaTransform(const Eigen::Affine3d& tf_eigen,
                         asa::Provider::PoseRotationTranslationScale* tf_asa) {
  // We very lazily just map these to eigen and set them.
  float rot_array[9] = {0};
  float trans_array[3] = {0};
  Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> rotation(rot_array);
  Eigen::Map<Eigen::Matrix<float, 3, 1>> translation(trans_array);

  rotation = tf_eigen.linear().cast<float>();
  translation = tf_eigen.translation().cast<float>();
  tf_asa->Rotation(rot_array);
  tf_asa->Translation(trans_array);
  tf_asa->Scale(1.0);
}

void asaToEigenTransform(
    const asa::Provider::PoseRotationTranslationScale& tf_asa,
    Eigen::Affine3d* tf_eigen) {
  // The inverse of above. Still lazily map.
  Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> rotation(
      tf_asa.Rotation());
  Eigen::Map<const Eigen::Matrix<float, 3, 1>> translation(
      tf_asa.Translation());

  tf_eigen->linear() = rotation.cast<double>();
  tf_eigen->translation() = translation.cast<double>();
}

AzureSpatialAnchorsInterface::AzureSpatialAnchorsInterface(
    const AsaRosConfig& config)
    : config_(config), frame_count_(0) {
  // Register the provider. Has to be done first now.
  provider_.reset(new AsaRosProvider(config.max_queue_size));
  asa::Provider::ProviderRegistration registration =
      asa::Provider::Registrar::Register(provider_);

  session_.reset(new asa::CloudSpatialAnchorSession());

  session_->Configuration()->AccountId(config.account_id);
  session_->Configuration()->AccountKey(config.account_key);

  if (!config.account_domain.empty()) {
    session_->Configuration()->AccountDomain(config.account_domain);
  }
}

void AzureSpatialAnchorsInterface::start() {
  // Create a session handle and session.
  asa_session_handle context_handle =
      reinterpret_cast<asa_session_handle>(0x12345);
  session_->Session(context_handle);
  session_->Start();

  // Reset frame count.
  frame_count_ = 0;

  // Bind an event handler so we have some idea of what's going on.
  session_update_token_.reset(
      new Microsoft::Azure::SpatialAnchors::event_token);
  *session_update_token_ = session_->SessionUpdated(
      std::bind(&AzureSpatialAnchorsInterface::sessionUpdateHandler, this,
                std::placeholders::_1, std::placeholders::_2));
}

void AzureSpatialAnchorsInterface::reset() {
  // Resets the session.
  session_->Reset();
}

AzureSpatialAnchorsInterface::~AzureSpatialAnchorsInterface() {
  if (watcher_) {
    watcher_->Stop();
  }
  session_->Stop();
  session_->Dispose();
}

void AzureSpatialAnchorsInterface::addFrame(const uint64_t timestamp_ns,
                                            const cv::Mat& image,
                                            const Eigen::Affine3d& T_W_C,
                                            double fx, double fy, double cx,
                                            double cy) {
  std::unique_lock<std::mutex> frame_lock(frame_mutex_, std::try_to_lock);

  if (!frame_lock) {
    // This means we're currently creating an anchor, just skip this frame, who
    // cares.
    return;
  }

  // This is the thing we need to populate.
  asa::Provider::Frame frame;

  // This is the camera object, describing camera params.
  asa::Provider::PinholeCamera camera;

  camera.Width(image.cols);
  camera.Height(image.rows);
  camera.SetFocalLength(fx, fy);
  camera.SetPrincipalPoint(cx, cy);
  frame.Camera(camera);

  // Assume canonical ROS world frame where gravity points down in z :)
  frame.DownVector(0, 0, -1);

  // This is custom for us to set. Need to be able to retrieve the image
  // pixels based on this.
  size_t* image_id = new size_t;
  *image_id = provider_->addImageToQueue(image);
  frame.FrameContext(image_id);

  // This takes in timestamp in MILLISECONDS. Truncating divide.
  frame.Timestamp(timestamp_ns / 1000);

  // Packed as [R00 R01 R02 ... R22 T0 T1 T2 S]. (rotation matrix first, then
  // translation, then scale).
  // We do some lazy Eigen mapping.
  asa::Provider::PoseRotationTranslationScale T_C_W_asa;

  Eigen::Affine3d T_W_C_copy = T_W_C;
  if (config_.force_opengl_axes) {
    // This rotates it to the z-backwards OpenGL convention, hopefully no longer
    // necessary.
    T_W_C_copy = T_W_C * (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  }

  Eigen::Affine3d T_C_W = T_W_C_copy.inverse();

  eigenToAsaTransform(T_C_W, &T_C_W_asa);

  frame.WorldToCameraPose(T_C_W_asa);

  try {
    // Finally we process the frame.
    session_->ProcessFrame(frame.GetHandle());
  } catch (Microsoft::Azure::SpatialAnchors::runtime_error& e) {
    LOG(ERROR) << "Couldn't add a frame: " << e.message();

    LOG(ERROR) << "Type of camera: "
               << reinterpret_cast<asap_frame*>(frame.GetHandle())->camera->type;
    return;
  }

  // Keep track.
  VLOG(3) << "Adding frame #" << frame_count_;
  frame_count_++;
}

void AzureSpatialAnchorsInterface::addFrame(
    const sensor_msgs::Image& image_msg,
    const sensor_msgs::CameraInfo& camera_msg,
    const geometry_msgs::TransformStamped& T_W_C_msg) {
  // Let's just unpack what we need.
  uint64_t timestamp_ns = image_msg.header.stamp.toNSec();

  // Camera parameters.
  size_t height = camera_msg.height;
  size_t width = camera_msg.width;

  double fx = camera_msg.K[0];
  double fy = camera_msg.K[4];
  double cx = camera_msg.K[2];
  double cy = camera_msg.K[5];

  // Convert the image.
  cv::Mat image = cv_bridge::toCvCopy(image_msg)->image;

  // Convert the pose.
  Eigen::Affine3d T_W_C = tf2::transformToEigen(T_W_C_msg);

  // Call the underlying function!
  addFrame(timestamp_ns, image, T_W_C, fx, fy, cx, cy);
}

// Create an anchor at the desired pose relative to the WORLD coordinate frame
// that the poses are given in.
bool AzureSpatialAnchorsInterface::createAnchor(
    const Eigen::Affine3d& anchor_in_world_frame, std::string* anchor_id) {
  if (frame_count_ == 0) {
    LOG(ERROR) << "No frames have been added! Can't create an anchor!";
    return false;
  }

  LOG(INFO) << "Starting to create anchor with anchor_in_world_frame:\n"
            << anchor_in_world_frame.matrix();

  // Create a new anchor at the correct location relative to the world.
  asa::Provider::PoseRotationTranslationScale T_W_A_asa;
  eigenToAsaTransform(anchor_in_world_frame, &T_W_A_asa);

  LOG(INFO) << "T_W_A_asa:\n" << T_W_A_asa;

  std::unique_ptr<AsaRosAnchor> ros_anchor =
      std::make_unique<AsaRosAnchor>(T_W_A_asa);
  std::shared_ptr<asa::CloudSpatialAnchor> asa_anchor =
      std::make_shared<asa::CloudSpatialAnchor>();

  // Bind the Spatial (Cloud) anchor to the little local one (again just
  // stores the pose).
  asa_anchor->LocalAnchor(ros_anchor->GetHandle());

  asa::Status status;
  try {
    WaitForValue<asa::Status> wait_for_status;
    session_->CreateAnchorAsync(
        asa_anchor, [&](asa::Status status) { wait_for_status.Set(status); });

    status = wait_for_status.Wait(std::chrono::seconds(20));

    LOG(INFO) << "Status = " << to_string(status);
    LOG(INFO) << "Spatial Anchor Identifier = " << asa_anchor->Identifier();

  } catch (std::exception& e) {
    LOG(ERROR) << "Failed to create anchor: " << e.what();
    return false;
  }

  if (status == asa::Status::OK) {
    return true;
  }
  return false;
}

// Create an anchor at the desired pose relative to the WORLD coordinate frame
// that the poses are given in.
bool AzureSpatialAnchorsInterface::createAnchorWithCallback(
    const Eigen::Affine3d& anchor_in_world_frame,
    const CreatedAnchorCallbackFunction& callback) {
  if (frame_count_ == 0) {
    LOG(ERROR) << "No frames have been added! Can't create an anchor!";
    return false;
  }

  create_anchor_mutex_.lock();

  LOG(INFO) << "Starting to create anchor with anchor_in_world_frame:\n"
            << anchor_in_world_frame.matrix();

  // Create a new anchor at the correct location relative to the world.
  asa::Provider::PoseRotationTranslationScale T_W_A_asa;
  eigenToAsaTransform(anchor_in_world_frame, &T_W_A_asa);

  LOG(INFO) << "T_W_A_asa:\n" << T_W_A_asa;

  local_anchor_ = new AsaRosAnchor(T_W_A_asa);
  cloud_anchor_.reset(new asa::CloudSpatialAnchor);

  // Bind the Spatial (Cloud) anchor to the little local one (again just
  // stores the pose).
  cloud_anchor_->LocalAnchor(local_anchor_->GetHandle());

  asa::Status status;
  try {
    frame_mutex_.lock();
    session_->CreateAnchorAsync(
        cloud_anchor_, [this, callback](asa::Status status) {
          LOG(INFO) << "Create anchor status: " << to_string(status);
          this->frame_mutex_.unlock();
          callback(status == asa::Status::OK, this->cloud_anchor_->Identifier(),
                   to_string(status));
          cloud_anchor_.reset();
          local_anchor_ = nullptr;
          create_anchor_mutex_.unlock();
        });
  } catch (std::exception& e) {
    LOG(ERROR) << "Failed to create anchor: " << e.what();
    frame_mutex_.unlock();
    create_anchor_mutex_.unlock();
    return false;
  }
  return true;
}

// Query an anchor, returns relative pose between the anchor and the world.
bool AzureSpatialAnchorsInterface::queryAnchor(
    const std::string& anchor_id, Eigen::Affine3d* anchor_in_world_frame) {
  if (frame_count_ == 0) {
    LOG(ERROR) << "No frames have been added! Can't query an anchor!";
    return false;
  }

  std::string anchor_id_copy = trimWhitespace(anchor_id);
  if (!isValidUuid(anchor_id_copy)) {
    LOG(ERROR) << "Invalid anchor id: " << anchor_id_copy;
    return false;
  }

  LOG(INFO) << "Starting to look for anchor ID: " << anchor_id_copy;

  std::mutex found_anchors_mutex;
  std::condition_variable found_anchors_cv;
  std::vector<std::shared_ptr<asa::CloudSpatialAnchor>> found_anchors;

  // Dispose of the previous token if any:
  if (anchor_located_token_) {
    session_->AnchorLocated(*anchor_located_token_);
  }

  // Set up a lambda to update a local variable when anchors are found.
  anchor_located_token_.reset(
      new Microsoft::Azure::SpatialAnchors::event_token);
  *anchor_located_token_ = session_->AnchorLocated(
      [&](void*, const std::shared_ptr<asa::AnchorLocatedEventArgs>&
                     anchor_located_event) {
        std::unique_lock<std::mutex> lock(found_anchors_mutex);
        found_anchors.push_back(anchor_located_event->Anchor());
        found_anchors_cv.notify_all();
      });

  // Create an anchor watcher for these specific criteria.
  try {
    std::shared_ptr<asa::AnchorLocateCriteria> criteria =
        std::make_shared<asa::AnchorLocateCriteria>();
    criteria->Identifiers({anchor_id_copy});

    if (watcher_) {
      watcher_->Stop();
      watcher_.reset();
    }
    watcher_ = session_->CreateWatcher(criteria);

    {
      std::unique_lock<std::mutex> lock(found_anchors_mutex);
      if (!found_anchors_cv.wait_for(lock, std::chrono::seconds(10), [&]() {
            return !found_anchors.empty();
          })) {
        LOG(WARNING) << "Timeout or no anchors found.";
        return false;
      }
    }

    // Stop the watcher.
    watcher_->Stop();
    watcher_.reset();

  } catch (Microsoft::Azure::SpatialAnchors::runtime_error& e) {
    LOG(ERROR) << "Couldn't query an anchor: " << e.message();

    watcher_->Stop();
    watcher_.reset();
  }

  LOG(INFO) << "Found: " << found_anchors[0]->Identifier() << std::endl;

  AsaRosAnchor* ros_anchor = static_cast<AsaRosAnchor*>(
      Microsoft::Azure::SpatialAnchors::Provider::ARAnchor::FromHandle(
          found_anchors[0]->LocalAnchor()));
  asa::Provider::PoseRotationTranslationScale T_W_A_asa;
  T_W_A_asa = ros_anchor->anchorInWorldFrameRTS();

  LOG(INFO) << "Final pose:\n" << T_W_A_asa << std::endl;

  asaToEigenTransform(T_W_A_asa, anchor_in_world_frame);

  return true;
}

bool AzureSpatialAnchorsInterface::queryAnchorWithCallback(
    const std::string& anchor_id, const FoundAnchorCallbackFunction& callback) {
  // Split the anchor string along comma-separation.
  return queryAnchorsWithCallback(splitByDelimeter(anchor_id, ','), callback);
}

bool AzureSpatialAnchorsInterface::queryAnchorsWithCallback(
    const std::vector<std::string>& anchor_ids,
    const FoundAnchorCallbackFunction& callback) {
  anchors_to_query_.clear();
  for (const std::string& anchor_id : anchor_ids) {
    std::string anchor_id_copy = trimWhitespace(anchor_id);
    if (!isValidUuid(anchor_id_copy)) {
      LOG(ERROR) << "Invalid anchor id: " << anchor_id_copy;
      return false;
    }

    LOG(INFO) << "Starting to look for anchor ID: " << anchor_id_copy;
    anchors_to_query_.push_back(anchor_id_copy);
  }

  // Dispose of the previous token if any:
  if (anchor_located_token_) {
    session_->AnchorLocated(*anchor_located_token_);
  }

  // Set up a lambda to update a local variable when anchors are found.
  anchor_located_token_.reset(
      new Microsoft::Azure::SpatialAnchors::event_token);
  *anchor_located_token_ = session_->AnchorLocated(
      [this, callback](void*,
                       const std::shared_ptr<asa::AnchorLocatedEventArgs>&
                           anchor_located_event) {
        AsaRosAnchor* ros_anchor = static_cast<AsaRosAnchor*>(
            Microsoft::Azure::SpatialAnchors::Provider::ARAnchor::FromHandle(
                anchor_located_event->Anchor()->LocalAnchor()));

        Eigen::Affine3d anchor_in_world_frame = Eigen::Affine3d::Identity();
        asaToEigenTransform(ros_anchor->anchorInWorldFrameRTS(),
                            &anchor_in_world_frame);

        callback(anchor_located_event->Anchor()->Identifier(),
                 anchor_in_world_frame);
      });

  try {
    // Create an anchor watcher for these specific criteria.
    std::shared_ptr<asa::AnchorLocateCriteria> criteria =
        std::make_shared<asa::AnchorLocateCriteria>();
    criteria->Identifiers(anchors_to_query_);

    if (watcher_) {
      watcher_->Stop();
      watcher_.reset();
    }

    // Overwrite the existing watcher. Does it need to be explicitly stopped?
    watcher_ = session_->CreateWatcher(criteria);

  } catch (Microsoft::Azure::SpatialAnchors::runtime_error& e) {
    LOG(ERROR) << "Couldn't query an anchor: " << e.message();

    watcher_->Stop();
    watcher_.reset();
  }
  return true;
}

void AzureSpatialAnchorsInterface::sessionUpdateHandler(
    void*,
    const std::shared_ptr<
        Microsoft::Azure::SpatialAnchors::SessionUpdatedEventArgs>& args) {
  std::unique_lock<std::mutex> lock(session_update_mutex_);

  if (config_.print_status) {
    LOG(INFO) << "----------------------------------------------------------\n";
    LOG(INFO) << "      ReadyForCreateProgress : "
              << args->Status()->ReadyForCreateProgress() << "\n";
    LOG(INFO) << "RecommendedForCreateProgress : "
              << args->Status()->RecommendedForCreateProgress() << "\n";
    LOG(INFO) << "                UserFeedback : "
              << to_string(args->Status()->UserFeedback()) << "\n";
  }
}

std::string AzureSpatialAnchorsInterface::trimWhitespace(const std::string& s) {
  // From: https://www.techiedelight.com/remove-whitespaces-string-cpp/
  std::string s_copy;
  std::regex r("[\'\"\\s]+");
  s_copy = std::regex_replace(s, r, "");

  return s_copy;
}

std::vector<std::string> AzureSpatialAnchorsInterface::splitByDelimeter(
    const std::string& s, char delimiter) {
  // From: https://www.fluentcpp.com/2017/04/21/how-to-split-a-string-in-c/
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream token_stream(s);
  while (std::getline(token_stream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

bool AzureSpatialAnchorsInterface::isValidUuid(const std::string& id) {
  // From: https://www.regextester.com/99148
  const std::regex uuid_regex(
      "[0-9a-fA-F]{8}\\-[0-9a-fA-F]{4}\\-[0-9a-fA-F]{4}\\-[0-9a-fA-F]{4}\\-[0-"
      "9a-fA-F]{12}");
  return std::regex_match(id, uuid_regex);
}

}  // namespace asa_ros
