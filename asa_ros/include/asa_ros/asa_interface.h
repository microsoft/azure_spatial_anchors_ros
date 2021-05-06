#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <mutex>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// Forward declaration:
namespace Microsoft {
namespace Azure {
namespace SpatialAnchors {
class CloudSpatialAnchorSession;
class CloudSpatialAnchorWatcher;
class CloudSpatialAnchor;
class SessionUpdatedEventArgs;
class OnLogDebugEventArgs;
class SessionErrorEventArgs;
struct event_token;
}  // namespace SpatialAnchors
}  // namespace Azure
}  // namespace Microsoft

namespace asa_ros {

// Forward declaration again.
class AsaRosProvider;
class AsaRosAnchor;

struct AsaRosConfig {
  std::string account_domain = "mixedreality.azure.com";
  std::string account_id = "";
  std::string account_key = "";
  // Queue size to keep of images in memory. Crank up for large bag files,
  // can be cranked down for live running.
  size_t max_queue_size = 1000;
  // Flip the optical axes to be in the OpenGL convention (IOS/Android/API2).
  // Should not be necessary in latest SDK versions.
  bool force_opengl_axes = false;
  // Whether to print status updates for the session. Mostly for debugging.
  bool print_status = true;
};

class AzureSpatialAnchorsInterface {
 public:
  AzureSpatialAnchorsInterface(const AsaRosConfig& config);
  virtual ~AzureSpatialAnchorsInterface();

  // MUST be called before anything else.
  void start();

  // Resets any visual data with the current session.
  // Image data will be wiped, but this would allow you to re-locate anchors
  // again.
  // Please note that this will NOT automatially re-start watching any anchors,
  // this has to be done by calling queryAnchor{s}WithCallback again.
  void reset();

  // A few different ways to add frames.
  // T_W_C is the transform of world FROM camera optical axes.
  // Optical axes are z forward, x right, y down.
  // f = focal length, c = principal point.
  // This is the main method that all others call.
  void addFrame(const uint64_t timestamp_ns, const cv::Mat& image,
                const Eigen::Affine3d& T_W_C, double fx, double fy, double cx,
                double cy);

  // Add other ROS-specific ones.
  void addFrame(const sensor_msgs::Image& image_msg,
                const sensor_msgs::CameraInfo& camera_msg,
                const geometry_msgs::TransformStamped& T_W_C_msg);

  // Create an anchor *synchronously* at the desired pose relative to the WORLD
  // coordinate frame that the poses are given in.
  bool createAnchor(const Eigen::Affine3d& anchor_in_world_frame,
                    std::string* anchor_id);

  // success, anchor_id, failure_reason.
  typedef std::function<void(bool, const std::string&, const std::string&)>
      CreatedAnchorCallbackFunction;

  // Asynchronous anchor creation: calls callback when the creation is
  // completed, which may take several seconds.
  bool createAnchorWithCallback(const Eigen::Affine3d& anchor_in_world_frame,
                                const CreatedAnchorCallbackFunction& callback);

  // Query an anchor synchronously, timeout of 10 s. Returns relative pose
  // between the anchor and the world. Strongly prefer to use
  // queryAnchorWithCallback below.
  bool queryAnchor(const std::string& anchor_id,
                   Eigen::Affine3d* anchor_in_world_frame);

  typedef std::function<void(const std::string&, const Eigen::Affine3d&)>
      FoundAnchorCallbackFunction;

  // Query an anchor, calls the specified callback if found.
  // The anchor_id may be a single anchor ID or a comma-separated list of IDs.
  // (Whitespace is trimmed.)
  // First parameter of the callback is the ID of the found anchor (blank
  // if none), second parameter is the Transform to World from the found Anchor.
  bool queryAnchorWithCallback(const std::string& anchor_id,
                               const FoundAnchorCallbackFunction& callback);
  // Same as above, multiple anchors at once.
  bool queryAnchorsWithCallback(const std::vector<std::string>& anchor_ids,
                                const FoundAnchorCallbackFunction& callback);

  // Set the callback function to publish ancho creating feedback to ROS
  typedef std::function<void(const float, const float, const std::string&)>
      CreateAnchorFeedbackCallbackFunction;
  void setCreateAnchorFeedbackCallback(
      const CreateAnchorFeedbackCallbackFunction& callback);

  // === Helpers ===
  // Validate that a string is a valid UUID for Anchor UUID, account IDs, etc.
  static bool isValidUuid(const std::string& id);
  static std::string trimWhitespace(const std::string& s);
  static std::vector<std::string> splitByDelimeter(const std::string& s,
                                                   char delimiter);
  
  // Attatches a few logging handlers
  void ActivateInterfaceLevelLogging();


 private:
  // Callback for session updates. Currently optionally prints to LOG(INFO).
  void sessionUpdateHandler(
      void*,
      const std::shared_ptr<
          Microsoft::Azure::SpatialAnchors::SessionUpdatedEventArgs>& args);
  
  // Callback for session debug logs invoked by the Linux SDK
  void sessionDebugHandler(
      void*,
      const std::shared_ptr<
          Microsoft::Azure::SpatialAnchors::OnLogDebugEventArgs>& args);

  // Callback for session error logs invoked by the Linux SDK
  void sessionErrorHandler(
      void*,
      const std::shared_ptr<
          Microsoft::Azure::SpatialAnchors::SessionErrorEventArgs>& args);

  // Cache the configuration settings for stuff that needs to be used at
  // run-time.
  AsaRosConfig config_;

  // List of all the anchors that are being tracked. Call
  // queryAnchor{s}WithCallback to reset this and add new ones.
  // This is only used when the reset() function is called to continue querying
  // existing anchors.
  std::vector<std::string> anchors_to_query_;

  // The session: this is supposed to be consistent with one coordinate frame.
  // Anchors, etc. can only be found once per session.
  std::shared_ptr<Microsoft::Azure::SpatialAnchors::CloudSpatialAnchorSession>
      session_;

  // Data provider which keeps a short cache of image data for feature
  // extraction.
  std::shared_ptr<AsaRosProvider> provider_;

  // Anchor watcher for async calls. Only one ASYNC watcher is active at a
  // time.
  std::shared_ptr<Microsoft::Azure::SpatialAnchors::CloudSpatialAnchorWatcher>
      watcher_;

  // Cache these for async anchor creation.
  // Local anchor gives ownership to the ASA SDK, so needs to not be a managed
  // pointer.
  AsaRosAnchor* local_anchor_;
  std::shared_ptr<Microsoft::Azure::SpatialAnchors::CloudSpatialAnchor>
      cloud_anchor_;

  // Tokens to keep for clean-up.
  std::unique_ptr<Microsoft::Azure::SpatialAnchors::event_token>
      session_update_token_;
  std::unique_ptr<Microsoft::Azure::SpatialAnchors::event_token>
      anchor_located_token_;
  std::unique_ptr<Microsoft::Azure::SpatialAnchors::event_token>
      session_debuglog_token_;
  std::unique_ptr<Microsoft::Azure::SpatialAnchors::event_token>
      session_error_token_;

  // Keep track of how many frames were added.
  size_t frame_count_;

  // Callback function to send create anchor feedback to ROS
  CreateAnchorFeedbackCallbackFunction feedback_callback_;

  // Mutex for locking new frames when creating an anchor.
  std::mutex frame_mutex_;
  std::mutex session_update_mutex_;
  std::mutex create_anchor_mutex_;
};

}  // namespace asa_ros
