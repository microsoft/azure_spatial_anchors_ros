#pragma once

#include <cv_bridge/cv_bridge.h>
#include <AzureSpatialAnchorsProvider.hpp>
#include <map>
#include <mutex>

namespace asa_ros {

// This is a wrapper interface class that's... not particularly useful
// in our case, but mirrors how ARCore/ARKit work.
class AsaRosProvider
    : public Microsoft::Azure::SpatialAnchors::Provider::ARProvider {
 public:
  AsaRosProvider(size_t max_queue_size);

  void* CreatePlatformAnchor(
      const Microsoft::Azure::SpatialAnchors::Provider::Pose&
          anchor_in_world_frame) override final;

  void ReleasePlatformAnchor(void* asa_ros_anchor) override final;

  void GetPlatformAnchorToWorldPose(
      const void* asa_ros_anchor,
      Microsoft::Azure::SpatialAnchors::Provider::Pose& anchor_in_world_frame)
      override final;

  void GetPixelData(const void* frame_context, uint8_t* pixel_buffer,
                    size_t buffer_size) override final;

  // Adds an image to the query queue, returns the unique query ID of the image.
  size_t addImageToQueue(const cv::Mat& image);

 private:
  size_t max_queue_size_;

  std::map<size_t, cv::Mat> image_queue_;
  size_t next_queue_id_;

  std::mutex provider_mutex_;
};

// We have to have some kind of fake platform anchors just to fit the
// convention. These are literally just arbitrary coordinate frames in
// space.
class AsaRosAnchor {
 public:
  AsaRosAnchor(const Microsoft::Azure::SpatialAnchors::Provider::Pose&
                   anchor_in_world_frame)
      : anchor_in_world_frame_(anchor_in_world_frame) {}

  Microsoft::Azure::SpatialAnchors::Provider::Pose anchorInWorldFrame() const {
    return anchor_in_world_frame_;
  }

  // Creates A COPY!!!
  Microsoft::Azure::SpatialAnchors::Provider::PoseRotationTranslationScale
  anchorInWorldFrameRTS() const {
    return *(
        anchor_in_world_frame_.As<Microsoft::Azure::SpatialAnchors::Provider::
                                      PoseRotationTranslationScale>());
  }

 private:
  Microsoft::Azure::SpatialAnchors::Provider::Pose anchor_in_world_frame_;
};

}  // namespace asa_ros
