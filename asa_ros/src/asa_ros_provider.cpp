#include <glog/logging.h>
#include <iostream>

#include "asa_ros/asa_ros_provider.h"

namespace asa_ros {

AsaRosProvider::AsaRosProvider(size_t max_queue_size)
    : max_queue_size_(max_queue_size), next_queue_id_(0) {}

Microsoft::Azure::SpatialAnchors::Provider::ARAnchor*
AsaRosProvider::CreateAnchor(
    const Microsoft::Azure::SpatialAnchors::Provider::Pose&
        anchor_in_world_frame) {
  return new AsaRosAnchor(anchor_in_world_frame);
}

void AsaRosProvider::GetAnchorToWorldPose(
    const Microsoft::Azure::SpatialAnchors::Provider::ARAnchor& anchor,
    Microsoft::Azure::SpatialAnchors::Provider::Pose& anchor_in_world_frame) {
  anchor_in_world_frame =
      static_cast<const AsaRosAnchor&>(anchor).anchorInWorldFrame();
}

void AsaRosProvider::GetPixelData(const void* frame_context,
                                  uint8_t* pixel_buffer, size_t buffer_size) {
  std::unique_lock<std::mutex> provider_lock(provider_mutex_);

  size_t image_key = *(static_cast<const size_t*>(frame_context));

  const auto iter = image_queue_.find(image_key);
  if (iter != image_queue_.end()) {
    const cv::Mat& image = iter->second;
    size_t image_size = image.total() * image.elemSize();
    if (image_size != buffer_size) {
      LOG(ERROR) << "Image size and buffer size don't match! Image size: "
                 << image_size << " Buffer size: " << buffer_size;
      return;
    }
    std::memcpy(pixel_buffer, image.data, image.total() * image.elemSize());
  } else {
    LOG(ERROR) << "Request image missing. Image id: " << image_key;
  }
}

size_t AsaRosProvider::addImageToQueue(const cv::Mat& image) {
  std::unique_lock<std::mutex> provider_lock(provider_mutex_);
  image_queue_[next_queue_id_] = image;

  VLOG(3) << "Added image " << next_queue_id_ << " to queue.\n";

  if (image_queue_.size() > max_queue_size_) {
    // Desired starting element:
    size_t desired_start = next_queue_id_ - max_queue_size_;
    // TODO: better (safer) way of doing this!
    image_queue_.erase(image_queue_.begin(), image_queue_.find(desired_start));
  }
  next_queue_id_++;
  return next_queue_id_ - 1;
}

}  // namespace asa_ros
