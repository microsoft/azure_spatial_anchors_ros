// Copyright (c) Microsoft Corporation. All rights reserved.
#include <string_view>
#include <iostream>

#include "asa_ros/internal/asa_helper.h"

namespace Microsoft {
namespace Azure {
namespace SpatialAnchors {
const char* to_string(const Status& status) {
  switch (status) {
    case Status::OK:
      return "Status::OK";
    case Status::Failed:
      return "Status::Failed";
    case Status::ObjectDisposed:
      return "Status::ObjectDisposed";
    case Status::OutOfMemory:
      return "Status::OutOfMemory";
    case Status::InvalidArgument:
      return "Status::InvalidArgument";
    case Status::OutOfRange:
      return "Status::OutOfRange";
    case Status::NotImplemented:
      return "Status::NotImplemented";
    case Status::KeyNotFound:
      return "Status::KeyNotFound";
    case Status::MetadataTooLarge:
      return "Status::MetadataTooLarge";
    case Status::ApplicationNotAuthenticated:
      return "Status::ApplicationNotAuthenticated";
    case Status::ApplicationNotAuthorized:
      return "Status::ApplicationNotAuthorized";
    case Status::ConcurrencyViolation:
      return "Status::ConcurrencyViolation";
    case Status::NotEnoughSpatialData:
      return "Status::NotEnoughSpatialData";
    case Status::NoSpatialLocationHint:
      return "Status::NoSpatialLocationHint";
    case Status::CannotConnectToServer:
      return "Status::CannotConnectToServer";
    case Status::ServerError:
      return "Status::ServerError";
    case Status::AlreadyAssociatedWithADifferentStore:
      return "Status::AlreadyAssociatedWithADifferentStore";
    case Status::AlreadyExists:
      return "Status::AlreadyExists";
    case Status::NoLocateCriteriaSpecified:
      return "Status::NoLocateCriteriaSpecified";
    case Status::NoAccessTokenSpecified:
      return "Status::NoAccessTokenSpecified";
    case Status::UnableToObtainAccessToken:
      return "Status::UnableToObtainAccessToken";
    case Status::TooManyRequests:
      return "Status::TooManyRequests";
    case Status::LocateCriteriaMissingRequiredValues:
      return "Status::LocateCriteriaMissingRequiredValues";
    case Status::LocateCriteriaInConflict:
      return "Status::LocateCriteriaInConflict";
    case Status::LocateCriteriaInvalid:
      return "Status::LocateCriteriaInvalid";
    case Status::LocateCriteriaNotSupported:
      return "Status::LocateCriteriaNotSupported";
    case Status::Unknown:
      return "Status::Unknown";
    case Status::HttpTimeout:
      return "Status::HttpTimeout";
    default:
      return "**Unknown**";
  }
}

const char* to_string(const SessionUserFeedback& sessionUserFeedback) {
  switch (sessionUserFeedback) {
    case SessionUserFeedback::None:
      return "SessionUserFeedback::None";
    case SessionUserFeedback::NotEnoughMotion:
      return "SessionUserFeedback::NotEnoughMotion";
    case SessionUserFeedback::MotionTooQuick:
      return "SessionUserFeedback::MotionTooQuick";
    case SessionUserFeedback::NotEnoughFeatures:
      return "SessionUserFeedback::NotEnoughFeatures";
    default:
      return "**Unknown**";
  }
}

namespace Provider {

std::ostream& operator<<(std::ostream& os,
                         const PoseRotationTranslationScale& pose) {
  char buffer[15];
  auto format = [&](const float& f) -> std::string {
    std::snprintf(buffer, 15, "%10.6f", f);
    return buffer;
  };
  const auto& rotation = pose.Rotation();
  const auto& translation = pose.Translation();
  const auto& scale = pose.Scale();
  os << "[" << format(rotation[0]) << "," << format(rotation[1]) << ","
     << format(rotation[2]) << " ] [" << format(translation[0]) << " ]"
     << format(scale) << "\n";
  os << "[" << format(rotation[3]) << "," << format(rotation[4]) << ","
     << format(rotation[5]) << " ] [" << format(translation[1]) << " ]\n";
  os << "[" << format(rotation[6]) << "," << format(rotation[7]) << ","
     << format(rotation[8]) << " ] [" << format(translation[2]) << " ]\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Pose& pose) {
  if (const PoseRotationTranslationScale* typedPose =
          pose.As<PoseRotationTranslationScale>()) {
    return os << *typedPose;
  }

  return os;
        }
    }
}}}
