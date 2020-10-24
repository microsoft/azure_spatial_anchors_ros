// Copyright (c) Microsoft Corporation. All rights reserved.

#pragma once

#include <AzureSpatialAnchors.h>
#include <AzureSpatialAnchorsProvider.hpp>
#include <iosfwd>

namespace Microsoft {
namespace Azure {
namespace SpatialAnchors {
const char* to_string(const Status& status);

const char* to_string(const SessionUserFeedback& sessionUserFeedback);

namespace Provider {
std::ostream& operator<<(std::ostream& os,
                         const PoseRotationTranslationScale& pose);

std::ostream& operator<<(std::ostream& os, const Pose& pose);
}  // namespace Provider
}  // namespace SpatialAnchors
}  // namespace Azure
}  // namespace Microsoft
