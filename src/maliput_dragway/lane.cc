// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput_dragway/lane.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <maliput/common/maliput_abort.h>
#include <maliput/math/saturate.h>

#include "maliput_dragway/branch_point.h"
#include "maliput_dragway/road_geometry.h"
#include "maliput_dragway/segment.h"

using std::make_unique;

namespace maliput {
namespace dragway {

Lane::Lane(const Segment* segment, const api::LaneId& id, int index, double length, double y_offset,
           const api::RBounds& lane_bounds, const api::RBounds& segment_bounds, const api::HBounds& elevation_bounds)
    : segment_(segment),
      id_(id),
      index_(index),
      length_(length),
      y_offset_(y_offset),
      lane_bounds_(lane_bounds),
      segment_bounds_(segment_bounds),
      elevation_bounds_(elevation_bounds) {
  MALIPUT_DEMAND(segment != nullptr);
  MALIPUT_DEMAND(lane_bounds_.min() >= segment_bounds_.min());
  MALIPUT_DEMAND(lane_bounds_.max() <= segment_bounds_.max());
  // TODO(liang.fok) Consider initializing this variable in the constructor's
  // initializer list so branch_point_ can be declared `const`.
  branch_point_ = make_unique<BranchPoint>(api::BranchPointId(id.string() + "_Branch_Point"), this,
                                           segment->junction()->road_geometry());
}

const api::Segment* Lane::do_segment() const { return segment_; }

void Lane::set_lane_to_left(api::Lane* lane_to_left) { lane_to_left_ = lane_to_left; }

void Lane::set_lane_to_right(api::Lane* lane_to_right) { lane_to_right_ = lane_to_right; }

const api::BranchPoint* Lane::DoGetBranchPoint(const api::LaneEnd::Which) const { return branch_point_.get(); }

const api::LaneEndSet* Lane::DoGetConfluentBranches(api::LaneEnd::Which which_end) const {
  return branch_point_->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(api::LaneEnd::Which which_end) const {
  return branch_point_->GetOngoingBranches({this, which_end});
}

std::optional<api::LaneEnd> Lane::DoGetDefaultBranch(api::LaneEnd::Which which_end) const {
  return branch_point_->GetDefaultBranch({this, which_end});
}

api::RBounds Lane::do_lane_bounds(double) const { return lane_bounds_; }

api::RBounds Lane::do_segment_bounds(double) const { return segment_bounds_; }

api::HBounds Lane::do_elevation_bounds(double, double) const { return elevation_bounds_; }

api::LanePosition Lane::DoEvalMotionDerivatives(const api::LanePosition&, const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

api::InertialPosition Lane::DoToInertialPosition(const api::LanePosition& lane_pos) const {
  return api::InertialPosition::FromXyz(
      math::Vector3{lane_pos.s(), lane_pos.r() + Lane::y_offset(), lane_pos.h()} +
      segment()->junction()->road_geometry()->inertial_to_backend_frame_translation());
}

api::Rotation Lane::DoGetOrientation(const api::LanePosition&) const {
  return api::Rotation();  // Default is Identity.
}

api::LanePositionResult Lane::DoToLanePosition(const api::InertialPosition& inertial_pos) const {
  return InertialToLaneSegmentPositionBackend(inertial_pos, true);
}

api::LanePositionResult Lane::DoToSegmentPosition(const api::InertialPosition& inertial_pos) const {
  return InertialToLaneSegmentPositionBackend(inertial_pos, false);
}

api::LanePositionResult Lane::InertialToLaneSegmentPositionBackend(const api::InertialPosition& inertial_pos,
                                                                   bool use_lane_boundaries) const {
  const math::Vector3 inertial_to_backend_frame_translation =
      segment()->junction()->road_geometry()->inertial_to_backend_frame_translation();

  const double min_x{inertial_to_backend_frame_translation.x()};
  const double max_x{length_ + inertial_to_backend_frame_translation.x()};
  const double min_y{(use_lane_boundaries ? lane_bounds_.min() : segment_bounds_.min()) + y_offset_ +
                     inertial_to_backend_frame_translation.y()};
  const double max_y{(use_lane_boundaries ? lane_bounds_.max() : segment_bounds_.max()) + y_offset_ +
                     inertial_to_backend_frame_translation.y()};
  const double min_z{elevation_bounds_.min() + inertial_to_backend_frame_translation.z()};
  const double max_z{elevation_bounds_.max() + inertial_to_backend_frame_translation.z()};

  const double x = inertial_pos.x();
  const double y = inertial_pos.y();
  const double z = inertial_pos.z();

  api::LanePositionResult result;
  result.nearest_position = {math::saturate(x, min_x, max_x), math::saturate(y, min_y, max_y),
                             math::saturate(z, min_z, max_z)};

  const double distance_unsat = (inertial_pos.xyz() - result.nearest_position.xyz()).norm();
  result.distance = std::max(0., distance_unsat);

  result.lane_position = api::LanePosition::FromSrh(
      result.nearest_position.xyz() - inertial_to_backend_frame_translation + math::Vector3{0., -y_offset_, 0.});

  return result;
}

}  // namespace dragway
}  // namespace maliput
