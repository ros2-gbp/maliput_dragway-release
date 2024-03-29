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
#include <cmath>
#include <limits>
#include <map>
#include <optional>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/check_id_indexing.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_dragway/branch_point.h"
#include "maliput_dragway/junction.h"
#include "maliput_dragway/lane.h"
#include "maliput_dragway/road_geometry.h"

namespace maliput {
namespace dragway {
namespace {

// To understand the characteristics of the geometry, consult the
// dragway::Segment and dragway::Lane detailed class overview docs.
class MaliputDragwayLaneTest : public ::testing::Test {
 public:
  MaliputDragwayLaneTest()
      : length_(100.0),
        lane_width_(6.0),
        shoulder_width_(0.5),
        maximum_height_(5.0),
        min_x_(0.),
        max_x_(length_),
        min_y_(-lane_width_ / 2 - shoulder_width_),
        max_y_(lane_width_ / 2 + shoulder_width_),
        min_z_(0.),
        max_z_(maximum_height_) {}

  // Contains expected segment r_min, segment r_max, and y_offset parameters
  // of a Dragway Lane.
  struct ExpectedLaneParameters {
    double y_offset{};
    double segment_r_min{};
    double segment_r_max{};
    double elevation_min{};
    double elevation_max{};
  };

  void MakeDragway(int num_lanes) {
    const api::RoadGeometryId road_geometry_id(std::to_string(num_lanes) + "LaneDragwayRoadGeometry");
    road_geometry_.reset(new RoadGeometry(road_geometry_id, num_lanes, length_, lane_width_, shoulder_width_,
                                          maximum_height_, kLinearTolerance, kAngularTolerance,
                                          kInertialToBackendFrameTranslation));

    const api::Junction* junction = road_geometry_->junction(0);
    ASSERT_NE(junction, nullptr);
    segment_ = junction->segment(0);
    ASSERT_NE(segment_, nullptr);
    lane_ = dynamic_cast<const Lane*>(segment_->lane(0));
    ASSERT_NE(lane_, nullptr);

    EXPECT_EQ(lane_->length(), length_);
    EXPECT_EQ(segment_->num_lanes(), num_lanes);
  }

  ExpectedLaneParameters GetExpectedLaneParameters(int num_lanes, int lane_index) const {
    ExpectedLaneParameters result{};
    if (num_lanes == 1) {
      result.y_offset = 0.0;
      result.segment_r_min = -3.5;
      result.segment_r_max = 3.5;
    } else if ((num_lanes == 2) && (lane_index == 0)) {
      result.y_offset = -3.0;
      result.segment_r_min = -3.5;
      result.segment_r_max = 9.5;
    } else if ((num_lanes == 2) && (lane_index == 1)) {
      result.y_offset = 3.0;
      result.segment_r_min = -9.5;
      result.segment_r_max = 3.5;
    } else {
      throw std::runtime_error("GetExpectedLaneParameters: bad input");
    }
    result.elevation_min = 0.0;
    result.elevation_max = 5.0;
    return result;
  }

  // Verifies the correctness of the provided `lane`.
  void VerifyLaneCorrectness(const api::Lane* lane, int num_lanes) {
    const ExpectedLaneParameters expected = GetExpectedLaneParameters(num_lanes, lane->index());

    // Tests Lane::lane_bounds().
    for (double s = 0; s < length_; s += length_ / 100) {
      const api::RBounds lane_bounds = lane->lane_bounds(s);
      EXPECT_TRUE(
          api::test::IsRBoundsClose(lane_bounds, api::RBounds(-lane_width_ / 2, lane_width_ / 2), kLinearTolerance));
    }

    EXPECT_EQ(lane->length(), length_);

    // Tests Lane::segment_bounds().
    for (double s = 0; s < length_; s += length_ / 100) {
      const api::RBounds segment_bounds = lane->segment_bounds(s);
      EXPECT_TRUE(api::test::IsRBoundsClose(
          segment_bounds, api::RBounds(expected.segment_r_min, expected.segment_r_max), kLinearTolerance));
    }

    // Tests Lane::elevation_bounds().
    for (double s = 0; s < length_; s += length_ / 10) {
      const api::RBounds segment_bounds = lane->segment_bounds(s);
      const api::HBounds elevation_bounds0 = lane->elevation_bounds(s, segment_bounds.min());
      const api::HBounds elevation_bounds1 = lane->elevation_bounds(s, segment_bounds.max());
      EXPECT_TRUE(api::test::IsHBoundsClose(
          elevation_bounds0, api::HBounds(expected.elevation_min, expected.elevation_max), kLinearTolerance));
      EXPECT_TRUE(api::test::IsHBoundsClose(
          elevation_bounds1, api::HBounds(expected.elevation_min, expected.elevation_max), kLinearTolerance));
    }

    // The following block of test code evaluates methods that take as input a
    // (s, r, h) lane position. Ideally, we want to check that the
    // method-under-test is correct for all combinations of (s, r, h). This,
    // unfortunately, it not possible since there are too many combinations of
    // (s, r, h). Instead we pick points in a grid that spans the (s, r, h)
    // state space and only check those points in the hopes that they are
    // representative of the entire state space.
    const double segment_width = expected.segment_r_max - expected.segment_r_min;
    for (double s = 0; s < length_; s += length_ / 20) {
      for (double r = expected.segment_r_min; r <= expected.segment_r_max; r += segment_width / 20) {
        for (double h = expected.elevation_min; h < expected.elevation_max;
             h += (expected.elevation_max - expected.elevation_min) * 0.1) {
          const api::LanePosition lane_position(s, r, h);

          // Tests Lane::ToInertialPosition().
          const api::InertialPosition inertial_position = lane->ToInertialPosition(lane_position);
          const double linear_tolerance = lane->segment()->junction()->road_geometry()->linear_tolerance();
          EXPECT_DOUBLE_EQ(inertial_position.x(), s);
          EXPECT_NEAR(inertial_position.y(), expected.y_offset + r, linear_tolerance);
          EXPECT_DOUBLE_EQ(inertial_position.z(), h);

          // Tests Lane::GetOrientation().
          const api::Rotation rotation = lane->GetOrientation(lane_position);
          EXPECT_TRUE(api::test::IsRotationClose(rotation, api::Rotation::FromRpy(0.0, 0.0, 0.0), kAngularTolerance));

          // Tests Lane::EvalMotionDerivatives().
          //
          // The following translational velocities can be any value. We just
          // want to verify that the same values are returned from
          // Lane::EvalMotionDerivatives().
          const double kSigma_v = 1.1;
          const double kRho_v = 2.2;
          const double kEta_v = 3.3;

          const api::LanePosition motion_derivatives =
              lane->EvalMotionDerivatives(lane_position, api::IsoLaneVelocity(kSigma_v, kRho_v, kEta_v));
          EXPECT_TRUE(api::test::IsLanePositionClose(motion_derivatives, api::LanePosition(kSigma_v, kRho_v, kEta_v),
                                                     kLinearTolerance));
        }
      }
    }
  }

  // Verifies that the branches within the provided `lane` are correct. The
  // provided `lane` must be in the provided `road_geometry`.
  void VerifyBranches(const api::Lane* lane, const RoadGeometry* road_geometry) const {
    // Verifies that the same BranchPoint covers both ends of the dragway lane.
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kStart), lane->GetBranchPoint(api::LaneEnd::kFinish));

    const api::BranchPoint* branch_point = lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(branch_point, nullptr);
    EXPECT_EQ(branch_point->id(), api::BranchPointId(lane->id().string() + "_Branch_Point"));
    EXPECT_EQ(branch_point->road_geometry(), road_geometry);

    // Verifies correctness of the confluent branches.
    {
      const api::LaneEndSet* lane_end_set_start = lane->GetConfluentBranches(api::LaneEnd::kStart);
      const api::LaneEndSet* lane_end_set_finish = lane->GetConfluentBranches(api::LaneEnd::kFinish);
      EXPECT_EQ(lane_end_set_start->size(), 1);
      EXPECT_EQ(lane_end_set_finish->size(), 1);

      const api::LaneEnd& lane_end_start = lane->GetConfluentBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_start.lane, lane);
      EXPECT_EQ(lane_end_start.end, api::LaneEnd::kStart);
      const api::LaneEnd& lane_end_finish = lane->GetConfluentBranches(api::LaneEnd::kFinish)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
      EXPECT_EQ(lane_end_finish.end, api::LaneEnd::kFinish);
    }

    // Verifies correctness of the ongoing branches.
    {
      const api::LaneEndSet* lane_end_set_start = lane->GetOngoingBranches(api::LaneEnd::kStart);
      const api::LaneEndSet* lane_end_set_finish = lane->GetOngoingBranches(api::LaneEnd::kFinish);
      EXPECT_EQ(lane_end_set_start->size(), 1);
      EXPECT_EQ(lane_end_set_finish->size(), 1);

      const api::LaneEnd& lane_end_start = lane->GetOngoingBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_start.lane, lane);
      EXPECT_EQ(lane_end_start.end, api::LaneEnd::kFinish);
      const api::LaneEnd& lane_end_finish = lane->GetOngoingBranches(api::LaneEnd::kFinish)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
      EXPECT_EQ(lane_end_finish.end, api::LaneEnd::kStart);
    }

    // Verifies correctness of the default branches.
    {
      const std::optional<api::LaneEnd> default_start_lane_end = lane->GetDefaultBranch(api::LaneEnd::kStart);
      EXPECT_TRUE(default_start_lane_end);
      EXPECT_EQ(default_start_lane_end->end, api::LaneEnd::kFinish);
      EXPECT_EQ(default_start_lane_end->lane, lane);

      const std::optional<api::LaneEnd> default_finish_lane_end = lane->GetDefaultBranch(api::LaneEnd::kFinish);
      EXPECT_TRUE(default_finish_lane_end);
      EXPECT_EQ(default_finish_lane_end->end, api::LaneEnd::kStart);
      EXPECT_EQ(default_finish_lane_end->lane, lane);
    }
  }

  void PopulateInertialPositionTestCases() {
    /*
      A figure of the one-lane dragway is shown below. The minimum and maximum
      values of the dragway' segment surface are demarcated.

      X
      Y = max_y ^  Y = min_y
      :
      |     :     |
      |     :     |
      --------+-----+-----+---------  X = max_x
      | .   :   . |
      | .   :   . |
      | .   :   . |
      | .  The  . |
      | .Dragway. |
      | .   :   . |
      | .   :   . |
      | .   :   . |
      Y <----------+-----o-----+---------  X = min_x
      |     :     |
      :
      ->|-|<- : ->|-|<-  shoulder_width
      :
      |<--:-->|      lane_width
      V
    */

    // Defines the test case values for x and y. Points both far away and close
    // to the segment area are evaluated, with a focus on edge cases.
    std::vector<double> x_values{min_x_ - 10.,   min_x_ - 1e-10, min_x_,         min_x_ + 1e-10, (max_x_ + min_x_) / 2.,
                                 max_x_ - 1e-10, max_x_,         max_x_ + 1e-10, max_x_ + 10.};
    std::vector<double> y_values{min_y_ - 10.,   min_y_ - 1e-10, min_y_,         min_y_ + 1e-10, (max_y_ + min_y_) / 2.,
                                 max_y_ - 1e-10, max_y_,         max_y_ + 1e-10, max_y_ + 10.};
    std::vector<double> z_values{min_z_ - 10.,   min_z_ - 1e-10, min_z_,         min_z_ + 1e-10, (max_z_ + min_z_) / 2.,
                                 max_z_ - 1e-10, max_z_,         max_z_ + 1e-10, max_z_ + 10.};
    x_test_cases_ = x_values;
    y_test_cases_ = y_values;
    z_test_cases_ = z_values;
  }

  std::unique_ptr<RoadGeometry> road_geometry_;
  const api::Segment* segment_{nullptr};
  const Lane* lane_{nullptr};

  const double length_{};
  const double lane_width_{};
  const double shoulder_width_{};
  const double maximum_height_{};

  const double min_x_{};
  const double max_x_{};
  const double min_y_{};
  const double max_y_{};
  const double min_z_{};
  const double max_z_{};

  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the segment r_min and r_max.
  const double kLinearTolerance = 1e-15;

  const double kAngularTolerance = std::numeric_limits<double>::epsilon();

  const math::Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};

  // For tests involving ToLanePosition().
  std::vector<double> x_test_cases_{};
  std::vector<double> y_test_cases_{};
  std::vector<double> z_test_cases_{};
};

/*
 Tests a dragway containing one lane. It is arranged as shown below in the world
 frame:

               x
               ^
      |<-------|------->|    segment r_max / r_min
      | |<-----|----->| |    lane    r_max / r_min
      -------------------    s = length_
      | |      ^      | |
      | |      :      | |
      | |      ^      | |
      | |      :      | |
  y <----------o---------->  s = 0
               |
               V
 */
TEST_F(MaliputDragwayLaneTest, SingleLane) {
  const int kNumLanes = 1;
  MakeDragway(kNumLanes);

  EXPECT_EQ(segment_->id(), api::SegmentId("Dragway_Segment_ID"));
  EXPECT_EQ(lane_->segment(), segment_);

  VerifyLaneCorrectness(lane_, kNumLanes);
  VerifyBranches(lane_, road_geometry_.get());
  EXPECT_TRUE(api::test::CheckIdIndexing(road_geometry_.get()));
}

/*
 Tests a dragway containing two lanes. The two lanes are arranged as shown below
 in the world frame:

                              x
                              ^
                              |
              |<-------|--------------------->|  lane 1 segment r_max / r_min
              |<---------------------|------->|  lane 0 segment r_max / r_min
              | |<-----|----->|<-----|----->| |  lane           r_max / r_min
              ----------------|----------------  s = length_
              | |      ^      |      ^      | |
              | |      :      |      :      | |
              | |      ^      |      ^      | |
              | |      :      |      :      | |  s = 0
      y <---------------------o------------------------------>
                    index 1   |    index 0
                              V
 */
TEST_F(MaliputDragwayLaneTest, TwoLaneDragway) {
  const int kNumLanes = 2;
  MakeDragway(kNumLanes);

  EXPECT_EQ(segment_->id(), api::SegmentId("Dragway_Segment_ID"));

  for (int i = 0; i < kNumLanes; ++i) {
    const api::Lane* lane = segment_->lane(i);
    ASSERT_NE(lane, nullptr);
    VerifyLaneCorrectness(lane, kNumLanes);
    VerifyBranches(lane, road_geometry_.get());
  }
  EXPECT_TRUE(api::test::CheckIdIndexing(road_geometry_.get()));
}

// Tests dragway::RoadGeometry::ToRoadPosition() using a two-lane dragway where
// the x,y projection of all geographic positions provided to it are in the
// road' segment region. This unit test also verifies that
// dragway::RoadGeometry::IsInertialPositionOnDragway() does not incorrectly return
// false.
TEST_F(MaliputDragwayLaneTest, TestToRoadPositionOnRoad) {
  MakeDragway(2 /* num lanes */);

  // Spot checks geographic positions on lane 0 and the right shoulder with a
  // focus on edge cases.
  for (double x = 0; x <= length_; x += length_ / 2) {
    for (double y = -lane_width_ - shoulder_width_; y <= 0; y += (lane_width_ + shoulder_width_) / 2) {
      for (double z = 0; z <= maximum_height_; z += maximum_height_ / 2.) {
        const api::RoadPositionResult result = road_geometry_->ToRoadPosition(api::InertialPosition(x, y, z));
        const api::Lane* expected_lane = road_geometry_->junction(0)->segment(0)->lane(0);
        EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, api::InertialPosition(x, y, z),
                                                       kLinearTolerance));
        EXPECT_DOUBLE_EQ(result.distance, 0);
        EXPECT_EQ(result.road_position.lane, expected_lane);
        EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                                   api::LanePosition(x, y + lane_width_ / 2, z), kLinearTolerance));
      }
    }
  }

  // Spot checks geographic positions on lane 1 and the left shoulder with a
  // focus on edge cases.
  for (double x = 0; x <= length_; x += length_ / 2) {
    for (double y = 0; y <= lane_width_ + shoulder_width_; y += (lane_width_ + shoulder_width_) / 2) {
      for (double z = 0; z <= maximum_height_; z += maximum_height_ / 2.) {
        const api::RoadPositionResult result = road_geometry_->ToRoadPosition(api::InertialPosition(x, y, z));
        const int lane_index = (y == 0 ? 0 : 1);
        const api::Lane* expected_lane = road_geometry_->junction(0)->segment(0)->lane(lane_index);
        EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, api::InertialPosition(x, y, z),
                                                       kLinearTolerance));
        EXPECT_DOUBLE_EQ(result.distance, 0);
        EXPECT_EQ(result.road_position.lane, expected_lane);
        if (y == 0) {
          EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                                     api::LanePosition(x, y + lane_width_ / 2, z), kLinearTolerance));
        } else {
          EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                                     api::LanePosition(x, y - lane_width_ / 2, z), kLinearTolerance));
        }
      }
    }
  }
}

// Tests dragway::RoadGeometry::ToRoadPosition() using a two-lane dragway where
// the x,y projection of all geographic positions provided to it may be outside
// of the road' segment region. This unit test also verifies that
// dragway::RoadGeometry::IsInertialPositionOnDragway() does not incorrectly return
// false.
TEST_F(MaliputDragwayLaneTest, TestToRoadPositionOffRoad) {
  MakeDragway(2 /* num lanes */);

  // Computes the bounds of the road' segment region.
  const double x_max = length_;
  const double x_min = 0;
  const double y_max = lane_width_ + shoulder_width_;
  const double y_min = -y_max;
  const double z_min = 0;
  const double z_max = maximum_height_;

  // Spot checks a region that is a superset of the road' segment bounds with
  // a focus on edge cases.
  const double test_x_min = x_min - 10;
  const double test_x_max = x_max + 10;
  const double test_y_min = y_min - 10;
  const double test_y_max = y_max + 10;
  const double test_z_min = z_min - 10;
  const double test_z_max = z_max + 10;

  // Defines the test case values for x and y. Points both far away and close to
  // the segment area are evaluated.
  const std::vector<double> x_test_cases{test_x_min, x_min - 1e-10, x_max + 1e-10, test_x_max};
  const std::vector<double> y_test_cases{test_y_min, y_min - 1e-10, y_max + 1e-10, test_y_max};
  const std::vector<double> z_test_cases{test_z_min, z_min - 1e-10, z_max + 1e-10, test_z_max};
  // TODO(maddog@tri.global)  Needs test cases where *one* coordinate is
  //                          out-of-bounds.  (Currently, only tests when
  //                          *all* coordinates are out-of-bounds.)
  for (const auto x : x_test_cases) {
    for (const auto y : y_test_cases) {
      for (const auto z : z_test_cases) {
        const api::RoadPositionResult result = road_geometry_->ToRoadPosition(api::InertialPosition(x, y, z));
        EXPECT_LE(result.nearest_position.x(), x_max);
        EXPECT_GE(result.nearest_position.x(), x_min);
        EXPECT_LE(result.nearest_position.y(), y_max);
        EXPECT_GE(result.nearest_position.y(), y_min);
        EXPECT_LE(result.nearest_position.z(), z_max);
        EXPECT_GE(result.nearest_position.z(), z_min);

        api::InertialPosition expected_nearest_position;
        expected_nearest_position.set_x(x);
        expected_nearest_position.set_y(y);
        expected_nearest_position.set_z(z);
        if (x < x_min) {
          expected_nearest_position.set_x(x_min);
        }
        if (x > x_max) {
          expected_nearest_position.set_x(x_max);
        }
        if (y < y_min) {
          expected_nearest_position.set_y(y_min);
        }
        if (y > y_max) {
          expected_nearest_position.set_y(y_max);
        }
        if (z < z_min) {
          expected_nearest_position.set_z(z_min);
        }
        if (z > z_max) {
          expected_nearest_position.set_z(z_max);
        }

        EXPECT_TRUE(
            api::test::IsInertialPositionClose(result.nearest_position, expected_nearest_position, kLinearTolerance));
        // TODO(maddog@tri.global)  Should test for explicit correct distance.
        EXPECT_LT(0, result.distance);
        const int expected_lane_index = (y > 0 ? 1 : 0);
        const Lane* expected_lane =
            dynamic_cast<const Lane*>(road_geometry_->junction(0)->segment(0)->lane(expected_lane_index));
        EXPECT_EQ(result.road_position.lane, expected_lane);
        EXPECT_TRUE(api::test::IsLanePositionClose(
            result.road_position.pos,
            api::LanePosition(expected_nearest_position.x(), expected_nearest_position.y() - expected_lane->y_offset(),
                              expected_nearest_position.z()),
            kLinearTolerance));
      }
    }
  }
}

// Tests dragway::Lane::ToSegmentPosition() using geographic positions whose
// projections onto the XY plane reside within the lane's region.
TEST_F(MaliputDragwayLaneTest, TestToSegmentPosition) {
  MakeDragway(1 /* num lanes */);
  PopulateInertialPositionTestCases();

  for (const double x : x_test_cases_) {
    for (const double y : y_test_cases_) {
      for (const double z : z_test_cases_) {
        const api::LanePositionResult result = lane_->ToSegmentPosition(api::InertialPosition(x, y, z));
        api::InertialPosition expected_nearest_position(x, y, z);
        if (x < min_x_) {
          expected_nearest_position.set_x(min_x_);
        }
        if (x > max_x_) {
          expected_nearest_position.set_x(max_x_);
        }
        if (y < min_y_) {
          expected_nearest_position.set_y(min_y_);
        }
        if (y > max_y_) {
          expected_nearest_position.set_y(max_y_);
        }
        if (z < min_z_) {
          expected_nearest_position.set_z(min_z_);
        }
        if (z > max_z_) {
          expected_nearest_position.set_z(max_z_);
        }

        EXPECT_TRUE(
            api::test::IsInertialPositionClose(result.nearest_position, expected_nearest_position, kLinearTolerance));
        // TODO(maddog@tri.global)  Should test for explicit correct distance.
        EXPECT_GE(result.distance, 0);
        EXPECT_TRUE(api::test::IsLanePositionClose(
            result.lane_position,
            api::LanePosition(expected_nearest_position.x(), expected_nearest_position.y() - lane_->y_offset(),
                              expected_nearest_position.z()),
            kLinearTolerance));
      }
    }
  }
}

// Holds RoadPositionResult expected arguments.
struct RoadPositionResultExpectation {
  api::LanePosition lane_position;
  api::InertialPosition nearest_position;
  double distance{};
};

// Test parameters and expected results for MaliputDragwayFindRoadPositionTest
struct FindRoadPositionsTestParameters {
  int dragway_num_lanes{};
  double radius{};
  api::InertialPosition inertial_position;
  std::map<api::LaneId, RoadPositionResultExpectation> expected_results;
};

// Operator overload for GTest log.
std::ostream& operator<<(std::ostream& os, const FindRoadPositionsTestParameters& test_parameters) {
  os << "{dragway_num_lanes: " << test_parameters.dragway_num_lanes << ", radius: " << test_parameters.radius
     << ", inertial_position: " << test_parameters.inertial_position << ", expected_results: {...}}";
  return os;
}

// To understand the characteristics of the geometry, consult the
// dragway::Segment and dragway::Lane detailed class overview docs.
class MaliputDragwayFindRoadPositionTest : public ::testing::TestWithParam<FindRoadPositionsTestParameters> {
 public:
  const double kLength = 100.;
  const double kLaneWidth = 6.;
  const double kShoulderWidth = 0.5;
  const double kMaximumHeight = 5.0;

  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the segment r_min and r_max.
  const double kLinearTolerance = 1e-15;

  const double kAngularTolerance = std::numeric_limits<double>::epsilon();

  const math::Vector3 kInertialToBackendFrameTranslation{0., 0., 0.};

  static std::vector<FindRoadPositionsTestParameters> TestParameters();

  void MakeDragway(int num_lanes) {
    const api::RoadGeometryId road_geometry_id(std::to_string(num_lanes) + "LaneDragwayRoadGeometry");
    road_geometry_.reset(new RoadGeometry(road_geometry_id, num_lanes, kLength, kLaneWidth, kShoulderWidth,
                                          kMaximumHeight, kLinearTolerance, kAngularTolerance,
                                          kInertialToBackendFrameTranslation));
  }

 protected:
  void SetUp() override {
    const FindRoadPositionsTestParameters test_parameters = GetParam();
    dragway_num_lanes_ = test_parameters.dragway_num_lanes;
    radius_ = test_parameters.radius;
    inertial_position_ = test_parameters.inertial_position;
    expected_results_ = test_parameters.expected_results;
  }

  int dragway_num_lanes_{};
  double radius_{};
  api::InertialPosition inertial_position_;
  std::map<api::LaneId, RoadPositionResultExpectation> expected_results_;
  std::unique_ptr<RoadGeometry> road_geometry_;
};

std::vector<FindRoadPositionsTestParameters> MaliputDragwayFindRoadPositionTest::TestParameters() {
  return {
      // For the following set of test cases, an Inertial Frame position is placed
      // outside the road volume on purpose expecting the method to derive the
      // right LanePosition for each coordinate (s, r, and h), not only a mapping
      // to the ground.

      // { Single-lane Dragway tests cases.
      // Point is outside the RoadGeometry volume with zero radius, no possible
      // match.
      {1, 0., api::InertialPosition(10., 20., 30.), {}},
      // Point is outside the RoadGeometry volume with small radius, no possible
      // match.
      {1, 10., api::InertialPosition(10., 20., 30.), {}},
      // Point is outside the RoadGeometry volume with big enough radius to produce
      // a match.
      {
          1,
          35.,
          api::InertialPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 3., 5.), 30.23243291566195}},
          },
      },
      // Same result as before but with a relatively big radius.
      {
          1,
          1000.,
          api::InertialPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 3., 5.), 30.23243291566195}},
          },
      },
      // Biggest radius possible, the closest point to Inertial position is
      // expected for all lanes (just one).
      {
          1,
          std::numeric_limits<double>::infinity(),
          api::InertialPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 3., 5.), 30.23243291566195}},
          },
      },
      // } Single-lane Dragway tests cases.

      // { Multiple-lane Dragway test cases.
      // Point is outside the RoadGeometry volume with zero radius, no possible
      // match.
      {3, 0., api::InertialPosition(10., 20., 30.), {}},
      // Point is outside the RoadGeometry volume with small radius, no possible
      // match.
      {3, 10., api::InertialPosition(10., 20., 30.), {}},
      // Point is outside the RoadGeometry volume with big enough radius to
      // produce a match.
      {
          3,
          35.,
          api::InertialPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., -3, 5.), 33.97057550292606}},
              {api::LaneId("Dragway_Lane_1"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 3, 5.), 30.23243291566195}},
              {api::LaneId("Dragway_Lane_2"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 9, 5.), 27.313000567495326}},
          },
      },
      // Point is within the RoadGeometry volume, specifically at the edge of Dragway Lane 2.
      {
          3,
          1.,
          api::InertialPosition(10., 9., 5.),
          {
              {api::LaneId("Dragway_Lane_2"), {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 9., 5.), 0.}},
          },
      },
      // Point is within the RoadGeometry volume, specifically between Dragway Lane 2 and 1.
      {
          3,
          1.,
          api::InertialPosition(10., 3., 5.),
          {
              {api::LaneId("Dragway_Lane_1"), {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 3., 5.), 0.}},
              {api::LaneId("Dragway_Lane_2"),
               {api::LanePosition(10., -3., 5.), api::InertialPosition(10., 3., 5.), 0.}},
          },
      },
      // Point is outside the RoadGeometry volume with infinite radius, all lanes
      // are expected to produce a result.
      {
          3,
          std::numeric_limits<double>::infinity(),
          api::InertialPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., -3., 5.), 33.97057550292606}},
              {api::LaneId("Dragway_Lane_1"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 3., 5.), 30.23243291566195}},
              {api::LaneId("Dragway_Lane_2"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 9., 5.), 27.313000567495326}},
          },
      },
      // Point is within the RoadGeometry volume, with infinite radius, all lanes
      // are expected to produce a result.
      {
          3,
          std::numeric_limits<double>::infinity(),
          api::InertialPosition(10., 9., 5.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3., 5.), api::InertialPosition(10., -3., 5.), 12.}},
              {api::LaneId("Dragway_Lane_1"), {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 3., 5.), 6.}},
              {api::LaneId("Dragway_Lane_2"), {api::LanePosition(10., 3., 5.), api::InertialPosition(10., 9., 5.), 0.}},
          },
      },
      // } Multiple-lane Dragway test cases.
  };
}

// Evaluates RoadGeometry::FindRoadPositions() using a single-lane Dragway with
// different radii and Inertial positions.
TEST_P(MaliputDragwayFindRoadPositionTest, FindRoadPositionsWithOneLane) {
  MakeDragway(dragway_num_lanes_);

  std::vector<api::RoadPositionResult> road_position_results =
      road_geometry_->FindRoadPositions(inertial_position_, radius_);
  EXPECT_EQ(road_position_results.size(), expected_results_.size());

  // Evaluates RoadPosition matching.
  for (const api::RoadPositionResult& dut : road_position_results) {
    EXPECT_TRUE(expected_results_.find(dut.road_position.lane->id()) != expected_results_.end());
    EXPECT_TRUE(api::test::IsLanePositionClose(
        dut.road_position.pos, expected_results_[dut.road_position.lane->id()].lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(
        dut.nearest_position, expected_results_[dut.road_position.lane->id()].nearest_position, kLinearTolerance));
    EXPECT_NEAR(dut.distance, expected_results_[dut.road_position.lane->id()].distance, kLinearTolerance);
  }
}

INSTANTIATE_TEST_CASE_P(MaliputDragwayFindLaneMappingsTestGroup, MaliputDragwayFindRoadPositionTest,
                        ::testing::ValuesIn(MaliputDragwayFindRoadPositionTest::TestParameters()));

class DragwayWithInertialToBackendFrameTranslation : public ::testing::Test {
 protected:
  void SetUp() override { lane_ = dut_.junction(0)->segment(0)->lane(0); }
  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the segment r_min and r_max.
  const double kLinearTolerance{1e-15};
  const double kAngularTolerance{std::numeric_limits<double>::epsilon()};
  const RoadGeometry dut_{api::RoadGeometryId("non zero translation"),
                          1 /* num_lanes */,
                          10. /* length */,
                          5. /* lane_width */,
                          0. /* shoulder_width */,
                          5. /* maximum_height */,
                          kLinearTolerance,
                          kAngularTolerance,
                          {1.2, -3.4, 0.5} /* inertial to backend frame translation */};
  const api::Lane* lane_{};
};

TEST_F(DragwayWithInertialToBackendFrameTranslation, ToInertialPosition) {
  EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(1.2, -3.4, 0.5),
                                                 lane_->ToInertialPosition({0., 0., 0.}), kLinearTolerance));
  EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(11.2, -3.4, 0.5),
                                                 lane_->ToInertialPosition({10., 0., 0.}), kLinearTolerance));
}

TEST_F(DragwayWithInertialToBackendFrameTranslation, ToSegmentPosition) {
  {
    const api::LanePositionResult result = lane_->ToSegmentPosition({1.2, -3.4, 0.5});
    EXPECT_TRUE(api::test::IsLanePositionClose(api::LanePosition(0., 0., 0.), result.lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(1.2, -3.4, 0.5), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(0., result.distance, kLinearTolerance);
  }
  {
    const api::LanePositionResult result = lane_->ToSegmentPosition({11.2, -3.4, 0.5});
    EXPECT_TRUE(api::test::IsLanePositionClose(api::LanePosition(10., 0., 0.), result.lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(11.2, -3.4, 0.5), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(0., result.distance, kLinearTolerance);
  }
}

TEST_F(DragwayWithInertialToBackendFrameTranslation, ToRoadPosition) {
  {
    const api::RoadPositionResult result = dut_.ToRoadPosition({1.2, -3.4, 0.5}, std::nullopt);
    EXPECT_EQ(lane_, result.road_position.lane);
    EXPECT_TRUE(
        api::test::IsLanePositionClose(api::LanePosition(0., 0., 0.), result.road_position.pos, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(1.2, -3.4, 0.5), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(0., result.distance, kLinearTolerance);
  }
  {
    const api::RoadPositionResult result = dut_.ToRoadPosition({11.2, -3.4, 0.5});
    EXPECT_EQ(lane_, result.road_position.lane);
    EXPECT_TRUE(
        api::test::IsLanePositionClose(api::LanePosition(10., 0., 0.), result.road_position.pos, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(11.2, -3.4, 0.5), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(0., result.distance, kLinearTolerance);
  }
}

class DragwayToLaneAndSegmentPositionTest : public ::testing::Test {
 protected:
  void SetUp() override { lane_ = dut_.junction(0)->segment(0)->lane(0); }
  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the segment r_min and r_max.
  const double kLinearTolerance{1e-15};
  const double kAngularTolerance{std::numeric_limits<double>::epsilon()};
  const RoadGeometry dut_{api::RoadGeometryId("non zero translation"),
                          4 /* num_lanes */,
                          10. /* length */,
                          5. /* lane_width */,
                          0. /* shoulder_width */,
                          5. /* maximum_height */,
                          kLinearTolerance,
                          kAngularTolerance,
                          {} /* inertial to backend frame translation */};
  const api::Lane* lane_{};
};

TEST_F(DragwayToLaneAndSegmentPositionTest, ToLanePosition) {
  {
    // Out of the segment bounds.
    api::LanePositionResult result = lane_->ToLanePosition({5., 45., 0.});
    EXPECT_TRUE(api::test::IsLanePositionClose(api::LanePosition(5., 2.5, 0.), result.lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(5., -5., 0.), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(50., result.distance, kLinearTolerance);

    result = lane_->ToSegmentPosition({5., 45., 0.});
    EXPECT_TRUE(
        api::test::IsLanePositionClose(api::LanePosition(5., 17.5, 0.), result.lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(5., 10., 0.), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(35., result.distance, kLinearTolerance);
  }
  {
    // Out of the lane bounds, within segment bounds
    api::LanePositionResult result = lane_->ToLanePosition({5., 5., 0.});
    EXPECT_TRUE(api::test::IsLanePositionClose(api::LanePosition(5., 2.5, 0.), result.lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(5., -5., 0.), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(10., result.distance, kLinearTolerance);

    result = lane_->ToSegmentPosition({5., 5., 0.});
    EXPECT_TRUE(
        api::test::IsLanePositionClose(api::LanePosition(5., 12.5, 0.), result.lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(5., 5., 0.), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(0., result.distance, kLinearTolerance);
  }
  {
    // Within the lane bounds.
    api::LanePositionResult result = lane_->ToLanePosition({5., -5., 0.});
    EXPECT_TRUE(api::test::IsLanePositionClose(api::LanePosition(5., 2.5, 0.), result.lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(5., -5., 0.), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(0., result.distance, kLinearTolerance);

    result = lane_->ToSegmentPosition({5., -5., 0.});
    EXPECT_TRUE(api::test::IsLanePositionClose(api::LanePosition(5., 2.5, 0.), result.lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(api::InertialPosition(5., -5., 0.), result.nearest_position,
                                                   kLinearTolerance));
    EXPECT_NEAR(0., result.distance, kLinearTolerance);
  }
}

}  // namespace
}  // namespace dragway
}  // namespace maliput
