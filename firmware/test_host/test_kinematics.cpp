// SPDX-License-Identifier: Apache-2.0
//
// Host tests for the leg kinematics (firmware/src/kinematics.h). No framework: a failing check
// prints its location and the program exits non-zero. Run with firmware/test_host/run_tests.sh.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "kinematics.h"

using hg::FootPosition;
using hg::IkResult;
using hg::LegAngles;
using hg::Mode;
using hg::RobotPose;

namespace {

int g_checks = 0;
int g_failures = 0;
std::string g_context;

#define CHECK(cond)                                                                                \
  do {                                                                                             \
    ++g_checks;                                                                                    \
    if (!(cond)) {                                                                                 \
      ++g_failures;                                                                                \
      std::printf("%s:%d: [%s] CHECK(%s) failed\n", __FILE__, __LINE__, g_context.c_str(), #cond); \
    }                                                                                              \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                      \
  do {                                                                                             \
    ++g_checks;                                                                                    \
    const double va = static_cast<double>(a);                                                      \
    const double vb = static_cast<double>(b);                                                      \
    if (!(std::fabs(va - vb) <= (tol))) {                                                          \
      ++g_failures;                                                                                \
      std::printf("%s:%d: [%s] CHECK_NEAR(%s, %s, %g) failed: %.6f vs %.6f\n", __FILE__, __LINE__, \
                  g_context.c_str(), #a, #b, static_cast<double>(tol), va, vb);                    \
    }                                                                                              \
  } while (0)

constexpr double kPi = 3.14159265358979323846;
constexpr double kL1 = 108.0;
constexpr double kL2 = 130.0;

// Independent double-precision reference for a foot directly below the hip.
struct RefAngles {
  double hip;
  double knee;
};
RefAngles reference_below(double h, double pitch_deg) {
  const double p = pitch_deg * kPi / 180.0;
  const double x = -h * std::sin(p);
  const double z = -h * std::cos(p);
  const double r = std::hypot(x, z);
  const double knee = std::acos((kL1 * kL1 + kL2 * kL2 - r * r) / (2 * kL1 * kL2));
  const double hip = std::atan2(x, -z) + std::acos((kL1 * kL1 + r * r - kL2 * kL2) / (2 * kL1 * r));
  return {hip * 180.0 / kPi, knee * 180.0 / kPi};
}

double distance(const FootPosition& a, const FootPosition& b) {
  return std::hypot(static_cast<double>(a.x_mm) - b.x_mm, static_cast<double>(a.z_mm) - b.z_mm);
}

bool within_limits(const LegAngles& a) {
  return std::isfinite(a.hip_deg) && std::isfinite(a.knee_deg) && a.hip_deg >= hg::kHipMinDeg &&
         a.hip_deg <= hg::kHipMaxDeg && a.knee_deg >= hg::kKneeMinDeg &&
         a.knee_deg <= hg::kKneeMaxDeg;
}

// ---------------------------------------------------------------------------------------------
// Reference poses
// ---------------------------------------------------------------------------------------------

void test_stand_pose() {
  g_context = "STAND";
  RobotPose pose;
  uint8_t status = 0xFF;
  CHECK(hg::reference_pose(Mode::kStand, &pose, &status));
  CHECK(status == 0);
  const RefAngles ref = reference_below(190.0, 0.0);
  for (const LegAngles& leg : pose.legs) {
    CHECK_NEAR(leg.hip_deg, ref.hip, 0.01);
    CHECK_NEAR(leg.knee_deg, ref.knee, 0.01);
    CHECK_NEAR(leg.hip_deg, 41.2, 0.05);    // Design plan: about +41.2 deg.
    CHECK_NEAR(leg.knee_deg, 105.6, 0.05);  // Design plan: about 105.6 deg.
    const FootPosition foot = hg::leg_fk(leg);
    CHECK_NEAR(foot.x_mm, 0.0, 0.1);  // Directly below the hip axis.
    CHECK_NEAR(foot.z_mm, -190.0, 0.1);
  }
}

void test_lie_pose() {
  g_context = "LIE";
  RobotPose pose;
  uint8_t status = 0xFF;
  CHECK(hg::reference_pose(Mode::kLie, &pose, &status));
  CHECK(status == 0);
  const RefAngles ref = reference_below(75.0, 0.0);
  for (const LegAngles& leg : pose.legs) {
    CHECK_NEAR(leg.hip_deg, ref.hip, 0.01);
    CHECK_NEAR(leg.knee_deg, ref.knee, 0.01);
    CHECK_NEAR(leg.knee_deg, 35.2, 0.05);  // Design plan: about 35.2 deg.
    const FootPosition foot = hg::leg_fk(leg);
    CHECK_NEAR(foot.x_mm, 0.0, 0.1);
    CHECK_NEAR(foot.z_mm, -75.0, 0.1);
  }
}

void test_sit_pose() {
  g_context = "SIT";
  RobotPose pose;
  uint8_t status = 0xFF;
  CHECK(hg::reference_pose(Mode::kSit, &pose, &status));
  CHECK(status == 0);
  const double pitch = 25.0 * kPi / 180.0;
  const double front_h = 210.0;
  const double rear_h = front_h - 2.0 * 93.0 * std::sin(pitch);
  for (int i = 0; i < hg::kNumLegs; ++i) {
    const bool front = hg::is_front(static_cast<hg::Leg>(i));
    const LegAngles& leg = pose.legs[i];
    const double h = front ? front_h : rear_h;
    const RefAngles ref = reference_below(h, 25.0);
    CHECK_NEAR(leg.hip_deg, ref.hip, 0.01);
    CHECK_NEAR(leg.knee_deg, ref.knee, 0.01);
    // Every joint at least 5 deg inside its limits.
    CHECK(leg.hip_deg >= hg::kHipMinDeg + 5.0f && leg.hip_deg <= hg::kHipMaxDeg - 5.0f);
    CHECK(leg.knee_deg >= hg::kKneeMinDeg + 5.0f && leg.knee_deg <= hg::kKneeMaxDeg - 5.0f);
    // Rotate the FK foot back to the world: it must be on the floor, directly below the hip.
    const FootPosition f = hg::leg_fk(leg);
    const double world_dx = f.x_mm * std::cos(pitch) - f.z_mm * std::sin(pitch);
    const double world_z = h + f.x_mm * std::sin(pitch) + f.z_mm * std::cos(pitch);
    CHECK_NEAR(world_dx, 0.0, 0.1);
    CHECK_NEAR(world_z, 0.0, 0.1);
  }
  // Front legs extended, rear legs folded.
  CHECK(pose.legs[0].knee_deg > 115.0f && pose.legs[2].knee_deg < 75.0f);
  CHECK(pose.legs[0].hip_deg == pose.legs[1].hip_deg &&
        pose.legs[2].hip_deg == pose.legs[3].hip_deg);
}

void test_other_modes() {
  g_context = "WALK / TORQUE_OFF";
  RobotPose walk;
  RobotPose stand;
  CHECK(hg::reference_pose(Mode::kWalk, &walk));
  CHECK(hg::reference_pose(Mode::kStand, &stand));
  for (int i = 0; i < hg::kNumLegs; ++i) {
    CHECK(walk.legs[i].hip_deg == stand.legs[i].hip_deg);
    CHECK(walk.legs[i].knee_deg == stand.legs[i].knee_deg);
  }
  RobotPose unused;
  CHECK(!hg::reference_pose(Mode::kTorqueOff, &unused));
  CHECK(!hg::reference_pose(static_cast<Mode>(7), &unused));
}

// ---------------------------------------------------------------------------------------------
// IK / FK
// ---------------------------------------------------------------------------------------------

void test_round_trip_joint_space() {
  // Every joint pair inside the limits: FK, then IK must return the same angles (the knee-forward
  // branch is unique) and FK(IK) must land within 0.1 mm.
  g_context = "joint-space round trip";
  double worst_mm = 0.0;
  double worst_deg = 0.0;
  for (int h = -450; h <= 1000; h += 5) {
    for (int k = 300; k <= 1500; k += 5) {
      const LegAngles a{static_cast<float>(h) / 10.0f, static_cast<float>(k) / 10.0f};
      const FootPosition p = hg::leg_fk(a);
      const IkResult ik = hg::leg_ik(p);
      CHECK((ik.status & hg::kIkReachClamped) == 0);
      CHECK(within_limits(ik.angles));
      const double err = distance(hg::leg_fk(ik.angles), p);
      const double dang = std::fmax(std::fabs(ik.angles.hip_deg - a.hip_deg),
                                    std::fabs(ik.angles.knee_deg - a.knee_deg));
      worst_mm = std::fmax(worst_mm, err);
      worst_deg = std::fmax(worst_deg, dang);
      if (err > 0.1 || dang > 0.01) {
        CHECK(err <= 0.1);
        CHECK(dang <= 0.01);
        std::printf("  at hip %.1f knee %.1f: err %.4f mm, %.4f deg\n", a.hip_deg, a.knee_deg, err,
                    dang);
      }
    }
  }
  std::printf("joint-space round trip: worst %.5f mm, %.5f deg\n", worst_mm, worst_deg);
}

void test_round_trip_cartesian() {
  // A grid of targets over the whole plane: every target the IK reaches without clamping must
  // come back from FK within 0.1 mm; every result must be finite and within the joint limits.
  g_context = "cartesian round trip";
  int reached = 0;
  double worst_mm = 0.0;
  for (int xi = -1000; xi <= 1000; xi += 10) {
    for (int zi = -1000; zi <= 1000; zi += 10) {
      // -250 .. 250 mm in 2.5 mm steps.
      const FootPosition t{static_cast<float>(xi) / 4.0f, static_cast<float>(zi) / 4.0f};
      const IkResult ik = hg::leg_ik(t);
      CHECK(within_limits(ik.angles));
      if (ik.status != 0) continue;
      ++reached;
      const double err = distance(hg::leg_fk(ik.angles), t);
      worst_mm = std::fmax(worst_mm, err);
      if (err > 0.1) {
        CHECK(err <= 0.1);
        std::printf("  at (%.1f, %.1f): err %.4f mm\n", t.x_mm, t.z_mm, err);
      }
    }
  }
  // About 9,800 of the 40,401 grid points are reachable within the joint limits; guard against
  // a grid that silently tests nothing.
  CHECK(reached > 5000);
  std::printf("cartesian round trip: %d reachable targets, worst %.5f mm\n", reached, worst_mm);
}

void test_out_of_reach_clamped() {
  g_context = "too far";
  const IkResult far = hg::leg_ik({0.0f, -300.0f});
  CHECK(far.status & hg::kIkReachClamped);
  CHECK(within_limits(far.angles));
  const double far_r = std::hypot(hg::leg_fk(far.angles).x_mm, hg::leg_fk(far.angles).z_mm);
  CHECK(far_r <= hg::kMaxReachMm + 0.1);
  CHECK(far_r >= 229.0);  // Knee limit (150 deg) allows 229.96 mm.

  g_context = "too far forward";
  const IkResult fwd = hg::leg_ik({300.0f, 0.0f});
  CHECK(fwd.status & hg::kIkReachClamped);
  CHECK(within_limits(fwd.angles));

  g_context = "between knee-limit reach and reach clamp";
  const IkResult edge = hg::leg_ik({0.0f, -231.5f});
  CHECK((edge.status & hg::kIkReachClamped) == 0);
  CHECK(edge.status & hg::kIkKneeLimited);
  CHECK_NEAR(edge.angles.knee_deg, 150.0, 1e-4);

  g_context = "reach clamp bounds";
  CHECK_NEAR(hg::kMinReachMm, 27.0, 1e-6);                          // |L1 - L2| + 5 mm
  CHECK_NEAR(hg::kMaxReachMm, 233.0, 1e-6);                         // L1 + L2 - 5 mm
  CHECK(hg::leg_ik({0.0f, -232.9f}).status == hg::kIkKneeLimited);  // Inside the clamp.
  CHECK(hg::leg_ik({0.0f, -233.1f}).status & hg::kIkReachClamped);  // Just outside.
  CHECK((hg::leg_ik({0.0f, -27.1f}).status & hg::kIkReachClamped) == 0);
  CHECK(hg::leg_ik({0.0f, -26.9f}).status & hg::kIkReachClamped);
  // A clamped target keeps its direction: the hip angle equals that of the clamped point.
  CHECK_NEAR(hg::leg_ik({80.0f, -400.0f}).angles.hip_deg,
             hg::leg_ik({80.0f * 233.0f / std::hypot(80.0f, 400.0f),
                         -400.0f * 233.0f / std::hypot(80.0f, 400.0f)})
                 .angles.hip_deg,
             1e-3);

  g_context = "too close";
  const IkResult close = hg::leg_ik({0.0f, -10.0f});
  CHECK(close.status & hg::kIkReachClamped);
  CHECK(close.status & hg::kIkKneeLimited);  // 27 mm needs a 7.6 deg knee; limit is 30.
  CHECK(within_limits(close.angles));

  g_context = "degenerate targets";
  const float nan = std::nanf("");
  const FootPosition degenerate[] = {{0.0f, 0.0f}, {nan, -100.0f}, {0.0f, nan}, {1e30f, 1e30f}};
  for (const FootPosition& t : degenerate) {
    const IkResult ik = hg::leg_ik(t);
    CHECK(ik.status & hg::kIkReachClamped);
    CHECK(within_limits(ik.angles));
  }
}

void test_knee_points_forward() {
  // The knee lies forward of the hip-to-foot line for every reachable target.
  g_context = "knee forward";
  const FootPosition targets[] = {{0, -190}, {0, -75}, {60, -150}, {-60, -150}, {-100, -120}};
  for (const FootPosition& t : targets) {
    const IkResult ik = hg::leg_ik(t);
    CHECK(ik.status == 0);
    const double h = ik.angles.hip_deg * kPi / 180.0;
    const double kx = kL1 * std::sin(h);
    const double kz = -kL1 * std::cos(h);
    // 2D cross product (foot - hip) x (knee - hip) = tx * kz - tz * kx. With the foot straight
    // down, (0, -1) x (kx, kz) = kx, so a positive value means the knee is forward of the line.
    const double cross = static_cast<double>(t.x_mm) * kz - static_cast<double>(t.z_mm) * kx;
    CHECK(cross > 0.0);
  }
}

}  // namespace

int main() {
  test_stand_pose();
  test_lie_pose();
  test_sit_pose();
  test_other_modes();
  test_round_trip_joint_space();
  test_round_trip_cartesian();
  test_out_of_reach_clamped();
  test_knee_points_forward();
  std::printf("kinematics tests: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
