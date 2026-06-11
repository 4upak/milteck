#include "ballistics.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace homework_06 {
namespace {

BallisticsInput make_reference_input()
{
  return BallisticsInput{
    .drone_x = 100.0,
    .drone_y = 100.0,
    .drone_z = 100.0,
    .target_x = 200.0,
    .target_y = 200.0,
    .attack_speed = 10.0,
    .acceleration_path = 10.0,
    .ammo_name = "VOG-17",
  };
}

TEST(Ballistics, ComputesKnownDropPoint)
{
  const AmmoTable ammo_table = default_ammo_table();
  const BallisticsInput input = make_reference_input();

  const DropSolution solution = compute_drop_solution(input, lookup_ammo(ammo_table, input.ammo_name));

  EXPECT_NEAR(solution.fire_x, 173.759, 0.01);
  EXPECT_NEAR(solution.fire_y, 173.759, 0.01);
  EXPECT_GT(solution.time_of_flight, 0.0);
  EXPECT_GT(solution.horizontal_distance, 0.0);
}

TEST(Ballistics, ReportsUnknownAmmo)
{
  const AmmoTable ammo_table = default_ammo_table();

  EXPECT_THROW((void)lookup_ammo(ammo_table, "UNKNOWN-AMMO"), BallisticsError);
}

TEST(Ballistics, GlidingAmmoProducesFinitePositiveFlight)
{
  const AmmoTable ammo_table = default_ammo_table();
  BallisticsInput input = make_reference_input();
  input.ammo_name = "GLIDING-VOG";

  const DropSolution solution = compute_drop_solution(input, lookup_ammo(ammo_table, input.ammo_name));

  EXPECT_TRUE(std::isfinite(solution.time_of_flight));
  EXPECT_TRUE(std::isfinite(solution.horizontal_distance));
  EXPECT_GT(solution.time_of_flight, 0.0);
  EXPECT_GT(solution.horizontal_distance, 0.0);
}

TEST(Ballistics, RejectsZeroDistanceToTarget)
{
  const AmmoTable ammo_table = default_ammo_table();
  BallisticsInput input = make_reference_input();
  input.target_x = input.drone_x;
  input.target_y = input.drone_y;

  EXPECT_THROW((void)compute_drop_solution(input, lookup_ammo(ammo_table, input.ammo_name)), BallisticsError);
}

TEST(Ballistics, ComputesManeuverPointWhenRunupDoesNotFit)
{
  const AmmoTable ammo_table = default_ammo_table();
  BallisticsInput input{
    .drone_x = 100.0,
    .drone_y = 50.0,
    .drone_z = 120.0,
    .target_x = 160.0,
    .target_y = 90.0,
    .attack_speed = 24.0,
    .acceleration_path = 15.0,
    .ammo_name = "VOG-17",
  };

  const DropSolution solution = compute_drop_solution(input, lookup_ammo(ammo_table, input.ammo_name));

  EXPECT_TRUE(solution.needs_maneuver);
  EXPECT_NEAR(solution.fire_x, 76.7403, 0.001);
  EXPECT_NEAR(solution.fire_y, 34.4935, 0.001);
}

}  // namespace
}  // namespace homework_06
