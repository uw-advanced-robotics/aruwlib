/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "tap/algorithms/ballistics.hpp"

using namespace tap::algorithms::ballistics;
using namespace tap::units;
using namespace tap::units::literals;
using namespace tap::units::math;

TEST(Ballistics, quadraticKinematicProjection_dt_zero_pos_unmoving)
{
    EXPECT_EQ(1_m, AbstractKinematicState::quadraticKinematicProjection(0_s, 1_m, 2_mps, 3_mps2));
}

TEST(
    Ballistics,
    quadraticKinematicProjection_position_constantly_increasing_when_vel_positive_acc_zero)
{
    EXPECT_NEAR(
        2,
        AbstractKinematicState::quadraticKinematicProjection(1_s, 1_m, 1_mps, 0_mps2).valueOf(),
        1E-5);
}

TEST(Ballistics, quadraticKinematicProjection_position_increases_quadratically_when_acc_positive)
{
    EXPECT_NEAR(
        1,
        AbstractKinematicState::quadraticKinematicProjection(1_s, 0_m, 0_mps, 2_mps2).valueOf(),
        1e-5);
    EXPECT_NEAR(
        4,
        AbstractKinematicState::quadraticKinematicProjection(2_s, 0_m, 0_mps, 2_mps2).valueOf(),
        1e-5);
    EXPECT_NEAR(
        9,
        AbstractKinematicState::quadraticKinematicProjection(3_s, 0_m, 0_mps, 2_mps2).valueOf(),
        1e-5);
}

TEST(Ballistics, projectForward_returns_constant_delta_position_when_vel_positive_no_acc)
{
    SecondOrderKinematicState kinematicState(
        Vector3Position(1_m, 1_m, 1_m),
        Vector3Velocity(1_mps, 1_mps, 1_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2));

    auto position1 = kinematicState.projectForward(1_s);
    auto position2 = kinematicState.projectForward(2_s);
    auto position3 = kinematicState.projectForward(3_s);

    auto diff12 = position2 - position1;
    auto diff23 = position3 - position2;

    EXPECT_NEAR(diff12.x.valueOf(), diff23.x.valueOf(), 1E-5);
    EXPECT_NEAR(diff12.y.valueOf(), diff23.y.valueOf(), 1E-5);
    EXPECT_NEAR(diff12.z.valueOf(), diff23.z.valueOf(), 1E-5);
}

TEST(Ballistics, computeTravelTime_horizontal_distance_only_bulletVelocity_10)
{
    Vector3Position targetPosition(10_m, 0_m, 0_m);

    Time travelTime = 0_s;
    Angle turretPitch = 0_rad;
    bool timeFound = computeTravelTime(targetPosition, 20_mps, &travelTime, &turretPitch);

    // 20 m/s velocity, 10 m target distance, time should be slightly greater than .5
    EXPECT_EQ(true, timeFound);
    EXPECT_LT(0.5_s, travelTime);
    EXPECT_GT(0.75_s, travelTime);
}

TEST(Ballistics, computeTravelTime_vertical_distance_only_hits_target)
{
    Vector3Position targetPosition(0_m, 0_m, 10_m);
    Time travelTime = 0_s;
    Angle turretPitch = 0_rad;
    bool timeFound = computeTravelTime(targetPosition, 20_mps, &travelTime, &turretPitch);

    EXPECT_EQ(true, timeFound);
}

TEST(Ballistics, computeTravelTime_vertical_distance_only_no_hit)
{
    Vector3Position targetPosition(0_m, 0_m, 10_m);
    Time travelTime = 0_s;
    Angle turretPitch = 0_rad;
    bool timeFound = computeTravelTime(targetPosition, 10_mps, &travelTime, &turretPitch);

    EXPECT_EQ(false, timeFound);
}

TEST(Ballistics, computeTravelTime_bulletVelocity_0)
{
    Vector3Position targetPosition(3_m, 3_m, 2_m);

    Time travelTime = 0_s;
    Angle turretPitch = 0_rad;
    bool timeFound = computeTravelTime(targetPosition, 0_mps, &travelTime, &turretPitch);

    EXPECT_EQ(false, timeFound);
}

TEST(Ballistics, computeTravelTime_bulletVelocity_large)
{
    Vector3Position targetPosition(10_m, 10_m, 0_m);

    Time travelTime = 0_s;
    Angle turretPitch = 0_rad;
    bool timeFound = computeTravelTime(targetPosition, 1E9_mps, &travelTime, &turretPitch);

    EXPECT_EQ(true, timeFound);
    EXPECT_NEAR(0, travelTime.valueOf(), 1E-5);
    EXPECT_NEAR(0, turretPitch.valueOf(), 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_stationary)
{
    SecondOrderKinematicState targetState{
        Vector3Position(10_m, 0_m, 0_m),
        Vector3Velocity(0_mps, 0_mps, 0_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2)};

    Angle turretPitch = 0_rad;
    Angle turretYaw = 0_rad;
    Time timeOfFlight = 0_s;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        1E6_mps,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_NEAR(0, turretPitch.valueOf(), 1E-5);
    EXPECT_NEAR(0, turretYaw.valueOf(), 1E-5);
    EXPECT_NEAR(0, timeOfFlight.valueOf(), 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_moving_away_from_turret)
{
    SecondOrderKinematicState targetState{
        Vector3Position(10_m, 0_m, 0_m),
        Vector3Velocity(1_mps, 0_mps, 0_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2)};

    Angle turretPitch = 0_rad;
    Angle turretYaw = 0_rad;
    Time timeOfFlight = 0_s;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30_mps,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_LT(-45_deg, turretPitch);
    EXPECT_NEAR(0, turretYaw.valueOf(), 1E-5);
    EXPECT_NEAR(10. / 30., timeOfFlight.valueOf(), 0.15);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_moving_perpendicular_to_turret)
{
    SecondOrderKinematicState targetState(
        Vector3Position(10_m, 0_m, 0_m),
        Vector3Velocity(0_mps, 1_mps, 0_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2));

    Angle turretPitch = 0_rad;
    Angle turretYaw = 0_rad;
    Time timeOfFlight = 0_s;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30_mps,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_LT(0_deg, turretYaw);
    EXPECT_GT(30_deg, turretYaw);
    EXPECT_LT(-45_deg, turretPitch);
    EXPECT_NEAR(10. / 30., timeOfFlight.valueOf(), 0.15);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_velocity_increases_lead_position_decreases)
{
    SecondOrderKinematicState targetState(
        Vector3Position(10_m, 0_m, 0_m),
        Vector3Velocity(0_mps, 1_mps, 0_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2));

    Angle turretPitch30ms{0}, turretYaw30ms{0};
    Time timeOfFlight30ms{0};
    Angle turretPitch40ms{0}, turretYaw40ms{0};
    Time timeOfFlight40ms{0};
    Angle turretPitch50ms{0}, turretYaw50ms{0};
    Time timeOfFlight50ms{0};

    bool intersectionFound30ms = findTargetProjectileIntersection(
        targetState,
        30_mps,
        3,
        &turretPitch30ms,
        &turretYaw30ms,
        &timeOfFlight30ms);
    bool intersectionFound40ms = findTargetProjectileIntersection(
        targetState,
        40_mps,
        3,
        &turretPitch40ms,
        &turretYaw40ms,
        &timeOfFlight40ms);
    bool intersectionFound50ms = findTargetProjectileIntersection(
        targetState,
        50_mps,
        3,
        &turretPitch50ms,
        &turretYaw50ms,
        &timeOfFlight50ms);

    EXPECT_EQ(true, intersectionFound30ms);
    EXPECT_EQ(true, intersectionFound40ms);
    EXPECT_EQ(true, intersectionFound50ms);
    EXPECT_GT(turretYaw30ms, turretYaw40ms);
    EXPECT_GT(turretYaw40ms, turretYaw50ms);
    EXPECT_GT(timeOfFlight30ms, timeOfFlight40ms);
    EXPECT_GT(timeOfFlight40ms, timeOfFlight50ms);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_with_target_moving_torwards_turret)
{
    SecondOrderKinematicState targetState{
        Vector3Position(10_m, 0_m, 0_m),
        Vector3Velocity(-1_mps, 0_mps, 0_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2)};

    Angle turretPitch{0}, turretYaw{0};
    Time timeOfFlight{0};

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30_mps,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_GT(0_rad, turretPitch);
    EXPECT_NEAR(0, turretYaw.valueOf(), 1E-5);
    EXPECT_NEAR(10. / 30., timeOfFlight.valueOf(), 0.15);
}

TEST(Ballistics, findTargetProjectileIntersection_target_turret_position_identical)
{
    SecondOrderKinematicState targetState{
        Vector3Position(0_m, 0_m, 0_m),
        Vector3Velocity(0_mps, 0_mps, 0_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2)};

    Angle turretPitch{0}, turretYaw{0};
    Time timeOfFlight{0};

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30_mps,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(false, intersectionFound);
}

TEST(Ballistics, findTargetProjectileIntersection_target_out_of_range)
{
    SecondOrderKinematicState targetState{
        Vector3Position(-20_m, 0_m, 0_m),
        Vector3Velocity(0_mps, 0_mps, 0_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2)};

    Angle turretPitch{0}, turretYaw{0};
    Time timeOfFlight{0};

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        1_mps,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(false, intersectionFound);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_vertical_dist_btwn_turret_and_target_with_target_moving_away_from_turret)
{
    SecondOrderKinematicState targetState{
        Vector3Position(0_m, 0_m, 10_m),
        Vector3Velocity(0_mps, 0_mps, 1_mps),
        Vector3Acceleration(0_mps2, 0_mps2, 0_mps2),
    };

    Angle turretPitch{0}, turretYaw{0};
    Time timeOfFlight{0};

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30_mps,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_NEAR(0, turretYaw.valueOf(), 1E-5);
    EXPECT_GT(0_rad, turretPitch);
    EXPECT_GT(10_s / 30., timeOfFlight);
}
