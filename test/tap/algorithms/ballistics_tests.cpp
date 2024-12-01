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

TEST(Ballistics, quadraticKinematicProjection_dt_zero_pos_unmoving)
{
    EXPECT_EQ(1, AbstractKinematicState::quadraticKinematicProjection(0, 1, 2, 3));
}

TEST(
    Ballistics,
    quadraticKinematicProjection_position_constantly_increasing_when_vel_positive_acc_zero)
{
    EXPECT_NEAR(2, AbstractKinematicState::quadraticKinematicProjection(1, 1, 1, 0), 1E-5);
}

TEST(Ballistics, quadraticKinematicProjection_position_increases_quadratically_when_acc_positive)
{
    EXPECT_NEAR(1, AbstractKinematicState::quadraticKinematicProjection(1, 0, 0, 2), 1e-5);
    EXPECT_NEAR(4, AbstractKinematicState::quadraticKinematicProjection(2, 0, 0, 2), 1e-5);
    EXPECT_NEAR(9, AbstractKinematicState::quadraticKinematicProjection(3, 0, 0, 2), 1e-5);
}

TEST(Ballistics, projectForward_returns_constant_delta_position_when_vel_positive_no_acc)
{
    SecondOrderKinematicState kinematicState(
        modm::Vector3f(1, 1, 1),
        modm::Vector3f(1, 1, 1),
        modm::Vector3f(0, 0, 0));

    auto position1 = kinematicState.projectForward(1);
    auto position2 = kinematicState.projectForward(2);
    auto position3 = kinematicState.projectForward(3);

    auto diff12 = position2 - position1;
    auto diff23 = position3 - position2;

    EXPECT_NEAR(diff12.x, diff23.x, 1E-5);
    EXPECT_NEAR(diff12.y, diff23.y, 1E-5);
    EXPECT_NEAR(diff12.z, diff23.z, 1E-5);
}

TEST(Ballistics, computeTravelTime_horizontal_distance_only_bulletVelocity_10)
{
    modm::Vector3f targetPosition(10, 0, 0);

    float travelTime = 0;
    float turretPitch = 0;
    bool timeFound = computeTravelTime(targetPosition, 20, &travelTime, &turretPitch);

    // 20 m/s velocity, 10 m target distance, time should be slightly greater than .5
    EXPECT_EQ(true, timeFound);
    EXPECT_LT(0.5, travelTime);
    EXPECT_GT(0.75, travelTime);
}

TEST(Ballistics, computeTravelTime_vertical_distance_only_hits_target)
{
    modm::Vector3f targetPosition(0, 0, 10);
    float travelTime = 0;
    float turretPitch = 0;
    bool timeFound = computeTravelTime(targetPosition, 20, &travelTime, &turretPitch);

    EXPECT_EQ(true, timeFound);
}

TEST(Ballistics, computeTravelTime_vertical_distance_only_no_hit)
{
    modm::Vector3f targetPosition(0, 0, 10);
    float travelTime = 0;
    float turretPitch = 0;
    bool timeFound = computeTravelTime(targetPosition, 10, &travelTime, &turretPitch);

    EXPECT_EQ(false, timeFound);
}

TEST(Ballistics, computeTravelTime_bulletVelocity_0)
{
    modm::Vector3f targetPosition(3, 3, 2);

    float travelTime = 0;
    float turretPitch = 0;
    bool timeFound = computeTravelTime(targetPosition, 0, &travelTime, &turretPitch);

    EXPECT_EQ(false, timeFound);
}

TEST(Ballistics, computeTravelTime_bulletVelocity_large)
{
    modm::Vector3f targetPosition(10, 10, 0);

    float travelTime = 0;
    float turretPitch = 0;
    bool timeFound = computeTravelTime(targetPosition, 1E9, &travelTime, &turretPitch);

    EXPECT_EQ(true, timeFound);
    EXPECT_NEAR(0, travelTime, 1E-5);
    EXPECT_NEAR(0, turretPitch, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_stationary)
{
    SecondOrderKinematicState targetState{
        modm::Vector3f(10, 0, 0),
        modm::Vector3f(0, 0, 0),
        modm::Vector3f(0, 0, 0)};

    float turretPitch = 0;
    float turretYaw = 0;
    float timeOfFlight = 0;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        1E6,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_NEAR(0, turretPitch, 1E-5);
    EXPECT_NEAR(0, turretYaw, 1E-5);
    EXPECT_NEAR(0, timeOfFlight, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_moving_away_from_turret)
{
    SecondOrderKinematicState targetState{
        modm::Vector3f(10, 0, 0),
        modm::Vector3f(1, 0, 0),
        modm::Vector3f(0, 0, 0)};

    float turretPitch = 0;
    float turretYaw = 0;
    float timeOfFlight = 0;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_LT(-modm::toRadian(45), turretPitch);
    EXPECT_NEAR(0, turretYaw, 1E-5);
    EXPECT_NEAR(10. / 30., timeOfFlight, 0.15);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_moving_perpendicular_to_turret)
{
    SecondOrderKinematicState targetState(
        modm::Vector3f(10, 0, 0),
        modm::Vector3f(0, 1, 0),
        modm::Vector3f(0, 0, 0));

    float turretPitch = 0;
    float turretYaw = 0;
    float timeOfFlight = 0;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_LT(0, turretYaw);
    EXPECT_GT(modm::toRadian(30), turretYaw);
    EXPECT_LT(-modm::toRadian(45), turretPitch);
    EXPECT_NEAR(10. / 30., timeOfFlight, 0.15);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_velocity_increases_lead_position_decreases)
{
    SecondOrderKinematicState targetState(
        modm::Vector3f(10, 0, 0),
        modm::Vector3f(0, 1, 0),
        modm::Vector3f(0, 0, 0));

    float turretPitch30ms, turretYaw30ms, timeOfFlight30ms;
    float turretPitch40ms, turretYaw40ms, timeOfFlight40ms;
    float turretPitch50ms, turretYaw50ms, timeOfFlight50ms;

    bool intersectionFound30ms = findTargetProjectileIntersection(
        targetState,
        30,
        3,
        &turretPitch30ms,
        &turretYaw30ms,
        &timeOfFlight30ms);
    bool intersectionFound40ms = findTargetProjectileIntersection(
        targetState,
        40,
        3,
        &turretPitch40ms,
        &turretYaw40ms,
        &timeOfFlight40ms);
    bool intersectionFound50ms = findTargetProjectileIntersection(
        targetState,
        50,
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
        modm::Vector3f(10, 0, 0),
        modm::Vector3f(-1, 0, 0),
        modm::Vector3f(0, 0, 0)};

    float turretPitch, turretYaw, timeOfFlight;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_GT(0, turretPitch);
    EXPECT_NEAR(0, turretYaw, 1E-5);
    EXPECT_NEAR(10. / 30., timeOfFlight, 0.15);
}

TEST(Ballistics, findTargetProjectileIntersection_target_turret_position_identical)
{
    SecondOrderKinematicState targetState{
        modm::Vector3f(0, 0, 0),
        modm::Vector3f(0, 0, 0),
        modm::Vector3f(0, 0, 0)};

    float turretPitch, turretYaw, timeOfFlight;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(false, intersectionFound);
}

TEST(Ballistics, findTargetProjectileIntersection_target_out_of_range)
{
    SecondOrderKinematicState targetState{
        modm::Vector3f(-20, 0, 0),
        modm::Vector3f(0, 0, 0),
        modm::Vector3f(0, 0, 0)};

    float turretPitch, turretYaw, timeOfFlight;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        1,
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
        modm::Vector3f(0, 0, 10),
        modm::Vector3f(0, 0, 1),
        modm::Vector3f(0, 0, 0),
    };

    float turretPitch, turretYaw, timeOfFlight;

    bool intersectionFound = findTargetProjectileIntersection(
        targetState,
        30,
        3,
        &turretPitch,
        &turretYaw,
        &timeOfFlight);

    EXPECT_EQ(true, intersectionFound);
    EXPECT_NEAR(0, turretYaw, 1E-5);
    EXPECT_GT(0, turretPitch);
    EXPECT_GT(10. / 30., timeOfFlight);
}
