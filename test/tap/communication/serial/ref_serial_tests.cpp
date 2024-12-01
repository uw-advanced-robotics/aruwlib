/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/robot_to_robot_message_handler_mock.hpp"

using namespace tap;
using namespace tap::communication::serial;
using namespace tap::arch;

template <typename T>
static DJISerial::ReceivedSerialMessage constructMsg(const T &data, int type)
{
    DJISerial::ReceivedSerialMessage msg;
    msg.header.seq = 0;
    msg.messageType = type;
    msg.header.dataLength = sizeof(T);
    memset(msg.data, 0, sizeof(msg.data));
    memcpy(msg.data, reinterpret_cast<const uint8_t *>(&data), sizeof(T));
    return msg;
}

TEST(RefSerial, messageReceiveCallback__competition_status)
{
    struct GameStatus
    {
        uint8_t gameType : 4;
        uint8_t gameProgress : 4;
        uint16_t stageRemainTime;
        uint64_t syncTimeStamp;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    GameStatus testData;

    testData.gameType = 3;
    testData.gameProgress = 2;
    testData.stageRemainTime = 200;
    testData.syncTimeStamp = 0x12345678;
    msg = constructMsg(testData, 1);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(200, refSerial.getGameData().stageTimeRemaining);
    EXPECT_EQ(RefSerial::Rx::GameType::ROBOMASTER_AI_CHALLENGE, refSerial.getGameData().gameType);
    EXPECT_EQ(RefSerial::Rx::GameStage::INITIALIZATION, refSerial.getGameData().gameStage);
    EXPECT_EQ(0x12345678, refSerial.getGameData().unixTime);

    testData.gameProgress = 4;
    msg = constructMsg(testData, 1);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameStage::IN_GAME, refSerial.getGameData().gameStage);
}

TEST(RefSerial, messageReceiveCallback__competition_result)
{
    struct GameResult
    {
        uint8_t winner;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    GameResult testData;

    testData.winner = 0;
    msg = constructMsg(testData, 2);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::DRAW, refSerial.getGameData().gameWinner);

    testData.winner = 1;
    msg = constructMsg(testData, 2);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::RED, refSerial.getGameData().gameWinner);

    testData.winner = 2;
    msg = constructMsg(testData, 2);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::GameWinner::BLUE, refSerial.getGameData().gameWinner);
}

TEST(RefSerial, messageReceiveCallback__robot_hp)
{
    struct GameRobotHP
    {
        uint16_t red1RobotHP;
        uint16_t red2RobotHP;
        uint16_t red3RobotHP;
        uint16_t red4RobotHP;
        uint16_t red5RobotHP;
        uint16_t red7RobotHP;
        uint16_t redOutpostHP;
        uint16_t redBaseHP;
        uint16_t blue1RobotHP;
        uint16_t blue2RobotHP;
        uint16_t blue3RobotHP;
        uint16_t blue4RobotHP;
        uint16_t blue5RobotHP;
        uint16_t blue7RobotHP;
        uint16_t blueOutpostHP;
        uint16_t blueBaseHP;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;

    GameRobotHP testData;

    testData.red1RobotHP = 1;
    testData.red2RobotHP = 2;
    testData.red3RobotHP = 3;
    testData.red4RobotHP = 4;
    testData.red5RobotHP = 5;
    testData.red7RobotHP = 6;
    testData.redOutpostHP = 7;
    testData.redBaseHP = 8;
    testData.blue1RobotHP = 9;
    testData.blue2RobotHP = 10;
    testData.blue3RobotHP = 11;
    testData.blue4RobotHP = 12;
    testData.blue5RobotHP = 13;
    testData.blue7RobotHP = 14;
    testData.blueOutpostHP = 15;
    testData.blueBaseHP = 16;
    msg = constructMsg(testData, 3);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(1, refSerial.getRobotData().allRobotHp.red.hero1);
    EXPECT_EQ(2, refSerial.getRobotData().allRobotHp.red.engineer2);
    EXPECT_EQ(3, refSerial.getRobotData().allRobotHp.red.standard3);
    EXPECT_EQ(4, refSerial.getRobotData().allRobotHp.red.standard4);
    EXPECT_EQ(5, refSerial.getRobotData().allRobotHp.red.standard5);
    EXPECT_EQ(6, refSerial.getRobotData().allRobotHp.red.sentry7);
    EXPECT_EQ(7, refSerial.getRobotData().allRobotHp.red.outpost);
    EXPECT_EQ(8, refSerial.getRobotData().allRobotHp.red.base);
    EXPECT_EQ(9, refSerial.getRobotData().allRobotHp.blue.hero1);
    EXPECT_EQ(10, refSerial.getRobotData().allRobotHp.blue.engineer2);
    EXPECT_EQ(11, refSerial.getRobotData().allRobotHp.blue.standard3);
    EXPECT_EQ(12, refSerial.getRobotData().allRobotHp.blue.standard4);
    EXPECT_EQ(13, refSerial.getRobotData().allRobotHp.blue.standard5);
    EXPECT_EQ(14, refSerial.getRobotData().allRobotHp.blue.sentry7);
    EXPECT_EQ(15, refSerial.getRobotData().allRobotHp.blue.outpost);
    EXPECT_EQ(16, refSerial.getRobotData().allRobotHp.blue.base);
}

struct RefWarning
{
    uint8_t level;
    uint8_t foulRobotId;
    uint8_t count;
} modm_packed;

TEST(RefSerial, messageReceiveCallback__decodeToWarningData)
{
    tap::arch::clock::ClockStub clock;
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    RefWarning testData;

    clock.time += 1'000;

    testData.level = 2;
    testData.foulRobotId = static_cast<uint8_t>(RefSerialData::RobotId::BLUE_SOLDIER_1);
    testData.count = 3;

    msg = constructMsg(testData, 0x0104);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(2, refSerial.getRobotData().refereeWarningData.level);
    EXPECT_EQ(
        RefSerialData::RobotId::BLUE_SOLDIER_1,
        refSerial.getRobotData().refereeWarningData.foulRobotID);
    EXPECT_EQ(3, refSerial.getRobotData().refereeWarningData.count);
    EXPECT_EQ(clock.time, refSerial.getRobotData().refereeWarningData.lastReceivedWarningRobotTime);
}

struct GameRobotStatus
{
    uint8_t robotId;
    uint8_t robot_level;
    uint16_t remainHP;
    uint16_t maxHP;
    uint16_t coolingRate;
    uint16_t heatLimit;
    uint16_t chassisPowerLimit;
    uint8_t mainsPowerGimbalOutput : 1;
    uint8_t mainsPowerChassisOutput : 1;
    uint8_t mainsPowerShooterOutput : 1;
} modm_packed;

TEST(RefSerial, messageReceiveCallback__operatorBlinded_true_operator_offending)
{
    tap::arch::clock::ClockStub clock;
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    RefWarning testData;
    GameRobotStatus robotData;

    EXPECT_FALSE(refSerial.operatorBlinded());

    clock.time += 10'000;

    EXPECT_FALSE(refSerial.operatorBlinded());

    testData.level = 2;
    testData.foulRobotId = static_cast<uint8_t>(RefSerialData::RobotId::BLUE_SOLDIER_1);
    testData.count = 1;

    msg = constructMsg(testData, 0x0104);
    refSerial.messageReceiveCallback(msg);

    robotData.robotId = static_cast<uint8_t>(RefSerialData::RobotId::BLUE_SOLDIER_1);
    robotData.remainHP = 0;
    msg = constructMsg(robotData, 0x0201);
    refSerial.messageReceiveCallback(msg);

    EXPECT_TRUE(refSerial.operatorBlinded());

    clock.time += RefSerialData::Rx::RefereeWarningData::OFFENDING_OPERATOR_BLIND_TIME - 1;
    refSerial.messageReceiveCallback(msg);  // call again to ensure ref serial is "online"

    EXPECT_TRUE(refSerial.operatorBlinded());

    clock.time += 2;

    EXPECT_FALSE(refSerial.operatorBlinded());
}

TEST(RefSerial, messageReceiveCallback__operatorBlinded_true_operator_not_offending)
{
    tap::arch::clock::ClockStub clock;
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    RefWarning testData;
    GameRobotStatus robotData;

    clock.time += 10'000;

    testData.level = 2;
    testData.foulRobotId = static_cast<uint8_t>(RefSerialData::RobotId::BLUE_SOLDIER_2);
    testData.count = 1;

    msg = constructMsg(testData, 0x0104);
    refSerial.messageReceiveCallback(msg);

    robotData.robotId = static_cast<uint8_t>(RefSerialData::RobotId::BLUE_SOLDIER_1);
    robotData.remainHP = 0;
    msg = constructMsg(robotData, 0x0201);
    refSerial.messageReceiveCallback(msg);

    EXPECT_TRUE(refSerial.operatorBlinded());

    clock.time += RefSerialData::Rx::RefereeWarningData::NONOFFENDING_OPERATOR_BLIND_TIME - 1;
    refSerial.messageReceiveCallback(msg);  // call again to ensure ref serial is "online"

    EXPECT_TRUE(refSerial.operatorBlinded());

    clock.time += 2;

    EXPECT_FALSE(refSerial.operatorBlinded());
}

TEST(RefSerial, messageReceiveCallback__robot_status)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    GameRobotStatus testData;

    testData.robot_level = 3;
    testData.remainHP = 1001;
    testData.maxHP = 2001;
    testData.coolingRate = 120;
    testData.heatLimit = 100;
    testData.chassisPowerLimit = 987;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(3, refSerial.getRobotData().robotLevel);
    EXPECT_EQ(1001, refSerial.getRobotData().currentHp);
    EXPECT_EQ(2001, refSerial.getRobotData().maxHp);
    EXPECT_EQ(120, refSerial.getRobotData().turret.coolingRate);
    EXPECT_EQ(100, refSerial.getRobotData().turret.heatLimit);
    EXPECT_EQ(987, refSerial.getRobotData().chassis.powerConsumptionLimit);

    // red team
    for (int i = static_cast<int>(RefSerial::RobotId::RED_HERO);
         i <= static_cast<int>(RefSerial::RobotId::RED_RADAR_STATION);
         i++)
    {
        testData.robotId = i;
        msg = constructMsg(testData, 0x0201);

        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(i, static_cast<int>(refSerial.getRobotData().robotId));
    }

    // blue team
    for (int i = static_cast<int>(RefSerial::RobotId::BLUE_HERO);
         i <= static_cast<int>(RefSerial::RobotId::BLUE_RADAR_STATION);
         i++)
    {
        testData.robotId = i;
        msg = constructMsg(testData, 0x0201);

        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(i, static_cast<int>(refSerial.getRobotData().robotId));
    }

    // Main controller power supply condition

    testData.mainsPowerGimbalOutput = 1;
    testData.mainsPowerChassisOutput = 0;
    testData.mainsPowerShooterOutput = 0;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER, refSerial.getRobotData().robotPower);

    testData.mainsPowerGimbalOutput = 0;
    testData.mainsPowerChassisOutput = 1;
    testData.mainsPowerShooterOutput = 0;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER, refSerial.getRobotData().robotPower);

    testData.mainsPowerGimbalOutput = 0;
    testData.mainsPowerChassisOutput = 0;
    testData.mainsPowerShooterOutput = 1;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::RobotPower::SHOOTER_HAS_POWER, refSerial.getRobotData().robotPower);

    testData.mainsPowerGimbalOutput = 1;
    testData.mainsPowerChassisOutput = 1;
    testData.mainsPowerShooterOutput = 1;
    msg = constructMsg(testData, 0x0201);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(
        RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER | RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER |
            RefSerial::Rx::RobotPower::SHOOTER_HAS_POWER,
        refSerial.getRobotData().robotPower);
}

TEST(RefSerial, messageReceiveCallback__robot_status_dps_measured)
{
    tap::arch::clock::ClockStub clock;
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    GameRobotStatus testData;

    // vector of arrays of length 3 containing a time, current HP, and expected DPS
    std::vector<std::array<int, 3>> timeHPExpectedDPS{
        {0, 100, 0},
        {10, 90, 10},
        {20, 90, 10},
        {30, 90, 10},
        {40, 80, 20},
        {1500, 80, 0}};

    for (const auto &triplet : timeHPExpectedDPS)
    {
        clock.time = triplet[0];

        testData.remainHP = triplet[1];
        msg = constructMsg(testData, 0x0201);
        refSerial.messageReceiveCallback(msg);

        EXPECT_EQ(triplet[1], refSerial.getRobotData().currentHp);
        EXPECT_EQ(triplet[2], refSerial.getRobotData().receivedDps);
    }
}

TEST(RefSerial, messageReceiveCallback__power_and_heat)
{
    struct PowerHeatData
    {
        uint16_t chassisVolt;
        uint16_t chassisCurrent;
        float chassisPower;
        uint16_t chassis_power_buffer;
        uint16_t shooterId117mmCoolingHeat;
        uint16_t shooterId217mmCoolingHeat;
        uint16_t shooterId142mmCoolingHeat;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    PowerHeatData testData;

    testData.chassisVolt = 1234;
    testData.chassisCurrent = 4321;
    testData.chassisPower = 6789;
    testData.chassis_power_buffer = 120;
    testData.shooterId117mmCoolingHeat = 145;
    testData.shooterId217mmCoolingHeat = 431;
    testData.shooterId142mmCoolingHeat = 900;
    msg = constructMsg(testData, 0x0202);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(1234, refSerial.getRobotData().chassis.volt);
    EXPECT_EQ(4321, refSerial.getRobotData().chassis.current);
    EXPECT_EQ(6789, refSerial.getRobotData().chassis.power);
    EXPECT_EQ(120, refSerial.getRobotData().chassis.powerBuffer);
    EXPECT_EQ(145, refSerial.getRobotData().turret.heat17ID1);
    EXPECT_EQ(431, refSerial.getRobotData().turret.heat17ID2);
    EXPECT_EQ(900, refSerial.getRobotData().turret.heat42);
}

TEST(RefSerial, messageReceiveCallback__robot_position)
{
    struct RobotPosData
    {
        float x;
        float y;
        float yaw;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    RobotPosData testData;

    testData.x = -89.31f;
    testData.y = 45069.24f;
    testData.yaw = -799.87f;
    msg = constructMsg(testData, 0x0203);

    refSerial.messageReceiveCallback(msg);

    EXPECT_NEAR(-89.31f, refSerial.getRobotData().chassis.position.x, 1E-3f);
    EXPECT_NEAR(45069.24f, refSerial.getRobotData().chassis.position.y, 1E-3f);
    EXPECT_NEAR(-799.87f, refSerial.getRobotData().turret.yaw, 1E-3f);
}

TEST(RefSerial, messageReceiveCallback__robot_buffs)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;

    RefSerial::Rx::RobotBuffStatus buff;
    buff.attackBuff = 10;
    buff.coolingBuff = 20;
    buff.defenseBuff = 30;
    buff.recoveryBuff = 40;
    buff.vulnerabilityBuff = 50;

    msg = constructMsg(buff, 0x0204);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(10, refSerial.getRobotData().robotBuffStatus.attackBuff);

    EXPECT_EQ(20, refSerial.getRobotData().robotBuffStatus.coolingBuff);

    EXPECT_EQ(30, refSerial.getRobotData().robotBuffStatus.defenseBuff);

    EXPECT_EQ(40, refSerial.getRobotData().robotBuffStatus.recoveryBuff);

    EXPECT_EQ(50, refSerial.getRobotData().robotBuffStatus.vulnerabilityBuff);
}

TEST(RefSerial, messageReceiveCallback__air_support_data)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    RefSerial::Rx::AirSupportData air;
    air.remainingStateTime = 255;
    air.state = RefSerial::Rx::AirSupportState::IN_AIR;

    DJISerial::ReceivedSerialMessage msg;
    msg = constructMsg(air, 0x0205);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(255, refSerial.getGameData().airSupportData.remainingStateTime);
    EXPECT_EQ(RefSerial::Rx::AirSupportState::IN_AIR, refSerial.getGameData().airSupportData.state);
}

TEST(RefSerial, messageReceiveCallback__damage_status)
{
    struct RobotDamage
    {
        uint8_t armorId : 4;
        uint8_t hurtType : 4;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    RobotDamage testData;

    testData.armorId = 0;
    testData.hurtType = 0;
    msg = constructMsg(testData, 0x0206);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::FRONT, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::ARMOR_DAMAGE, refSerial.getRobotData().damageType);

    testData.armorId = 2;
    testData.hurtType = 1;
    msg = constructMsg(testData, 0x0206);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::REAR, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::MODULE_OFFLINE, refSerial.getRobotData().damageType);

    testData.armorId = 4;
    testData.hurtType = 5;
    msg = constructMsg(testData, 0x0206);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::ArmorId::TOP, refSerial.getRobotData().damagedArmorId);
    EXPECT_EQ(RefSerial::Rx::DamageType::COLLISION, refSerial.getRobotData().damageType);
}

TEST(RefSerial, messageReceiveCallback__launching_information)
{
    struct ShootData
    {
        uint8_t bulletType;
        uint8_t shooterId;
        uint8_t bulletFreq;
        float bulletSpeed;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    ShootData testData;

    testData.bulletType = 1;
    testData.shooterId = 1;
    testData.bulletFreq = 45;
    testData.bulletSpeed = 3452.12f;
    msg = constructMsg(testData, 0x0207);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::BulletType::AMMO_17, refSerial.getRobotData().turret.bulletType);
    EXPECT_EQ(
        RefSerial::Rx::MechanismID::TURRET_17MM_1,
        refSerial.getRobotData().turret.launchMechanismID);
    EXPECT_EQ(45, refSerial.getRobotData().turret.firingFreq);
    EXPECT_NEAR(3452.12f, refSerial.getRobotData().turret.bulletSpeed, 1E-3);

    testData.bulletType = 2;
    testData.shooterId = 2;
    msg = constructMsg(testData, 0x0207);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(RefSerial::Rx::BulletType::AMMO_42, refSerial.getRobotData().turret.bulletType);
    EXPECT_EQ(
        RefSerial::Rx::MechanismID::TURRET_17MM_2,
        refSerial.getRobotData().turret.launchMechanismID);
}

TEST(RefSerial, messageReceiveCallback__remaining_projectiles)
{
    struct BulletRemaining
    {
        uint16_t bulletRemainingNum17mm;
        uint16_t bulletRemainingNum42mm;
        uint16_t coinRemainingNum;
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    BulletRemaining testData;

    testData.bulletRemainingNum17mm = 123;
    testData.bulletRemainingNum42mm = 1890;
    testData.coinRemainingNum = 12892;
    msg = constructMsg(testData, 0x0208);

    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(123, refSerial.getRobotData().turret.bulletsRemaining17);
    EXPECT_EQ(1890, refSerial.getRobotData().turret.bulletsRemaining42);
    EXPECT_EQ(12892, refSerial.getRobotData().remainingCoins);
}

TEST(RefSerial, messageReceiveCallback__RFID_status)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;

    msg = constructMsg(static_cast<uint32_t>(0b00000101), 0x0209);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(msg.data[0], refSerial.getRobotData().rfidStatus.value);

    msg = constructMsg(static_cast<uint32_t>(0b11111010), 0x0209);
    refSerial.messageReceiveCallback(msg);

    EXPECT_EQ(msg.data[0], refSerial.getRobotData().rfidStatus.value);
}

static void updateRobotId(RefSerial &refSerial, RefSerial::RobotId id)
{
    DJISerial::ReceivedSerialMessage msg;
    GameRobotStatus testData;
    testData.remainHP = 300;
    testData.robotId = static_cast<uint16_t>(id);
    msg = constructMsg(testData, 0x0201);
    refSerial.messageReceiveCallback(msg);
}

TEST(
    RefSerial,
    getRobotIdBasedOnCurrentRobotTeam__returns_proper_robot_id_based_on_current_robot_id)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);

    EXPECT_EQ(
        RefSerial::RobotId::INVALID,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::INVALID));

    updateRobotId(refSerial, RefSerial::RobotId::BLUE_SENTINEL);

    EXPECT_EQ(
        RefSerial::RobotId::INVALID,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::INVALID));
    EXPECT_EQ(
        RefSerial::RobotId::BLUE_SOLDIER_2,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::BLUE_SOLDIER_2));
    EXPECT_EQ(
        RefSerial::RobotId::BLUE_SOLDIER_2,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::RED_SOLDIER_2));

    updateRobotId(refSerial, RefSerial::RobotId::RED_RADAR_STATION);

    EXPECT_EQ(
        RefSerial::RobotId::RED_HERO,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::BLUE_HERO));
    EXPECT_EQ(
        RefSerial::RobotId::RED_HERO,
        refSerial.getRobotIdBasedOnCurrentRobotTeam(RefSerial::RobotId::RED_HERO));
}

TEST(RefSerial, attachRobotToRobotMessageHandler__fails_to_add_if_msgId_out_of_bounsd)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    tap::mock::RobotToRobotMessageHandlerMock handler;

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    refSerial.attachRobotToRobotMessageHandler(0x014, &handler);
    refSerial.attachRobotToRobotMessageHandler(0x3ff, &handler);
}

TEST(RefSerial, attachRobotToRobotMessageHandler__fails_to_add_if_msgId_already_added)
{
    Drivers drivers;
    RefSerial refSerial(&drivers);
    tap::mock::RobotToRobotMessageHandlerMock handler;

    refSerial.attachRobotToRobotMessageHandler(0x201, &handler);

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(1);

    refSerial.attachRobotToRobotMessageHandler(0x201, &handler);
}

TEST(RefSerial, messageReceiveCallback__robot_to_robot_data_simple_message)
{
    struct SpecialData
    {
        RefSerial::Tx::InteractiveHeader interactiveHeader;
        uint8_t hi[2];
    } modm_packed;

    Drivers drivers;
    RefSerial refSerial(&drivers);
    DJISerial::ReceivedSerialMessage msg;
    SpecialData specialData;

    tap::mock::RobotToRobotMessageHandlerMock handler;

    specialData.interactiveHeader.dataCmdId = 0x201;
    specialData.hi[0] = 'h';
    specialData.hi[1] = 'i';
    msg = constructMsg(specialData, 0x301);

    refSerial.attachRobotToRobotMessageHandler(0x201, &handler);

    EXPECT_CALL(handler, functorOp).WillOnce([&](const DJISerial::ReceivedSerialMessage &message) {
        EXPECT_EQ('h', message.data[sizeof(RefSerial::Tx::InteractiveHeader)]);
        EXPECT_EQ('i', message.data[sizeof(RefSerial::Tx::InteractiveHeader) + 1]);
        EXPECT_EQ(sizeof(SpecialData), message.header.dataLength);
    });

    refSerial.messageReceiveCallback(msg);
}
