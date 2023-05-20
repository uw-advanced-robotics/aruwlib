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

#include "remote_map_state.hpp"

#include <numeric>

#include "tap/algorithms/math_user_utils.hpp"

#include "tap/errors/create_errors.hpp"

using namespace tap::communication::serial;
using namespace tap::algorithms;

namespace tap
{
namespace control
{
RemoteMapState::RemoteMapState(
    Remote::SwitchState leftss,
    Remote::SwitchState rightss,
    float wheel,
    float rightVertical,
    float rightHorizontal,
    float leftVertical,
    float leftHorizontal,
    const std::list<Remote::Key> &keySet,
    const std::list<Remote::Key> &negKeySet,
    bool mouseButtonLeftPressed,
    bool mouseButtonRightPressed)
{
    initLSwitch(leftss);
    initRSwitch(rightss);
    initChannel(Remote::Channel::WHEEL, wheel);
    initChannel(Remote::Channel::RIGHT_VERTICAL, rightVertical);
    initChannel(Remote::Channel::RIGHT_HORIZONTAL, rightHorizontal);
    initChannel(Remote::Channel::LEFT_VERTICAL, leftVertical);
    initChannel(Remote::Channel::LEFT_HORIZONTAL, leftHorizontal);
    initKeys(keySet);
    initNegKeys(negKeySet);
    if (mouseButtonLeftPressed)
    {
        initLMouseButton();
    }
    if (mouseButtonRightPressed)
    {
        initRMouseButton();
    }
}

RemoteMapState::RemoteMapState(Remote::Switch swh, Remote::SwitchState switchState)
{
    if (swh == Remote::Switch::LEFT_SWITCH)
    {
        initLSwitch(switchState);
    }
    else
    {
        initRSwitch(switchState);
    }
}

RemoteMapState::RemoteMapState(Remote::Channel channel, float threshold)
{
    initChannel(channel, threshold);
}

RemoteMapState::RemoteMapState(Remote::SwitchState leftss, Remote::SwitchState rightss)
{
    initLSwitch(leftss);
    initRSwitch(rightss);
}

RemoteMapState::RemoteMapState(
    const std::list<Remote::Key> &keySet,
    const std::list<Remote::Key> &negKeySet)
{
    initKeys(keySet);
    initNegKeys(negKeySet);
}

RemoteMapState::RemoteMapState(
    RemoteMapState::MouseButton button,
    const std::list<Remote::Key> &keySet,
    const std::list<Remote::Key> &negKeySet)
{
    if (button == MouseButton::LEFT)
    {
        initLMouseButton();
    }
    else
    {
        initRMouseButton();
    }
    initKeys(keySet);
    initNegKeys(negKeySet);
}

RemoteMapState::RemoteMapState(MouseButton button)
{
    if (button == MouseButton::LEFT)
    {
        initLMouseButton();
    }
    else
    {
        initRMouseButton();
    }
}

void RemoteMapState::initLSwitch(Remote::SwitchState ss)
{
    if (ss == Remote::SwitchState::UNKNOWN)
    {
        return;
    }
    lSwitch = ss;
}

void RemoteMapState::initRSwitch(Remote::SwitchState ss)
{
    if (ss == Remote::SwitchState::UNKNOWN)
    {
        return;
    }
    rSwitch = ss;
}

void RemoteMapState::initKeys(uint16_t keys)
{
    if (keys == 0)
    {
        return;
    }
    if ((this->negKeys & keys) != 0)
    {
        return;
    }
    this->keys = keys;
}

void RemoteMapState::initNegKeys(uint16_t negKeys)
{
    if (negKeys == 0)
    {
        return;
    }
    if ((this->keys & negKeys) != 0)
    {
        return;
    }
    this->negKeys = negKeys;
}

void RemoteMapState::initKeys(const std::list<Remote::Key> &keySet)
{
    uint16_t keys = std::accumulate(
        keySet.begin(),
        keySet.end(),
        0,
        [](int acc, Remote::Key key) { return acc |= 1 << static_cast<uint16_t>(key); });
    initKeys(keys);
}

void RemoteMapState::initNegKeys(const std::list<Remote::Key> &negKeySet)
{
    // extract a bit form of the key set.
    uint16_t negKeys = std::accumulate(
        negKeySet.begin(),
        negKeySet.end(),
        0,
        [](int acc, Remote::Key key) { return acc |= 1 << static_cast<uint16_t>(key); });
    initNegKeys(negKeys);
}

void RemoteMapState::initChannel(Remote::Channel channel, float threshold){
    if(fabsf(threshold) > 1){
        return;
    }
    switch(channel){
        case Remote::Channel::WHEEL:
            wheel = true;
            wheelThreshold = threshold;
            break;
        case Remote::Channel::RIGHT_VERTICAL:
            rightVertical = true;
            rightVerticalThreshold = threshold;
            break;
        case Remote::Channel::RIGHT_HORIZONTAL:
            rightHorizontal = true;
            rightHorizontalThreshold = threshold;
            break;
        case Remote::Channel::LEFT_VERTICAL:
            leftVertical = true;
            leftVerticalThreshold = threshold;
            break;
        case Remote::Channel::LEFT_HORIZONTAL:
            leftHorizontal = true;
            leftHorizontalThreshold = threshold;
            break;
        default:
            break;
    }

}

void RemoteMapState::initLMouseButton() { lMouseButton = true; }

void RemoteMapState::initRMouseButton() { rMouseButton = true; }

bool RemoteMapState::stateSubsetOf(const RemoteMapState &other) const
{
    if (rSwitch != Remote::SwitchState::UNKNOWN && rSwitch != other.rSwitch)
    {
        return false;
    }
    if (lSwitch != Remote::SwitchState::UNKNOWN && lSwitch != other.lSwitch)
    {
        return false;
    }
    if ((keys & other.keys) != keys)
    {
        return false;
    }
    if (lMouseButton && other.lMouseButton != lMouseButton)
    {
        return false;
    }
    if (rMouseButton && other.rMouseButton != rMouseButton)
    {
        return false;
    }
    if (wheel && (getSign(other.wheelThreshold) != getSign(wheelThreshold) || fabsf(other.wheelThreshold) < fabsf(wheelThreshold)))
    {
        return false;
    }
    if (rightVertical && (getSign(other.rightVerticalThreshold) != getSign(rightVerticalThreshold) || fabsf(other.rightVerticalThreshold) < fabsf(rightVerticalThreshold)))
    {
        return false;
    }
    if (rightHorizontal && (getSign(other.rightHorizontalThreshold) != getSign(rightHorizontalThreshold) || fabsf(other.rightHorizontalThreshold) < fabsf(rightHorizontalThreshold)))
    {
        return false;
    }
    if (leftVertical && (getSign(other.leftVerticalThreshold) != getSign(leftVerticalThreshold) || fabsf(other.leftVerticalThreshold) < fabsf(leftVerticalThreshold)))
    {
        return false;
    }
    if (leftHorizontal && (getSign(other.leftHorizontalThreshold) != getSign(leftHorizontalThreshold) || fabsf(other.leftHorizontalThreshold) < fabsf(leftHorizontalThreshold)))
    {
        return false;
    }
    return true;
}

bool operator==(const RemoteMapState &rms1, const RemoteMapState &rms2)
{
    return rms1.lSwitch == rms2.lSwitch && rms1.rSwitch == rms2.rSwitch && rms1.keys == rms2.keys &&
           rms1.negKeys == rms2.negKeys && rms1.lMouseButton == rms2.lMouseButton &&
           rms1.rMouseButton == rms2.rMouseButton && rms1.wheel == rms2.wheel && rms1.wheelThreshold == rms2.wheelThreshold 
           && rms1.rightVertical == rms2.rightVertical && rms1.rightVerticalThreshold == rms2.rightVerticalThreshold
           && rms1.rightHorizontal == rms2.rightHorizontal && rms1.rightHorizontalThreshold == rms2.rightHorizontalThreshold
           && rms1.leftVertical == rms2.leftVertical && rms1.leftVerticalThreshold == rms2.leftVerticalThreshold
           && rms1.leftHorizontal == rms2.leftHorizontal && rms1.leftHorizontalThreshold == rms2.leftHorizontalThreshold;
}

bool operator!=(const RemoteMapState &rms1, const RemoteMapState &rms2) { return !(rms1 == rms2); }

}  // namespace control
}  // namespace tap
