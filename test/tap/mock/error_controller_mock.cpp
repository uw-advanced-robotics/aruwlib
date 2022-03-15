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

#include "error_controller_mock.hpp"

bool errorDescriptionContainsSubstr(
    const tap::errors::SystemError& error,
    const std::string& substr)
{
    std::string errorStr = error.getDescription();
    return errorStr.find(substr) != std::string::npos;
}

namespace tap::mock
{
ErrorControllerMock::ErrorControllerMock(tap::Drivers *drivers)
    : tap::errors::ErrorController(drivers)
{
}
ErrorControllerMock::~ErrorControllerMock() {}
}  // namespace tap::mock
