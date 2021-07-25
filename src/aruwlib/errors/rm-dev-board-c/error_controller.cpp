/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "error_controller.hpp"

#include "aruwlib/algorithms/strtok.hpp"
#include "aruwlib/communication/gpio/leds.hpp"
#include "aruwlib/drivers.hpp"

#include "modm/container/linked_list.hpp"

namespace aruwlib
{
namespace errors
{
void ErrorController::addToErrorList(const SystemError& error)
{
    // only add error if it is not already added
    // Note that we are okay with comparing raw char pointers because an error generated
    // in our codebase use char pointers located in literals.
    for (SystemError sysErr : errorList)
    {
        if (sysErr.getDescription() == error.getDescription() &&
            sysErr.getFilename() == error.getFilename() &&
            sysErr.getLineNumber() == error.getLineNumber())
        {
            return;  // the error is already added
        }
    }
    if (errorList.getSize() >= errorList.getMaxSize())
    {
        errorList.removeFront();  // remove the oldest element in the error list
    }
    errorList.append(error);
}

void ErrorController::init()
{
    drivers->terminalSerial.addHeader("error", &drivers->errorController);
}

bool ErrorController::removeSystemErrorAtIndex(error_index_t index)
{
    if (index >= errorList.getSize())
    {
        return false;
    }
    if (index == 0)
    {
        errorList.removeFront();
        return true;
    }
    else if (index == errorList.getSize() - 1)
    {
        errorList.removeBack();
        return true;
    }
    error_index_t size = errorList.getSize();
    for (error_index_t i = 0; i < size; i++)
    {
        SystemError se = errorList.get(0);
        errorList.removeFront();
        if (i != index)
        {
            errorList.append(se);
        }
    }
    return true;
}

void ErrorController::removeAllSystemErrors()
{
    while (errorList.getSize() > 0)
    {
        errorList.removeBack();
    }
}

bool ErrorController::terminalSerialCallback(
    char* inputLine,
    modm::IOStream& outputStream,
    bool streamingEnabled)
{
    if (streamingEnabled)
    {
        outputStream << "Error Controller: streaming is not supported" << modm::endl;
        return false;
    }
    char* arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine);
    if (arg == nullptr || strcmp(arg, "-H") == 0)
    {
        help(outputStream);
        return arg != nullptr;
    }
    else if (strcmp(arg, "PrintErrors") == 0)
    {
        outputStream << "printing errors" << modm::endl;
        displayAllErrors(outputStream);
    }
    else if (strcmp(arg, "RemoveAllTerminalErrors") == 0)
    {
        clearAllTerminalErrors(outputStream);
    }
    else if (strcmp(arg, "RemoveTerminalError") == 0)
    {
        arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine);
        if (arg == nullptr)
        {
            outputStream << "Error Controller: must specify an index" << modm::endl;
            return false;
        }
        char* indexEnd;
        int index = strtol(arg, &indexEnd, 10);
        if (indexEnd != arg + strlen(arg))
        {
            outputStream << "Error Controller: invalid index: " << arg << modm::endl;
            return false;
        }
        removeTerminalError(index, outputStream);
    }
    else
    {
        outputStream << "Command not found, try again, type \"error -H\" for more." << modm::endl;
        return false;
    }
    return true;
}

void ErrorController::help(modm::IOStream& outputStream)
{
    outputStream << "Usage: error <target> [index]\n"
                    "  Where\"<target>\" is one of:\n"
                    "    - \"-H\": displays possible commands.\n"
                    "    - \"PrintErrors\": prints all errors in errorList, displaying their"
                    "description, lineNumber, fileName, and index.\n"
                    "    - \"RemoveTerminalError [Index]\": removes the error at the given "
                    "index. Example: Error RemoveTerminalError 1.\n"
                    "    - \"RemoveAllTerminalErrors\": removes all errors from the errorList.\n";
}

void ErrorController::displayAllErrors(modm::IOStream& outputStream)
{
    int index = 0;
    if (errorList.getSize() == 0)
    {
        outputStream << "No Errors Found" << modm::endl;
    }
    else
    {
        for (SystemError sysErr : errorList)
        {
            outputStream << index++ << ") " << sysErr.getDescription() << " ["
                         << sysErr.getFilename() << ':' << sysErr.getLineNumber() << ']'
                         << modm::endl;
        }
    }
}

// Syntax: Error RemoveTerminalError [Index]
void ErrorController::removeTerminalError(int index, modm::IOStream& outputStream)
{
    outputStream << "Removing Terminal Error at index..." << index << modm::endl;
    if (!removeSystemErrorAtIndex(index))
    {
        outputStream << "Invalid index" << modm::endl;
    }
}

void ErrorController::clearAllTerminalErrors(modm::IOStream& outputStream)
{
    outputStream << "Clearing all terminal errors..." << modm::endl;
    removeAllSystemErrors();
}

}  // namespace errors

}  // namespace aruwlib
