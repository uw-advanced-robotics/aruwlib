# Taproot Changelog

## January 2025

### Breaking Changes
- Updated modm, scons, GCC, and related tools and dependencies to modern versions. **Note that this update may require significant changes to your project and setup.** See the [extended changelog](./extended-changelogs/modm-update-jan2025-changes.md) for more details and upgrade instructions.

## October 2024

- Added `SequentialCommand`
    - Allows running multiple commands in order.
- Added `Concurrent[Race]Command`
    - Allows running multiple commands at the same time.
    - The race variant ends when any command has finished, while the normal variant waits for all to finish.
- Refactored the hardware testing feature.
    - Hardware tests are now done through commands set with `Subsystem::setTestCommand`
    - Hardware tests now only run when the robot is out of safe disconnect mode.
    - All tests are runnable with `CommandScheduler::runAllHardwareTests`, while individual tests can be run with `CommandScheduler::runHardwareTest(const Subsystem*)`
    - Similarly, `CommandScheduler::stopAllHardwareTests` and `CommandScheduler::stopHardwareTest(const Subsystem*)` exist.
    - `CommandScheduler::isRunningTest(const Subsystem*)` returns true if the subsystem is running a hardware test.
    - `CommandScheduler::hasPassedTest(const Subsystem*)` returns true if the subsystem has passed a hardware test.
    - `CommandScheduler::countRunningHardwareTests()` returns the number of hardware tests currently running.
    - Hardware Test Menu:
        - The tests now must be manually run and don't start when the menu is open.
        - There is now a line showing how many tests are running.
        - Each test is individually runnable and stoppable.
        - `x` is shown for failed/incomplete tests, `+` is shown for passed tests.
- Updated the `mpu6500` and `bmi088` to allow for variable temperature setpoints. This was done as in testing,
  the type C board was found to operate at a lower tempreature than the type A.
- Updated `MahonyAHRS` for IMUs to no longer include the 180 degree offset. IMUs will now intitialize at 0 degrees
- Updated `GovernorWithFallbackCommand` such that if the governed command is selected, the command stops if any governor is finished. Also stops the fallback command if all governors become ready.

### Breaking Changes
- Removed `Subsystem::isHardwareTestComplete`, `Subsystem::setHardwareTestsIncomplete`, `Subsystem::setHardwareTestsComplete`, `Subsystem::runHardwareTests`, `Subsystem::onHardwareTestStart`, `Subsystem::onHardwareTestComplete`
- Removed `CommandScheduler::startHardwareTests`, `CommandScheduler::stopHardwareTests`

## September 2024

- Added some more utility functions to `WrappedFloat`
  - `withinRange`, `rangeOverlap` can be used to deal with wrapped ranges
  - `withSameBounds` and `Angle::fromDegrees` can be used to construct `WrappedFloat`s
  - Fixed a bug with how `revolutions` was calculated

### Breaking Changes
- Bmi088 now has seperate `periodicIMUUpdate` and `read` methods. `periodicIMUUpdate` should 
be called at a fixed rate of mahony, and `read` should be called at a rate such that `periodicIMUUpdate` <= `read` <= sampling rate.
- The `Angle` class within `WrappedFloat` now has bounds of 0 to 2pi as opposed to -pi to pi. This affects values gotten from `getWrappedValue()`.

## July 2024

- Added `taproot:modm-project.xml:modm_hal_modules` option to include additional user-defined modm modules.

## June 2024

- Reduced max Ref Serial Transmission from `1280` bytes to `1000` bytes per second.
- Improved calculation for Ref Serial Transmitter timer lengths.
- Fixed bug where `VerticalScrollLogicHandler::getLargestIndexDisplayed()` returns index out of bounds when size is less than max entries
- Substituted uses of `UnjamIntegralCommand` with new marker interface `UnjamCommandInterface` to allow custom agitator unjam behavior. Any desired unjam behavior can be put into an implementer of `UnjamCommandInterface` and fed into the `MoveUnjamIntegralComprisedCommand`.
- Added copy assign operators to `transforms::Position` and `transforms::Vector`, as well as dot product, magnitude, and interpolation helpers.
- Expand `DjiSerial` Rx buffer to 1024 bytes.
- Remove check in `addMap()` preventing mappings with equal remote map states to allow for different command mapping implementations with different behaviors using the same remote state.

## May 2024

### Breaking Changes
- Ballistics now uses `AbstractKinematicState` instead of `MeasuredKinematicState`. This is a breaking change.
  - The previous functionality is still present in `SecondOrderKinematicState`, so migrating over 
      would involve replacing all usages of `MeasuredKinematicState` with this.
  - This allows teams to define custom motion models for their kinematic states by extending
      `AbstractKinematicState` and implementing `projectForward(float dt)`
  - Accessing the initial position has been replaced with `.projectForward(0)`

## April 2024

- Updated Ref Serial to support version 1.6.1. This has major breaking changes, but these are nessecary for working robots. See [this document](./extended-changelogs/ref-serial-1.6.1-changes.md) for more information.

- Added in I2C support for development board type A

- Make subsystem getName() const.

- Replaced `ContiguousFloat` with `WrappedFloat`

  - "`[x]=`" operators are now overloaded for arithmetic between WrappedFloats with identical bounds (Replaces `WrappedFloat.shiftUp/Down`)

  - `WrappedFloat.difference` is now `WrappedFloat.minDifference` and returns a float

  - `WrappedFloat.get/setValue` is now `WrappedFloat.get/setWrappedValue`, with the addition of `WrappedFloat.get/setUnwrappedValue`

## March 2024

- Added in constants for motor max output for the GM6020 and C620 motor controller
- Minor change to command mapping to allow for easier extended command mappings

## Febuary 2024

- Updates to transform to compute roll, pitch, and yaw
- Updated IMU with fixes to calibration and added support for variable calibration periods

## January 2024

- Actually wait for semaphore to be released in ref serial transmitter before trying to write again.

## November 2023

- Added `registerAndInitialize` function to `tap::control::Subsystem` to simplify the repetitive
  process of registering & initializing subsystems.

## October 2023
- Added `timeRemaining` method to `Timeout` class.

## July 2023

- Check to make sure message length of interrobot communications is not 1 byte; it seems that this is an undocumented requirement for a successful transmission.
- `CMSISMat` supports costly copy constructor/assignment

### Transforms Library

- `Position` class and `Vector` class which represents a difference in `Position`s
    - These differing types allow transforms to be more clear about the mathematical objects they 
    operate on
- `Orientation` class 
- `Transform` class which represents a transformation from one coordinate frame to another.
- Added `cross` and `fromEulerAngles` to `math_user_utils`

## June 2023

### Breaking Changes
- SafeDisconnect mode is now handled with `refreshSafeDisconnect`, which is run instead of `refresh` when in SafeDisconnect.
  - SafeDisconnect will still deschedule all commands when it is entered
  - Fix: All subsystems must now override `refresh` and `refreshSafeDisconnect`
- `ChassisSubsystemInterface` no longer has a `get\[Left/Right\]\[Front/Back\]RPM()` method
  - Fix: Remove `override` from these method headers  

### New Features
- `MotorInterface`s can now have their motor encoder values zero'd
- `BilinearInterpolator`: Can interpolate between values within a map of `<(x,y), VALUE>`
- `CMSISMat` now has negate and scale operators
- `Can` classes are now protected instead of private
- `taproot-scripts` submodule has been updated so that different taproot branches can be tracked instead of just `release` or `develop`  

## April 2023

### Breaking changes

- `tap::motorInterface` has two new methods to get position in radians. This is implemented already in `DjiMotor` and `DoubleDjiMotor`.
` adds `interpolateLinear2D` to math_user_utils, which is a bilinear interpolator for regularly-spaced datasets.

## November 2022

### Breaking changes

- `Remote::getWheel` removed and replaced with a new (normalized) option for `Remote::getChannel`.

## May 2022

### Breaking changes

- `MotorSim` and `DjiMotorSimHandler` APIs slightly changed, though functionality remains the same.
- Ballistics now provide projectile travel time as an output parameter

### All changes

- `GovernorLimitedCommand` added, a wrapper command that allows one to specify various
  conditions for the command being wrapped to run (!206).
- `RefSerialData`'s `MechanismID` enum values were changed to reflect changes in the RoboMaster ref
  serial protocol.
- `IntegrableSetpointSubsystem` and associated commands added. This allows you to move the integral
  of some setpoint around. An example of where this is useful would be velocity control of a motor,
  where you want to directly control the velocity of the motor and indirectly control the position
  (!188).
- Ballistics now provide projectile travel time as an output parameter

## April 2022

### Breaking changes

- Ref serial transmitter class added, which pulls out transmission-related code from the `RefSerial`
  class. Furthermore, functions that transmit directly handle bandwidth considerations in the
  `RefSerialTransmitter` class. An instance of this class should be instantiated for each
  protothread that ref serial transmissions will be performed in (!161, #175).
- Ref serial data `AddGraphicOperation` enum renamed to `GraphicOperation`.
- `taproot:modm_repo_lb` option has been removed since it is not necessary for the user to specify this (!181, #82)

### All changes

- Ref serial transmitter class allows for multiple protothreads to safely and concurrently interact
  with the ref serial UART port (!161, #175).
- Ref serial receiver now decodes game type and warning status information (!184).

## March 2022

### Breaking changes

- The `Mpu6500` object is now generated from the module `taproot:communication:sensors:imu:mpu6500`.
  `mpu6500.hpp` is now in the directory `src/tap/communication/sensors/imu`. The namespace structure
  has been updated accordingly (#18, !119).
- The `Mpu6500TerminalSerialHandler` has been renamed to `ImuTerminalSerialHandler`. This driver is
  no longer generated as part of `drivers.hpp`. You should put this object in your drivers object if
  you want to use it. This serial handler now takes in a pointer to a `ImuInterface` object (#18,
  !119).
- `SmoothPid` constructor that takes in individual gains (as opposed to a parameter struct) removed (!163).

### All changes

- Unit tests added for `Remote` class.
- Minor improvements to `DjiMotorTxHandler` class. Some API function names have been changed.
- `ImuInterface` added such that IMUs can be interchanged in various situations (#18, !119).
- `ImuMenu` added that prints information about an instance of an `ImuInterface` (#18, !119).
- Zero-length UART messages are no longer thrown out by DjiSerial class.
- CAN 2 pin defines for the Type C board are no longer incorrect (!165).

## February 2022

### Breaking changes

- Namespace `tap::serial` renamed to `tap::communication::serial`.
- `DJISerial` class no longer has a `send` function because it was clunky and unintuitive to use.

### All changes

- The BMI088 IMU on the RoboMaster Development Board Type C is now supported. The API is very
  similar to the `Mpu6500` class, with functions to get the accelerometer/gyroscope/angle data. The
  IMU by default connects but doesn't calibrate when the `initialize` function is called. To
  calibrate the IMU, call the `requestRecalibration` function, which will cause the IMU to stop
  computing angle data for a couple seconds while the IMU is calibration. For calibration to be
  performed correctly, the BMI088 should be level (#18, !96).
- To compliment the referee serial class, a new `StateHudIndicator` object added to store state and
  update graphics based on its internal state (!102).
- Minor cleanup to the `DJISerial` class (including tests).
- Add some utilities to the `DJISerial::SerialMessage` class.
- Add generic `RemoteMapState` constructor (!137).
- Fixed `Profiler` class. You can now use the `PROFILE` macro when profiling is enabled without the
  system running out of memory.
- Add parameters to `Bmi088` `initialize` function to allow the user to configure gains on the
  Mahony algorithm, add `reset` function to Mahony algorithm (!141)

## January 2022

### Breaking changes

- `tap::controls::ControlOperatorInterface` has been removed from Taproot. We have added it to [our
  personal open-source project, aruw-mcb](https://gitlab.com/aruw/controls/aruw-mcb) for those who
  would like to keep up with our implementation of that feature, but it will need to be
  added/implemented externally going forward in order to keep the functionality it provides.
- `tap::controls::chassis::PowerLimiter` API changed significantly. The constructor now takes in
  less parameters than before and their purpose is different. Also, rather than modifying motor
  outputs directly, the power limiter returns a fraction and it is the user's responsibility to
  multiply the motor output by this fraction.
- in the `Mpu6500` class, function called `initialized` changed to `getImuState` since the mpu's
  hardware can be "initialized" but not necessarily calibrated/ready to use.
- `Mpu6500` class contains `requestCalibration` function, which when called the mpu6500 enters a
  calibration state and the mpu6500 recomputes calibration parameters (!12, #123). You should call
  this function in user code to ensure proper calibration of the IMU.
- The `HoldRepeatCommandMapping` requires an extra parameter in its constructor. See all changes for
  more details.
- Almost everything in `tap::control::setpoint` has changed (!49). Most will fail loudly (i.e.: will
  cause compilation errors). Those that are potentially more insidious are documented below:
  - Order of parameters in constructor for `tap::control::setpoint::MoveUnjamComprisedCommand`
    changed (grouped by usage now). This is VERY IMPORTANT to catch and check as compiler may not
    throw warning.
- When testing, `setTime` has bene replaced by a `ClockStub` object that you should use to control
  the time in the context of a test.

### All changes

- `DjiMotor` comments improved (!95).
- Small improvement to command scheduler subsystem refresh loop logic.
- `tap::communication::sensors::current::CurrentSensorInterface` and `AnalogCurrentSensor` added,
  which are software constructs for current sensors.
- Power limiting logic improved and simplified, interfaces with a generic `CurrentSensorInterface`
  (!92).
- Taproot tests now build on Windows without warnings (!103).
- `tap::controls::ControlOperatorInterface` deprecated (!105).
- `tap::controls::turret::TurretSetpointCommand` deprecated.
- Tests added for commands in `tap::control::setpoint` (!49)
- `Mpu6500` class contains `requestCalibration` function, which when called the mpu6500 enters a
  calibration state when `isReady` returns `false` and the mpu6500 recomputes calibration
  parameters.
- `HoldRepeatCommandMapping` now has an extra parameter `endCommandsWhenNotHeld`. When set to false,
  commands are not forcibly ended when the remote state transitions from being held to hot held and
  instead are left to end naturally (or until interrupted by something else) (#95, !114).
- Print a clearer error message when the chosen compiler is not found on PATH. (!97)
- HAL options may be now passed to modm's project.xml file (!116).
- Support for UART ports 7 and 8 added to dev board type A (!116).
- Baud rates in `dji_serial.cpp` configurable via the project.xml file (#50, !116).
- Various improvements to the commands in `tap::control::setpoint` (!49).
  - Unjamming logic more straightforward.
  - MoveCommand pause after rotate time now functions as described.
- A `ClockStub` object has been added to allow the user to control the time during testing. This is
  a more refined approach that replaces the `setTime` function previously in
  `src/tap/arch/clock.hpp`.

## December 2021

### Breaking changes

- The `ErrorController` now no longer displays errors on the LEDs of the RoboMaster Type A board.
  Now, to create an error using the `RAISE_ERROR` macro, you only pass in a pointer to a
  `tap::Drivers` object and a description (i.e. `RAISE_ERROR(drivers, "crc failure")`).
- Minor cleanup to the `RefSerial` object. Some fields in the `RefSerial` class now use
  `MODM_FLAGSX`. See [modm's
  documentation](https://modm.io/reference/module/modm-architecture-register/) for how these work.
  Structs and enum values previously stored directly inside the `RefSerial` object now must be
  prefixed by `Rx` or `Tx`.
- `tap::communication::serial::ITerminalSerialCallback` interface renamed to
  `tap::communication::TerminalSerialCallbackInterface`.
- The drivers object is now generated in `taproot/src/tap` rather than in some user directory. To
  append your own drivers to the `tap::Drivers` object, inherit `tap::Drivers`.
- `tap::controls::chassis::PowerLimiter` API changed significantly. The constructor now takes in
  less parameters than before and their purpose is different. Also, rather than modifying motor
  outputs directly, the power limiter returns a fraction and it is the user's responsibility to
  multiply the motor output by this fraction.
- in the `Mpu6500` class, function called `initialized` changed to `getImuState` since the mpu's
  hardware can be "initialized" but not necessarily calibrated/ready to use.
- `Mpu6500` class contains `requestCalibration` function, which when called the mpu6500 enters a
  calibration state and the mpu6500 recomputes calibration parameters (!12, #123). You should call
  this function in user code to ensure proper calibration of the IMU.

### All changes

- Tests were added to the referee serial class. Minor logic/bug changes were made based on the tests
  (!80, #80).
- Robot-to-robot interaction handling was added to the `RefSerial` class. One can send a
  robot-to-robot message via `sendRobotToRobotMsg` and register a callback with the `RefSerial`
  object via `attachRobotToRobotMessageHandler`. This functionality is still in the beta-testing
  phase and needs further validation (!80, #80).
- `CanRxHandler` class now supports can ids between `0x1e4` and `0x224`. (!84, #124)
- Tests added to terminal serial and various bugs in related classes were removed (!67, #58).
- The `CommandScheduler` is now able to safely remove all commands when a user-specified
  "disconnected" state occurs. One can pass a `SafeDisconnectFunction` functor to the
  `CommandScheduler` to determine what causes a "disconnected" state (!75).
- `modm:math:interpolation` module now generated (!93).

## November 2021

### Breaking changes

- The `sim-modm` directory in a generated Taproot instance has a new structure. Files are now in
  `sim-modm/hosted-TARGET/modm`, where `TARGET` is `linux`, `windows` or `darwin`. Make sure to
  delete and cleanly re-generate your Taproot instance, and update your SConstruct file as shown in
  the template project.
- The `ErrorController` now no longer displays errors on the LEDs of the RoboMaster Type A board.
  Now, to create an error using the `RAISE_ERROR` macro, you only pass in a pointer to a
  `tap::Drivers` object and a description (i.e. `RAISE_ERROR(drivers, "crc failure")`).

### All changes

- "sim-modm" instance is now generated for all three major desktop platforms, with hardware builds
  and testing environments fully supported on each. (!73, #96, #15)
