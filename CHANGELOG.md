# Taproot Changelog

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
