# Ref Serial Update 1.6.1

This updates from some previous version of the ref serial system, likely whatever was used in 2022.

For more documentation on the changes, make sure to check out [`ref_serial_data.hpp`](../src/tap/communication/serial/ref_serial_data.hpp) and [`ref_serial.hpp`](../src/tap/communication/serial/ref_serial.hpp).
For implementation details, check out [`ref_serial.cpp`](../src/tap/communication/serial/ref_serial.cpp).

## Breaking changes

`ChassisData` struct:
- Moved `x` and `y` fields into `position` field.
- Deleted `z` field.

`TurretData` struct:
- Removed all barrel speed fields.
    - These are now available with the `MAX_LAUNCH_SPEED_17MM` and `MAX_LAUNCH_SPEED_42MM` static fields in `RefSerialData::Rx`
- Merged all cooling rate fields into `coolingRate` field.
- Merged all heat limit fields into `heatLimit` field.

`RobotBuffStatus` enum and `RobotBuffStatus_t` struct:
- Merged into a single `RobotBuffStatus` struct.
- The buffs are now individual fields instead of enum values.

`RobotData` struct:
- Removed `aerialEnergyStatus` field
- Replaced `robotBuffStatus` with new `RobotBuffStatus` struct instead of flags.


`RFIDActivationStatus` enum and `RFIDActivationStatus_t` struct:
- `32` bits now instead of `8`.
- Fields completely rewritten to accurately reflect current state of refree communication.
- Available options:
    - `BASE_BUFF`
    - `ELEVATED_RING_OWN`
    - `ELEVATED_RING_OPPONENT`
    - `TRAPEZOID_R3_OWN`
    - `TRAPEZOID_R3_OPPONENT`
    - `TRAPEZOID_R4_OWN`
    - `TRAPEZOID_R4_OPPONENT`
    - `POWER_RUNE_ACTIVATION`
    - `LAUNCH_RAMP_FRONT_OWN`
    - `LAUNCH_RAMP_BACK_OWN`
    - `LAUNCH_RAMP_FRONT_OPPONENT`
    - `LAUNCH_RAMP_BACK_OPPONENT`
    - `OUTPOST_BUFF`
    - `RESTORATION_ZONE`
    - `SENTRY_PATROL_OWN`
    - `SENTRY_PATROL_OPPONENT`
    - `LARGE_ISLAND_OWN`
    - `LARGE_ISLAND_OPPONENT`
    - `EXCHANGE_ZONE`
    - `CENTRAL_BUFF`

`GameType` enum:
- Renamed field `ROBOMASTER_COMPETITIONS` to `ROBOMASTER_RMUC`.

## Additions

New supported `RefSerial::MessageType`s:
- `REF_MESSAGE_TYPE_SITE_EVENT_DATA`
- `REF_MESSAGE_TYPE_PROJECTILE_SUPPPLIER_ACTION`
- `REF_MESSAGE_TYPE_DART_INFO`
- `REF_MESSAGE_TYPE_DART_STATION_INFO`
- `REF_MESSAGE_TYPE_GROUND_ROBOT_POSITION`
- `REF_MESSAGE_TYPE_RADAR_PROGRESS`
- `REF_MESSAGE_TYPE_SENTRY_INFO`
- `REF_MESSAGE_TYPE_RADAR_INFO`

Enums:
- `SiteDartHit`
- `SuppilerOutputStatus`
- `DartTarget`
- `AirSupportState`
- `DartStationState`

Flags:
- `SiteData`

Structs:
- `EventData`
- `SupplierAction`
- `RobotPosition`
- `DartInfo`
- `AirSupportState`
- `DartStationInfo`
- `GroundRobotPositions`
- `RadarMarkProgress`
- `SentryInfo`
- `RadarInfo`

`RefereeWarningData` struct:
- Added `count` field.

`GameData` struct:
- Added `eventData` field.
- Added `supplier` field.
- Added `dartInfo` field.
- Added `airSupportData` field.
- Added `dartStation` field.
- Added `positions` field.
- Added `radarProgress` field.
- Added `sentry` field.
- Added `radar` field.

## Other Changes

`RefSerialMenu` class:
- Removed print speed methods.
- Added `print17mm2Heat` method.
- Changed `REF_SERIAL_INFO_LINES` from `8` to `7`