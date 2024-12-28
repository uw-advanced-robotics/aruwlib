# Modm Update - December 2024

This update brings modm up to date along with many tools and dependencies that depend on or work alongside it.

Updating all these libraries and dependencies at the same time required a lot of simultaneous changes to the taproot source code, build scripts, CI, and docker containers.
Note that there is potential for some things to be broken, although substantial effort was put into making the transition as smooth as possible.
The major changes are listed here.
For a complete list, please view the commit history.

## Breaking changes
### Transition from Protothreads to Fibers
Modm has added "Fibers" which are a new method for implementing concurrency.
They are intended to replace Protothreads and Resumable functions (which are buggy and annoying to deal with).

To ease the transition to fibers, modm added a fiber compatibility backend to protothreads and resumable functions.
**This backend is now enabled by default.**

**These changes require teams to change the way they handle concurrency.**
See the [modm fiber module documentation](https://github.com/modm-io/modm/blob/develop/src/modm/processing/fiber/module.md) for more information.

The only Taproot component that directly implements a Protothread is the `MPU6500`.
Resumable functions are provided by the `RefSerialTransmitter`, `StateHUDIndicator`, and `SH1106` components.

### Tool versions
- ARM GCC and GCC should be updated to 13.3.
- Scons should be updated to 4.8.1.
- lbuild should be updated to 1.21.8.
- Other tools should be updated to compatible versions.
Generally, Ubuntu 24.04 is a good point of reference for package versions.
You may also try newer package versions, but use caution as compatibility is not guaranteed.

### Other
UART port TX and RX buffer sizes (default 256) are now controlled using the
`taproot:communication:serial:uart_port_{port_num}.tx_buffer_size`
and
`taproot:communication:serial:uart_port_{port_num}.rx_buffer_size`
options in your `project.xml` configuration.

The `modm:math:saturated` module was renamed to `modm:math:saturation`.