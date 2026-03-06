# YARP CiA-402 EtherCAT Device {#mainpage}

[TOC]

## Overview

This repository provides a **YARP device plugin** for **EtherCAT CiA-402** drives,
built on top of [SOEM](https://github.com/OpenEtherCATsociety/SOEM).

It exposes standard YARP motor-control interfaces (`IPositionControl`,
`IVelocityControl`, `ITorqueControl`, `ICurrentControl`, `IEncodersTimed`, …)
while communicating with drives that implement the CiA-402 (IEC 61800-7-201)
profile over EtherCAT (CoE).

### Key features

- **CiA-402 state machine** — automatic power-stage transitions
  (Shutdown → SwitchedOn → OperationEnabled) with fault handling.
- **Multiple control modes** — Profile Position (PP), Cyclic Synchronous
  Position (CSP), Cyclic Synchronous Velocity (CSV), and Cyclic Synchronous
  Torque (CST).
- **Dual-encoder support** — configurable feedback selection between up to
  two encoders per axis, with automatic shaft transformations.
- **Simple PID tuning** — optional joint-side PID gains automatically
  converted to drive units.
- **Home-position utility** — standalone tool to set and persist homing
  parameters on the drive's non-volatile memory.
- **Calibration check utility** — read-only diagnostic that compares live
  encoder readings against a reference TOML snapshot and produces a Markdown
  report.

## Documentation sections

| Page | Description |
|------|-------------|
| @subpage protocol_map | PDOs, SDOs, unit conversions — the single source of truth for all EtherCAT objects used by the driver. |
| @subpage modes_and_setpoints | How YARP control modes map to CiA-402 operation modes and which targets are written. |
| @subpage dual_encoder_handling | Encoder mounting semantics, feedback selection, and shaft transformations. |
| @ref yarp::dev::CiA402MotionControl | Main device driver API reference. |
| @ref CiA402::EthercatManager | Low-level EtherCAT master (SOEM wrapper). |
| @ref CiA402::StoreHome37 | Home-position persistence utility. |
| @subpage check_encoder_calibration | Encoder calibration check utility — compare live readings vs. reference TOML. |
| @ref CiA402::CheckEncoderCalibration | Calibration-check utility API reference. |

## Getting started

### Prerequisites

- [CMake](https://cmake.org/) ≥ 3.8
- [YARP](https://www.yarp.it/)
- [SOEM](https://github.com/OpenEtherCATsociety/SOEM)
- [toml++](https://github.com/marzer/tomlplusplus) (TOML parser for C++)
- Linux (tested; raw-socket access required for EtherCAT)

### Build

```bash
git clone https://github.com/gbionics/yarp-device-ethercat-cia402.git
cd yarp-device-ethercat-cia402
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### Quick run

```bash
# Ensure YARP can find the plugin
export YARP_DATA_DIRS=/path/to/install/share/yarp:$YARP_DATA_DIRS

# Grant raw-socket capabilities
sudo setcap cap_net_raw,cap_net_admin+ep $(which yarprobotinterface)

# Launch
yarprobotinterface --config config/robot/template_1_motor/config.xml
```

## License

BSD-3-Clause — see [LICENSE](https://github.com/gbionics/yarp-device-ethercat-cia402/blob/main/LICENSE).
