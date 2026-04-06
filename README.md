# yarp-device-ethercat-CiA402 🚀

## Overview 🌟
This repository provides a YARP device plugin for EtherCAT CiA402 drives.

## Compilation 🛠️

### Prerequisites 📋
Ensure the following dependencies are installed:
- [CMake](https://cmake.org/) (version 3.8 or higher)
- [YARP](https://www.yarp.it/)
- [SOEM](https://github.com/OpenEtherCATsociety/SOEM)
- [toml++](https://github.com/marzer/tomlplusplus) (TOML parser for C++)

> **Note**: This device has been tested only on Linux systems.

### Build Steps 🧩

1. Clone the repository:
   ```bash
   git clone https://github.com/ami-iit/yarp-device-ethercat-cia402.git
   cd yarp-device-ethercat-cia402
   ```
2. Create a build directory and navigate into it:
   ```bash
   mkdir build && cd build
   ```

3. Configure the project with CMake:
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install
   ```

4. Build the project:
   ```bash
   make
   ```

5. Install the plugin:
   ```bash
   make install
   ```

## Usage 🚀

### Running the Plugin 🏃‍♂️

After building the project, the plugin can be loaded into a YARP-based application. Make sure the `YARP_DATA_DIRS` environment variable includes the path to the plugin's configuration files:
```bash
export YARP_DATA_DIRS=/path/to/install/share/yarp:$YARP_DATA_DIRS
```

### Configuration ⚙️
The plugin requires a configuration file defining the EtherCAT network and device parameters. An example can be found at: [`config/robot/template_1_motor/config.xml`](config/robot/template_1_motor/config.xml)

#### Device parameters

The `CiA402MotionControl` device accepts the following parameters.

| Parameter | Type | Required | Description |
|---|---|---|---|
| `ifname` | string | Yes | Network interface name used by SOEM (for example `eth0`). |
| `num_axes` | int | Yes | Number of controlled axes. |
| `period` | float | Yes | Driver periodic thread period in seconds. |
| `enc1_mount` | list(string) | Yes | One item per axis. Allowed values: `motor`, `joint`. |
| `enc2_mount` | list(string) | Yes | One item per axis. Allowed values: `motor`, `joint`, `none`. |
| `position_feedback_joint` | list(string) | Yes | One item per axis. Allowed values: `6064`, `enc1`, `enc2`. |
| `position_feedback_motor` | list(string) | Yes | One item per axis. Allowed values: `6064`, `enc1`, `enc2`. |
| `velocity_feedback_joint` | list(string) | Yes | One item per axis. Allowed values: `606C`, `enc1`, `enc2`. |
| `velocity_feedback_motor` | list(string) | Yes | One item per axis. Allowed values: `606C`, `enc1`, `enc2`. |
| `inverted_motion_sense_direction` | list(bool) | Yes | One flag per axis. If `true`, command and feedback sign are inverted for that axis. |
| `position_window_deg` | list(double) | Yes | One item per axis. Position reached window in joint degrees (written through `0x6067`). |
| `timing_window_ms` | list(double) | Yes | One item per axis. Position reached timing window in milliseconds (`0x6068`). |
| `pos_limit_min_deg` | list(double) | Yes | One item per axis. Joint-side minimum limit in degrees. |
| `pos_limit_max_deg` | list(double) | Yes | One item per axis. Joint-side maximum limit in degrees. |
| `use_position_limits_from_config` | list(bool) | Yes | One flag per axis. If `true`, write `0x607D` from config. If `false`, read `0x607D` from drive and use that. |
| `axes_names` | list(string) | Yes | One name per axis, returned by axis info APIs. |
| `first_slave` | int | No | First EtherCAT slave index. Default: `1`. |
| `expected_slave_name` | string | No | Optional expected slave name for sanity checks. |
| `pdo_timeout_us` | int | No | PDO receive timeout in microseconds. Default: EthercatManager internal default. |
| `enable_dc` | bool | No | Enable distributed clocks (SYNC0). Default: `true`. |
| `dc_shift_ns` | int | No | SYNC0 phase shift in nanoseconds. Default: `0`. |
| `use_simple_pid_mode` | bool | No | If `true`, set `0x2002:00 = 1` (Simple PID). If `false`, set `0x2002:00 = 2` (Cascaded PID). Default: `false`. |
| `simple_pid_kp_nm_per_deg` | list(double) | No (pair) | Joint-side Kp values in Nm/deg for simple PID mode. Must be provided together with `simple_pid_kd_nm_s_per_deg`. |
| `simple_pid_kd_nm_s_per_deg` | list(double) | No (pair) | Joint-side Kd values in Nm*s/deg for simple PID mode. Must be provided together with `simple_pid_kp_nm_per_deg`. |
| `max_torque_joint_nm` | list(double) | No | Optional joint-side maximum torque in Nm. If provided, each value is converted to motor side and written to `0x6072:00` (per-thousand of `0x6076`). If omitted, the value already stored in the drive is used. |

All list parameters must contain exactly `num_axes` elements.

To select the drive internal position controller structure, configure:

- `use_simple_pid_mode` — boolean. When `true` the driver sets `0x2002:00 = 1` (Simple PID);
   when `false` it sets `0x2002:00 = 2` (Cascaded PID).

To program *simple PID* gains (only meaningful when `use_simple_pid_mode=true`), provide:

- `simple_pid_kp_nm_per_deg` — list of joint-side Kp values in Nm/deg
- `simple_pid_kd_nm_s_per_deg` — list of joint-side Kd values in Nm*s/deg

The two gain keys are optional as a pair: if exactly one of them is provided the driver
errors out.

When both are present (and `use_simple_pid_mode=true`) the device converts them to the
drive units (mNm/inc and mNm*s/inc), and writes `0x2012:01..03` with Ki clamped to zero.
If `use_simple_pid_mode=false`, any `simple_pid_*` keys are ignored (with a warning) to
avoid programming/printing `0x2012` with the wrong unit interpretation.

See
[`doc/protocol_map.md`](./doc/protocol_map.md) for the conversion details.

### Setting Up `yarprobotinterface` 🛠️
To ensure that the `yarprobotinterface` binary has the correct permissions and can locate its dependencies, execute:

```console
patchelf --add-rpath $(dirname $(dirname $(which yarprobotinterface)))/lib $(which yarprobotinterface)
sudo setcap cap_net_raw,cap_net_admin+ep $(which yarprobotinterface)
```

### Example 💡
To run the plugin with a specific configuration:
```bash
yarprobotinterface --config config/robot/template_1_motor/config.xml
```

## Supported Drives 🛠️
This plugin has been primarily tested with Synapticon drives. While it may be compatible with other EtherCAT drive models or manufacturers, some modifications might be necessary to ensure proper functionality. This is due to the plugin’s use of a custom Process Data Object (PDO) mapping, which extends beyond the standard CiA402 specification.

If you're looking to adapt the plugin for different hardware, we encourage you to open an issue or contribute improvements.

### Additional notes 📝
For more details, see:
- Protocol map — PDOs, SDOs, and conversion formulas: [doc/protocol_map.md](./doc/protocol_map.md)
- Modes and setpoints — available control modes and targets: [doc/modes_and_setpoints.md](./doc/modes_and_setpoints.md)
- Dual encoder handling — mounts, sources, and transformations: [doc/dual_encoder_handling.md](./doc/dual_encoder_handling.md)


## License 📜
This project is licensed under the BSD-3-Clause License. See the [`LICENSE`](LICENSE) file for details.
