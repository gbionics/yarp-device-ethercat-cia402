# Store Home Position 🧭

## Overview 🌟
Utility to set and persist the home position on EtherCAT CiA402 drives. It:
- Switches drives to Homing mode (CiA-402) and performs homing using method 37 (set current position as home) or 35.
- Optionally applies a home offset before persisting.
- Sets the vendor flag to restore the home at boot (if requested).
- Stores the new configuration to non-volatile memory.
- After calibration, reads back both encoder channels (raw and adjusted positions, resolutions) and writes a TOML snapshot to disk for later use.

The app talks via SDOs only; it does not drive motion trajectories. It operates on all slaves detected on the specified network interface.

> Note: Tested on Linux. Requires raw-socket/network admin capabilities to access EtherCAT via SOEM.

## Post-Install Setup 🔧
After installing the project, make sure the binary can find its libraries and has the required capabilities:

```console
patchelf --add-rpath $(dirname $(dirname $(which yarp-cia402-store-home-position)))/lib $(which yarp-cia402-store-home-position)
sudo setcap 'cap_net_raw,cap_net_admin+ep' "$(readlink -f $(which yarp-cia402-store-home-position))"
```

## Usage 🚀

Run the tool with your network interface and desired options:

```bash
yarp-cia402-store-home-position \
	--ifname eth0 \
	--method 37 \
	--home-offset 0 \
	--timeout-ms 2000 \
	--restore-on-boot 1 \
	--toml-output encoder_home_data.toml
```

You will be asked for an interactive confirmation (press ENTER) before proceeding.

### Options ⚙️
- `--ifname`: Network interface used by SOEM (default: `eth0`).
- `--method`: Homing method `37` (current position as home) or `35` (default: `37`).
- `--home-offset`: Additional home offset in encoder ticks (default: `0`).
- `--timeout-ms`: Homing attained wait timeout in milliseconds (default: `2000`).
- `--restore-on-boot`: `1/true` to mark the drive as referenced at boot, `0/false` otherwise (default: `1`).
- `--toml-output`: Path of the TOML file written after calibration (default: `encoder_home_data.toml`).

### Configuration file (optional) 📝
You can also provide options through a YARP `.ini` file and pass it with `--from`:

```ini
# store_home.ini
ifname eth0
method 37
home-offset 0
timeout-ms 2000
restore-on-boot true
toml-output encoder_home_data.toml
```

Run with:

```bash
yarp-cia402-store-home-position --from store_home.ini
```

## What It Does 🧠
For each discovered slave on `--ifname`:
1. Sets `0x6060 = 6` (Homing mode), writes `0x6098 = method` and optional `0x607C = home-offset`.
2. Triggers homing by toggling Controlword bit 4 and polls Statusword until homing attained.
3. Sets vendor flag `0x2005:02` according to `--restore-on-boot`.
4. Saves parameters with `0x1010:01 = 'evas'`.
5. Reads encoder feedback and writes a TOML snapshot.

### Encoder TOML snapshot 📄
After all slaves are calibrated, the tool reads the following SDOs and writes `--toml-output`:

| SDO        | Sub  | Type  | TOML field                         | Description                                      |
|:----------:|:----:|:-----:|------------------------------------|--------------------------------------------------|
| `0x2110`   | `:03`| UDINT | `counts_per_revolution`            | Encoder 1 resolution (counts/rev)                |
| `0x2111`   | `:01`| UDINT | `raw_position` / `raw_position_degrees`      | Encoder 1 raw position (unprocessed)  |
| `0x2111`   | `:02`| DINT  | `adjusted_position` / `adjusted_position_degrees` | Encoder 1 position after polarity/ST offset |
| `0x2112`   | `:03`| UDINT | `counts_per_revolution`            | Encoder 2 resolution (counts/rev)                |
| `0x2113`   | `:01`| UDINT | `raw_position` / `raw_position_degrees`      | Encoder 2 raw position (unprocessed)  |
| `0x2113`   | `:02`| DINT  | `adjusted_position` / `adjusted_position_degrees` | Encoder 2 position after polarity/ST offset |

Each encoder table also contains `raw_to_degrees_factor = 360.0 / counts_per_revolution`.

**Example output:**
```toml
[slave_1]
name = "SOMANET"

[slave_1.encoder1]
raw_position            = 2461952
raw_position_degrees    = 350.753906
adjusted_position       = 2461952
adjusted_position_degrees = 350.753906
counts_per_revolution   = 2527232
raw_to_degrees_factor   = 0.00014245014

[slave_1.encoder2]
raw_position            = 115712
raw_position_degrees    = 16.518066
adjusted_position       = 0
adjusted_position_degrees = 0.0
counts_per_revolution   = 2521124
raw_to_degrees_factor   = 0.00014280147
```

> Note: `adjusted_position` is zero until the encoder index has been found (incremental encoders).

## Safety Notes ⚠️
- Ensure the current mechanical position is the desired reference when using method 37.
- Verify axis is in a safe state; method 35 may involve motion as per drive firmware.
- Using `--restore-on-boot 1` makes the drive boot already referenced; validate alignment and encoder polarity beforehand.

