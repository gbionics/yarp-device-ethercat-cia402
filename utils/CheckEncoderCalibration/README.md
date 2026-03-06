# Check Encoder Calibration 🔍

## Overview 🌟
Utility to verify encoder calibration integrity on EtherCAT CiA402 drives. It:
- Connects to all slaves on the specified network interface via SDO.
- Reads the current encoder positions (raw and adjusted) and resolutions for both encoder channels.
- Compares the live values against a reference TOML file previously written by `store-home-position` after calibration.
- Generates a human-readable **Markdown report** with a summary table and per-slave detailed comparison tables showing reference values, current values, and deltas.

This allows an operator to quickly spot whether encoder state has drifted since the last calibration without re-running the homing procedure.

> Note: Tested on Linux. Requires raw-socket/network admin capabilities to access EtherCAT via SOEM.

## Post-Install Setup 🔧
After installing the project, make sure the binary can find its libraries and has the required capabilities:

```console
patchelf --add-rpath $(dirname $(dirname $(which yarp-cia402-check-encoder-calibration)))/lib $(which yarp-cia402-check-encoder-calibration)
sudo setcap 'cap_net_raw,cap_net_admin+ep' "$(readlink -f $(which yarp-cia402-check-encoder-calibration))"
```

## Usage 🚀

Run the tool with the reference TOML file from a previous calibration:

```bash
yarp-cia402-check-encoder-calibration \
	--ifname eth0 \
	--toml-input joint_calibration_2026_03_04_18_54_35.toml \
	--report-output calibration_check.md
```

### Options ⚙️
- `--ifname`: Network interface used by SOEM (default: `eth0`).
- `--toml-input`: **(required)** Path of the reference TOML file produced by `store-home-position`.
- `--report-output`: Path of the Markdown report to write (default: `encoder_calibration_check_<YYYY>_<MM>_<DD>_<HH>_<MM>_<SS>.md`).

### Configuration file (optional) 📝
You can also provide options through a YARP `.ini` file and pass it with `--from`:

```ini
# check_calibration.ini
ifname eth0
toml-input joint_calibration_2026_03_04_18_54_35.toml
report-output calibration_check.md
```

Run with:

```bash
yarp-cia402-check-encoder-calibration --from check_calibration.ini
```

## What It Does 🧠
1. Parses the reference TOML file produced by `store-home-position`.
2. Initializes the EtherCAT master on `--ifname` and discovers all slaves.
3. For each slave found in both the bus and the TOML file, reads the following SDOs:

| SDO        | Sub  | Type  | Description                                      |
|:----------:|:----:|:-----:|--------------------------------------------------|
| `0x2110`   | `:03`| UDINT | Encoder 1 resolution (counts/rev)                |
| `0x2111`   | `:01`| UDINT | Encoder 1 raw position                           |
| `0x2111`   | `:02`| DINT  | Encoder 1 adjusted position                      |
| `0x2112`   | `:03`| UDINT | Encoder 2 resolution (counts/rev)                |
| `0x2113`   | `:01`| UDINT | Encoder 2 raw position                           |
| `0x2113`   | `:02`| DINT  | Encoder 2 adjusted position                      |

4. Compares each value against the reference and computes the delta.
5. Writes a Markdown report.

## Report Structure 📄

The generated Markdown report contains:

### 1. Header
General information: date/time, reference TOML path, number of slaves checked.

### 2. Summary Table
A quick-glance table with one row per slave showing the adjusted-position delta in degrees for both encoder channels:

| Slave | Name    | Enc1 Adj Δ (deg) | Enc2 Adj Δ (deg) |
|:-----:|:--------|------------------:|------------------:|
| 1     | SOMANET | +0.001234         | -0.000567         |
| 2     | SOMANET | +0.000000         | +0.000012         |

### 3. Per-Slave Detailed Tables
For each slave, two tables (Encoder 1 and Encoder 2) with full comparison:

| Metric                    | Reference (TOML) | Current (Live) | Delta     |
|:--------------------------|----------------:|---------------:|----------:|
| Raw position (counts)     | 2461952          | 2461960        | +8        |
| Raw position (deg)        | 350.753906       | 350.754047     | +0.000141 |
| Adjusted position (counts)| 2461952          | 2461960        | +8        |
| Adjusted position (deg)   | 350.753906       | 350.754047     | +0.000141 |
| Counts per revolution     | 2527232          | 2527232        | 0         |
| Raw-to-degrees factor     | 0.000142         | 0.000142       | +0.000000 |

## Example Workflow 🔄

```bash
# 1. Calibrate and save reference (done once)
yarp-cia402-store-home-position --ifname eth0 --toml-output calibration_ref.toml

# 2. Later, verify calibration is still valid
yarp-cia402-check-encoder-calibration --ifname eth0 --toml-input calibration_ref.toml

# 3. Open the generated report
cat encoder_calibration_check_2026_03_05_14_30_00.md
```

## Safety Notes ⚠️
- This tool performs **read-only** SDO communication; it does not modify any drive parameters.
- The EtherCAT ring stays in SAFE-OP/PRE-OP throughout; no cyclic motion is commanded.
- If a slave present on the bus has no matching entry in the TOML reference file, it is skipped with a warning.
