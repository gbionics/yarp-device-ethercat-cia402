# Check Encoder Calibration {#check_encoder_calibration}

[TOC]

## Purpose

After a homing / calibration run (performed by @ref CiA402::StoreHome37), the
encoder positions are persisted on each drive's non-volatile memory and a TOML
snapshot is written to disk. Over time — due to mechanical play, power cycles,
firmware updates, or unexpected faults — the actual encoder readings may drift
from the reference values recorded in that TOML file.

**`check-encoder-calibration`** is a read-only diagnostic tool that:

1. Reads the reference TOML file produced by `store-home-position`.
2. Connects to all EtherCAT slaves via SDO (no cyclic motion is commanded).
3. Reads back each encoder channel's current raw and adjusted positions plus
   resolution parameters.
4. Computes the delta between the recorded and live values.
5. Writes a **Markdown report** with summary and per-slave comparison tables.

> The ring stays in SAFE-OP / PRE-OP throughout; no drive parameters are
> modified.

## SDOs Read

| SDO      | Sub    | Type  | Description                        |
|:--------:|:------:|:-----:|------------------------------------|
| `0x2110` | `:03`  | UDINT | Encoder 1 resolution (counts/rev)  |
| `0x2111` | `:01`  | UDINT | Encoder 1 raw position             |
| `0x2111` | `:02`  | DINT  | Encoder 1 adjusted position        |
| `0x2112` | `:03`  | UDINT | Encoder 2 resolution (counts/rev)  |
| `0x2113` | `:01`  | UDINT | Encoder 2 raw position             |
| `0x2113` | `:02`  | DINT  | Encoder 2 adjusted position        |

These are the same objects written to the TOML snapshot by
@ref CiA402::StoreHome37 — see @ref protocol_map for the full object
dictionary.

## Command-Line Usage

```bash
yarp-cia402-check-encoder-calibration \
    --ifname eth0 \
    --toml-input joint_calibration_2026_03_04_18_54_35.toml \
    --report-output calibration_check.md
```

### Parameters

| Key              | Type   | Description                                                  | Default |
|:----------------:|:------:|--------------------------------------------------------------|:-------:|
| `ifname`         | string | Network interface used by SOEM                               | `eth0`  |
| `toml-input`     | string | **(required)** Reference TOML file from `store-home-position`| —       |
| `report-output`  | string | Path of the Markdown report to write                         | `encoder_calibration_check_YYYY_MM_DD_HH_MM_SS.md` |

Parameters can also be provided via a YARP `.ini` file:

```ini
# check_calibration.ini
ifname eth0
toml-input joint_calibration_2026_03_04_18_54_35.toml
report-output calibration_check.md
```

```bash
yarp-cia402-check-encoder-calibration --from check_calibration.ini
```

## Report Format

The generated Markdown file has three sections:

### Header

| | |
|:--|:--|
| **Date** | 2026-03-05 14:30:00 |
| **Reference TOML** | `joint_calibration_2026_03_04_18_54_35.toml` |
| **Slaves checked** | 3 |

### Summary Table

One row per slave with the adjusted-position delta in degrees for quick
pass / fail inspection:

| Slave | Name    | Enc1 Adj Δ (deg)  | Enc2 Adj Δ (deg)  |
|:-----:|:--------|------------------:|------------------:|
| 1     | SOMANET | +0.001234         | −0.000567         |
| 2     | SOMANET | +0.000000         | +0.000012         |

### Per-Slave Detailed Tables

Two tables per slave (Encoder 1, Encoder 2) with full comparison:

| Metric                     | Reference (TOML) | Current (Live) | Delta     |
|:---------------------------|-----------------:|---------------:|----------:|
| Raw position (counts)      | 2461952          | 2461960        | +8        |
| Raw position (deg)         | 350.753906       | 350.754047     | +0.000141 |
| Adjusted position (counts) | 2461952          | 2461960        | +8        |
| Adjusted position (deg)    | 350.753906       | 350.754047     | +0.000141 |
| Counts per revolution      | 2527232          | 2527232        | 0         |
| Raw-to-degrees factor      | 0.000142         | 0.000142       | +0.000000 |

## Typical Workflow

```text
  store-home-position              check-encoder-calibration
  ───────────────────              ─────────────────────────
  Calibrate all drives   ──►  reference.toml
                                          │
                                          ▼
                                  Read reference + live SDOs
                                          │
                                          ▼
                                  Write Markdown report
```

```bash
# 1. Calibrate (done once or when needed)
yarp-cia402-store-home-position --ifname eth0 --toml-output reference.toml

# 2. Later, verify calibration integrity
yarp-cia402-check-encoder-calibration --ifname eth0 --toml-input reference.toml
```

## API Reference

- @ref CiA402::CheckEncoderCalibration — main application class.
