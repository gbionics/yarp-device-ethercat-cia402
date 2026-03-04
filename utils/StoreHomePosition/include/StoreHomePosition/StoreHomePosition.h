// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef STORE_HOME_POSITION_H
#define STORE_HOME_POSITION_H

#include <cstdint>
#include <memory>
#include <string>

#include <yarp/os/ResourceFinder.h>

namespace CiA402
{

/**
 * @brief Calibration utility: home all CiA-402 drives on the bus and persist the result to flash.
 *
 * This application performs a complete homing sequence on every EtherCAT slave found on the
 * specified network interface using CiA-402 homing methods 37 (current position as home, no index)
 * or 35 (current position as home, with index). The procedure is executed entirely via SDO
 * communication, so the ring never needs to reach OPERATIONAL state and no cyclic motion is
 * commanded.
 *
 * After all slaves have been homed and the home position has been saved to their non-volatile
 * memory, the application reads back the raw and adjusted encoder positions of both encoder
 * channels together with their resolution parameters and writes the data to a TOML file.
 * This file can subsequently be used by higher-level software to reconstruct the conversion
 * from raw encoder counts to degrees without querying the drives again.
 *
 * **Per-slave homing sequence:**
 *  1. Set operation mode to Homing (0x6060 = 6).
 *  2. Write the selected homing method to 0x6098 (35 or 37).
 *  3. Optionally write a home offset to 0x607C.
 *  4. Start homing by toggling bit 4 of the Controlword (0x6040) low→high.
 *  5. Poll Statusword (0x6041) bit 12 ("Homing attained") until set; abort on bit 13
 *     ("Homing error") or user-specified timeout.
 *  6. Write vendor-specific restore-on-startup flag to 0x2005:02.
 *  7. Persist all parameters to non-volatile memory via 0x1010:01 = @c 0x65766173 ("evas").
 *
 * **Post-calibration encoder snapshot (written to TOML):**
 *  - Encoder 1 (0x2111): raw position (:01, UDINT), adjusted position (:02, DINT).
 *  - Encoder 2 (0x2113): raw position (:01, UDINT), adjusted position (:02, DINT).
 *  - Encoder resolutions (counts/rev): 0x2110:03 for encoder 1, 0x2112:03 for encoder 2.
 *  - Computed angle in degrees for both raw and adjusted positions of each encoder.
 *  - Conversion factor (degrees/count) for each encoder.
 *
 * @note Only SDO communication is used; the ring stays in SAFE-OP/PRE-OP throughout.
 * @note Setting @c restore-on-boot to @c true causes the drive to boot already referenced,
 *       skipping the homing procedure on the next startup.
 * @note SOEM uses 1-based slave indices; slave numbering in the output TOML follows the same
 *       convention (@c slave_1, @c slave_2, …).
 */
class StoreHome37
{
public:
    /**
     * @brief Default constructor.
     *
     * Allocates the internal implementation object. No EtherCAT communication is started.
     */
    StoreHome37();

    /**
     * @brief Destructor.
     *
     * Releases all resources owned by the implementation. If the EtherCAT master was
     * initialised, it is cleanly shut down.
     */
    ~StoreHome37();

    /// @cond — non-copyable
    StoreHome37(const StoreHome37&) = delete;
    StoreHome37& operator=(const StoreHome37&) = delete;
    /// @endcond

    /**
     * @brief Run the full homing and persistence procedure on all slaves, then write encoder data.
     *
     * Reads configuration from the provided ResourceFinder, discovers all EtherCAT slaves on the
     * specified network interface, executes the per-slave homing sequence described in the class
     * documentation, and finally writes a TOML snapshot of the encoder state to disk.
     *
     * @param rf ResourceFinder populated with the parameters listed below.
     * @return @c true if every slave was homed, persisted, and the TOML file was written
     *         successfully; @c false on any error (initialisation failure, slave not found,
     *         homing error, homing timeout, SDO write failure, file I/O error).
     *
     * @note Expected keys in the ResourceFinder:
     * | Key              | Type        | Description                                           | Default                    |
     * |:----------------:|:-----------:|-------------------------------------------------------|:--------------------------:|
     * | ifname           | string      | Network interface name (e.g. @c "eth0")               | @c "eth0"                  |
     * | method           | int         | CiA-402 homing method: @c 35 or @c 37                 | @c 37                      |
     * | home-offset      | int32       | Additional home offset written to 0x607C              | @c 0                       |
     * | timeout-ms       | int         | Maximum time (ms) to wait for homing attained         | @c 2000                    |
     * | restore-on-boot  | bool / int  | Set drive restore-on-startup flag (0x2005:02)         | @c true                    |
     * | toml-output      | string      | Path of the TOML file written after calibration       | @c "encoder_home_data.toml"|
     */
    bool run(yarp::os::ResourceFinder& rf);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl; ///< Opaque implementation (PIMPL idiom).
};

} // namespace CiA402

#endif // STORE_HOME_POSITION_H