// SPDX-FileCopyrightText: Generative Bionics
// SPDX-License-Identifier: BSD-3-Clause

#ifndef CHECK_ENCODER_CALIBRATION_H
#define CHECK_ENCODER_CALIBRATION_H

#include <cstdint>
#include <memory>
#include <string>

#include <yarp/os/ResourceFinder.h>

namespace CiA402
{

/**
 * @brief Encoder calibration checker: compare live encoder readings against a reference TOML file.
 *
 * This utility connects to all EtherCAT slaves on the specified network interface using SDO
 * communication and reads back the current encoder positions (raw and adjusted) and resolutions
 * for both encoder channels. It then compares these live values against a reference TOML file
 * previously written by @ref CiA402::StoreHome37 (store-home-position application) immediately after calibration.
 *
 * The result is a human-readable Markdown report containing one table per slave, showing:
 *  - The reference value from the TOML file.
 *  - The current live value read from the drive.
 *  - The difference (delta) between the two.
 *
 * This allows an operator to quickly verify whether the encoder state has drifted since the last
 * calibration, without re-running the full homing procedure.
 *
 * @note Only SDO communication is used; the ring stays in SAFE-OP/PRE-OP throughout.
 * @note SOEM uses 1-based slave indices; the TOML file must follow the same convention
 *       (@c slave_1, @c slave_2, …).
 */
class CheckEncoderCalibration
{
public:
    /**
     * @brief Default constructor.
     *
     * Allocates the internal implementation object. No EtherCAT communication is started.
     */
    CheckEncoderCalibration();

    /**
     * @brief Destructor.
     *
     * Releases all resources owned by the implementation. If the EtherCAT master was
     * initialised, it is cleanly shut down.
     */
    ~CheckEncoderCalibration();

    /// @cond — non-copyable
    CheckEncoderCalibration(const CheckEncoderCalibration&) = delete;
    CheckEncoderCalibration& operator=(const CheckEncoderCalibration&) = delete;
    /// @endcond

    /**
     * @brief Run the full encoder check and produce a Markdown report.
     *
     * Reads configuration from the provided ResourceFinder, discovers all EtherCAT slaves on
     * the specified network interface, reads each slave's encoder data, compares it against
     * the reference TOML file, and writes a Markdown report to disk.
     *
     * @param rf ResourceFinder populated with the parameters listed below.
     * @return @c true if the check completed successfully and the report was written;
     *         @c false on any error (initialisation failure, TOML parse error, SDO read failure,
     *         file I/O error).
     *
     * @note Expected keys in the ResourceFinder:
     * | Key              | Type        | Description                                           | Default                    |
     * |:----------------:|:-----------:|-------------------------------------------------------|:--------------------------:|
     * | ifname           | string      | Network interface name (e.g. @c "eth0")               | @c "eth0"                  |
     * | toml-input       | string      | Path of the reference TOML file (from StoreHome37)    | *(required)*               |
     * | report-output    | string      | Path of the Markdown report to write                  | @c "encoder_calibration_check_YYYY_MM_DD_HH_MM_SS.md" |
     */
    bool run(yarp::os::ResourceFinder& rf);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl; ///< Opaque implementation (PIMPL idiom).
};

} // namespace CiA402

#endif // CHECK_ENCODER_CALIBRATION_H
