// SPDX-FileCopyrightText: Generative Bionics
// SPDX-License-Identifier: BSD-3-Clause

#ifndef YARP_DEV_CIA402_ENCODER_SDO_INDICES_H
#define YARP_DEV_CIA402_ENCODER_SDO_INDICES_H

#include <cstdint>

namespace CiA402
{

/// @name Encoder SDO indices
/// @{
static constexpr uint16_t IDX_ENC1_CONFIG = 0x2110; ///< :03 = resolution (counts/rev)
static constexpr uint16_t IDX_ENC1_DATA = 0x2111;   ///< :01 = raw position, :02 = adjusted position
static constexpr uint16_t IDX_ENC2_CONFIG = 0x2112;  ///< :03 = resolution (counts/rev)
static constexpr uint16_t IDX_ENC2_DATA = 0x2113;    ///< :01 = raw position, :02 = adjusted position
/// @}

/// @name Commutation and motor SDO indices
/// @{
static constexpr uint16_t IDX_COMMUTATION_OFFSET = 0x2001; ///< :00 = commutation angle offset (INT16, [0-4095] → [0-360] elec. deg)
static constexpr uint16_t IDX_MOTOR_SETTINGS = 0x2003;     ///< :01 = pole pairs (UINT8)
/// @}

} // namespace CiA402

#endif // YARP_DEV_CIA402_ENCODER_SDO_INDICES_H
