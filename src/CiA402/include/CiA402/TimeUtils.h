// SPDX-FileCopyrightText: Generative Bionics
// SPDX-License-Identifier: BSD-3-Clause

#ifndef YARP_DEV_CIA402_TIME_UTILS_H
#define YARP_DEV_CIA402_TIME_UTILS_H

#include <ctime>

namespace CiA402
{

inline std::tm getLocalTime(const std::time_t& t)
{
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    return tm;
}

} // namespace CiA402

#endif // YARP_DEV_CIA402_TIME_UTILS_H
