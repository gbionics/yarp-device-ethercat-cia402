// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>

#include <CiA402/EthercatManager.h>
#include <CiA402/LogComponent.h>

#include <StoreHomePosition/StoreHomePosition.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>

using namespace CiA402;
using namespace std::chrono_literals;

// ---- CiA-402 / Synapticon indices ----
static constexpr uint16_t IDX_CONTROLWORD = 0x6040; // uint16
static constexpr uint16_t IDX_STATUSWORD = 0x6041; // uint16
static constexpr uint16_t IDX_OPMODE = 0x6060; // int8  : 6 = Homing
static constexpr uint16_t IDX_POSITION_ACT = 0x6064; // int32 : for logging
static constexpr uint16_t IDX_HOMING_METHOD = 0x6098; // int8  : 35/37
static constexpr uint16_t IDX_HOME_OFFSET = 0x607C; // int32
static constexpr uint16_t IDX_STORE_PARAMS = 0x1010; // uint32: :01 = 'evas'

static constexpr uint16_t IDX_HOME_VENDOR = 0x2005; // Synapticon: :01 Home, :02 Restore-on-load

// ---- Statusword helpers ----
static inline bool swHomingAttained(uint16_t sw)
{
    return (sw & (1u << 12)) != 0;
}
static inline bool swHomingError(uint16_t sw)
{
    return (sw & (1u << 13)) != 0;
}

class StoreHome37::Impl
{
public:
    bool run(const std::string& ifname,
             int8_t hm,
             const std::vector<double>& homeOffsetsDeg,
             const std::vector<std::string>& enc1MountStr,
             const std::vector<std::string>& enc2MountStr,
             int timeoutMs,
             bool restore)
    {
        // Initialize EtherCAT master
        yCInfo(CIA402, "StoreHome37: initializing EtherCAT on %s", ifname.c_str());
        const auto rc = mgr.init(ifname);
        if (rc != EthercatManager::Error::NoError)
        {
            yCError(CIA402, "StoreHome37: init failed on %s (rc=%d)", ifname.c_str(), int(rc));
            return false;
        }

        // Discover slaves (1-based indices in SOEM)
        std::vector<int> slaves;
        for (int s = 1;; ++s)
        {
            auto name = mgr.getName(s);
            if (name.empty())
                break;
            yCInfo(CIA402, "StoreHome3537: found slave %d: %s", s, name.c_str());
            slaves.push_back(s);
        }
        if (slaves.empty())
        {
            yCError(CIA402, "StoreHome3537: no slaves found");
            return false;
        }

        const size_t numSlaves = slaves.size();

        std::vector<double> offsetsDegExpanded;
        if (!expandOffsets(homeOffsetsDeg, numSlaves, offsetsDegExpanded))
        {
            return false;
        }

        std::vector<Mount> enc1Mounts;
        if (!expandMounts(enc1MountStr, numSlaves, Mount::Motor, "enc1-mount", enc1Mounts))
        {
            return false;
        }
        std::vector<Mount> enc2Mounts;
        if (!expandMounts(enc2MountStr, numSlaves, Mount::None, "enc2-mount", enc2Mounts))
        {
            return false;
        }

        bool allOk = true;
        for (size_t i = 0; i < numSlaves; ++i)
        {
            const int s = slaves[i];
            int32_t hoffCounts = 0;
            if (!homeOffsetDegToCounts(s, offsetsDegExpanded[i], enc1Mounts[i], enc2Mounts[i],
                                       hoffCounts))
            {
                return false;
            }
            allOk &= homeAndPersistSingle(s, hm, hoffCounts, timeoutMs, restore);
        }
        return allOk;
    }

private:
    enum class Mount
    {
        None,
        Motor,
        Joint
    };

    enum class SensorSrc
    {
        Unknown,
        Enc1,
        Enc2
    };

    static const char* mountToString(Mount m)
    {
        switch (m)
        {
        case Mount::Motor:
            return "motor";
        case Mount::Joint:
            return "joint";
        default:
            return "none";
        }
    }

    static const char* srcToString(SensorSrc s)
    {
        switch (s)
        {
        case SensorSrc::Enc1:
            return "enc1";
        case SensorSrc::Enc2:
            return "enc2";
        default:
            return "unknown";
        }
    }

    static Mount parseMount(const std::string& s)
    {
        if (s == "motor")
            return Mount::Motor;
        if (s == "joint")
            return Mount::Joint;
        if (s == "none")
            return Mount::None;
        return Mount::None;
    }

    static bool expandOffsets(const std::vector<double>& src,
                              size_t count,
                              std::vector<double>& out)
    {
        out.clear();
        if (count == 0)
            return true;

        if (src.empty())
        {
            out.assign(count, 0.0);
            return true;
        }

        if (src.size() == 1)
        {
            out.assign(count, src.front());
            return true;
        }

        if (src.size() != count)
        {
            yCError(CIA402,
                    "StoreHome37: home-offset list size %zu does not match number of slaves %zu",
                    src.size(),
                    count);
            return false;
        }

        out = src;
        return true;
    }

    static bool expandMounts(const std::vector<std::string>& src,
                             size_t count,
                             Mount defaultMount,
                             const char* key,
                             std::vector<Mount>& out)
    {
        out.clear();
        if (count == 0)
            return true;

        if (src.empty())
        {
            out.assign(count, defaultMount);
            return true;
        }

        if (src.size() == 1)
        {
            out.assign(count, parseMount(src.front()));
            return true;
        }

        if (src.size() != count)
        {
            yCError(CIA402,
                    "StoreHome37: %s list size %zu does not match number of slaves %zu",
                    key,
                    src.size(),
                    count);
            return false;
        }

        out.reserve(src.size());
        for (const auto& item : src)
            out.push_back(parseMount(item));
        return true;
    }

    static int32_t clampCounts(long long v)
    {
        if (v > std::numeric_limits<int32_t>::max())
            return std::numeric_limits<int32_t>::max();
        if (v < std::numeric_limits<int32_t>::min())
            return std::numeric_limits<int32_t>::min();
        return static_cast<int32_t>(v);
    }

    SensorSrc readPosLoopSrc(int s)
    {
        uint8_t src = 0;
        auto e = mgr.readSDO<uint8_t>(s, 0x2012, 0x09, src);
        if (e != EthercatManager::Error::NoError)
        {
            yCWarning(CIA402, "s%02d: cannot read position loop source (0x2012:09)", s);
            return SensorSrc::Unknown;
        }
        return (src == 1)   ? SensorSrc::Enc1
               : (src == 2) ? SensorSrc::Enc2
                            : SensorSrc::Unknown;
    }

    bool readEncoderResolutions(int s, uint32_t& enc1Res, uint32_t& enc2Res)
    {
        enc1Res = 0;
        enc2Res = 0;

        auto e1 = mgr.readSDO<uint32_t>(s, 0x2110, 0x03, enc1Res);
        if (e1 != EthercatManager::Error::NoError)
            enc1Res = 0;

        auto e2 = mgr.readSDO<uint32_t>(s, 0x2112, 0x03, enc2Res);
        if (e2 != EthercatManager::Error::NoError)
            enc2Res = 0;

        return (enc1Res != 0 || enc2Res != 0);
    }

    double readGearRatio(int s)
    {
        uint32_t num = 1U;
        uint32_t den = 1U;

        if (mgr.readSDO<uint32_t>(s, 0x6091, 0x01, num) != EthercatManager::Error::NoError)
        {
            num = 1U;
        }
        if (mgr.readSDO<uint32_t>(s, 0x6091, 0x02, den) != EthercatManager::Error::NoError
            || den == 0U)
        {
            den = 1U;
        }

        return static_cast<double>(num) / static_cast<double>(den);
    }

    bool homeOffsetDegToCounts(int s,
                               double offsetDeg,
                               Mount enc1Mount,
                               Mount enc2Mount,
                               int32_t& counts)
    {
        if (offsetDeg == 0.0)
        {
            counts = 0;
            return true;
        }

        const SensorSrc src = readPosLoopSrc(s);
        uint32_t enc1Res = 0;
        uint32_t enc2Res = 0;
        (void)readEncoderResolutions(s, enc1Res, enc2Res);

        const double gearRatio = readGearRatio(s);

        uint32_t res = 0;
        Mount mount = Mount::None;

        switch (src)
        {
        case SensorSrc::Enc1:
            res = enc1Res;
            mount = enc1Mount;
            break;
        case SensorSrc::Enc2:
            res = enc2Res;
            mount = enc2Mount;
            break;
        default:
            if (enc1Res != 0 && enc1Mount != Mount::None)
            {
                res = enc1Res;
                mount = enc1Mount;
            } else if (enc2Res != 0 && enc2Mount != Mount::None)
            {
                res = enc2Res;
                mount = enc2Mount;
            }
            break;
        }

        if (res == 0 || mount == Mount::None)
        {
            yCError(CIA402,
                    "s%02d: cannot convert home-offset (deg=%.6f). "
                    "Check encoder resolution and mount configuration.",
                    s,
                    offsetDeg);
            return false;
        }

        double shaftDeg = offsetDeg;
        if (mount == Mount::Motor)
            shaftDeg = offsetDeg * gearRatio;

        const double cnt = (shaftDeg / 360.0) * static_cast<double>(res);
        counts = clampCounts(llround(cnt));

        yCInfo(CIA402,
               "s%02d: home-offset %.6f deg -> %d counts (loop=%s mount=%s res=%u gear=%.6f)",
               s,
               offsetDeg,
               counts,
               srcToString(src),
               mountToString(mount),
               res,
               gearRatio);

        return true;
    }

    bool homeAndPersistSingle(int s, int8_t hm, int32_t hoff, int timeoutMs, bool restore)
    {
        // -- 1) OpMode = Homing (6)
        {
            const int8_t op = 6;
            if (mgr.writeSDO<int8_t>(s, IDX_OPMODE, 0x00, op) != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: write 0x6060=6 failed", s);
                return false;
            }
        }

        // -- 2) 0x6098 = 37 (or 35)
        if (hm != 35 && hm != 37)
        {
            yCWarning(CIA402, "s%02d: invalid homing method %d, using 37", s, int(hm));
            hm = 37;
        }
        if (mgr.writeSDO<int8_t>(s, IDX_HOMING_METHOD, 0x00, hm) != EthercatManager::Error::NoError)
        {
            yCError(CIA402, "s%02d: write 0x6098=%d failed", s, int(hm));
            return false;
        }

        // -- 3) Optional extra home offset
        if (hoff != 0)
        {
            if (mgr.writeSDO<int32_t>(s, IDX_HOME_OFFSET, 0x00, hoff)
                != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: write 0x607C=%d failed", s, hoff);
                return false;
            }
        }

        // -- 4) Start homing: toggle Controlword bit4 (mode-specific "start")
        {
            uint16_t cw = 0;
            (void)mgr.readSDO<uint16_t>(s, IDX_CONTROLWORD, 0x00, cw); // best-effort read
            const uint16_t cwLow = static_cast<uint16_t>(cw & ~(1u << 4));
            const uint16_t cwHigh = static_cast<uint16_t>(cwLow | (1u << 4));
            if (mgr.writeSDO<uint16_t>(s, IDX_CONTROLWORD, 0x00, cwLow)
                    != EthercatManager::Error::NoError
                || mgr.writeSDO<uint16_t>(s, IDX_CONTROLWORD, 0x00, cwHigh)
                       != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: toggle 0x6040 bit4 failed", s);
                return false;
            }
        }

        // -- 5) Poll 0x6041: bit12 attained / bit13 error
        const auto t0 = std::chrono::steady_clock::now();
        while (true)
        {
            uint16_t sw = 0;
            if (mgr.readSDO<uint16_t>(s, IDX_STATUSWORD, 0x00, sw)
                != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: read 0x6041 failed", s);
                return false;
            }
            if (swHomingError(sw))
            {
                yCError(CIA402, "s%02d: homing error (0x6041=0x%04X)", s, sw);
                return false;
            }
            if (swHomingAttained(sw))
            {
                int32_t pos = 0;
                (void)mgr.readSDO<int32_t>(s, IDX_POSITION_ACT, 0x00, pos);
                yCInfo(CIA402, "s%02d: homing attained (pos=%d, sw=0x%04X)", s, pos, sw);
                break;
            }
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0)
                    .count()
                > timeoutMs)
            {
                yCError(CIA402, "s%02d: timeout waiting homing attained", s);
                return false;
            }
            std::this_thread::sleep_for(10ms);
        }

        // -- 6) (Optional) read vendor Home Position 0x2005:01 for logging
        int32_t homeVal = 0;
        (void)mgr.readSDO<int32_t>(s, IDX_HOME_VENDOR, 0x01, homeVal);
        yCInfo(CIA402, "s%02d: vendor Home (0x2005:01) = %d", s, homeVal);

        // -- 7) Set restore-on-startup flag per request (0x2005:02)
        {
            const uint8_t flag = restore ? uint8_t{1} : uint8_t{0};
            if (mgr.writeSDO<uint8_t>(s, IDX_HOME_VENDOR, 0x02, flag)
                != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: write 0x2005:02=%u failed", s, flag);
                return false;
            }
        }

        // -- 8) Save to flash: 0x1010:01 = 'evas' (0x65766173)
        {
            constexpr uint32_t EVAS = 0x65766173u;
            if (mgr.writeSDO<uint32_t>(s, IDX_STORE_PARAMS, 0x01, EVAS)
                != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: save 0x1010:01='evas' failed", s);
                return false;
            }
            yCInfo(CIA402,
                   "s%02d: configuration saved (home persisted, restoreOnStartup=%s)",
                   s,
                   restore ? "true" : "false");
        }

        return true;
    }

    EthercatManager mgr;
};

StoreHome37::StoreHome37()
{
    m_impl = std::make_unique<Impl>();
}

StoreHome37::~StoreHome37() = default;

bool StoreHome37::run(yarp::os::ResourceFinder& rf)
{
    const std::string ifname = rf.check("ifname") ? rf.find("ifname").asString()
                                                  : std::string("eth0");
    int methodTmp = 37;
    if (rf.check("method"))
    {
        methodTmp = rf.find("method").asInt32();
    }
    const int8_t method = static_cast<int8_t>(methodTmp);

    std::vector<double> homeOffsetsDeg;
    if (rf.check("home-offset"))
    {
        const auto v = rf.find("home-offset");
        if (v.isList())
        {
            auto* list = v.asList();
            if (list)
            {
                homeOffsetsDeg.reserve(static_cast<size_t>(list->size()));
                for (size_t i = 0; i < static_cast<size_t>(list->size()); ++i)
                {
                    homeOffsetsDeg.push_back(list->get(static_cast<int>(i)).asFloat64());
                }
            }
        } else if (v.isFloat64() || v.isInt32())
        {
            homeOffsetsDeg.push_back(v.asFloat64());
        } else
        {
            yCWarning(CIA402,
                      "StoreHome37: home-offset is not a list or numeric, using 0.0 deg");
        }
    }

    std::vector<std::string> enc1Mounts;
    if (rf.check("enc1-mount"))
    {
        const auto v = rf.find("enc1-mount");
        if (v.isList())
        {
            auto* list = v.asList();
            if (list)
            {
                enc1Mounts.reserve(static_cast<size_t>(list->size()));
                for (size_t i = 0; i < static_cast<size_t>(list->size()); ++i)
                {
                    enc1Mounts.push_back(list->get(static_cast<int>(i)).asString());
                }
            }
        } else if (v.isString())
        {
            enc1Mounts.push_back(v.asString());
        }
    }

    std::vector<std::string> enc2Mounts;
    if (rf.check("enc2-mount"))
    {
        const auto v = rf.find("enc2-mount");
        if (v.isList())
        {
            auto* list = v.asList();
            if (list)
            {
                enc2Mounts.reserve(static_cast<size_t>(list->size()));
                for (size_t i = 0; i < static_cast<size_t>(list->size()); ++i)
                {
                    enc2Mounts.push_back(list->get(static_cast<int>(i)).asString());
                }
            }
        } else if (v.isString())
        {
            enc2Mounts.push_back(v.asString());
        }
    }

    int timeoutMs = 2000;
    if (rf.check("timeout-ms"))
    {
        timeoutMs = rf.find("timeout-ms").asInt32();
    }

    bool restoreOnBoot = true;
    if (rf.check("restore-on-boot"))
    {
        // Accept bool or int
        if (rf.find("restore-on-boot").isBool())
        {
            restoreOnBoot = rf.find("restore-on-boot").asBool();
        } else
        {
            restoreOnBoot = (rf.find("restore-on-boot").asInt32() != 0);
        }
    }

    auto listToString = [](const std::vector<double>& v) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < v.size(); ++i)
        {
            if (i > 0)
                oss << ", ";
            oss << v[i];
        }
        oss << "]";
        return oss.str();
    };

    yCInfo(CIA402,
           "StoreHome37: ifname=%s method=%d home-offset-deg=%s timeout-ms=%d restore-on-boot=%s",
           ifname.c_str(),
           int(method),
           listToString(homeOffsetsDeg).c_str(),
           timeoutMs,
           restoreOnBoot ? "true" : "false");
    yCInfo(CIA402, "Do you want to proceed? (press ENTER to continue)");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    return m_impl->run(ifname,
                       method,
                       homeOffsetsDeg,
                       enc1Mounts,
                       enc2Mounts,
                       timeoutMs,
                       restoreOnBoot);
}
