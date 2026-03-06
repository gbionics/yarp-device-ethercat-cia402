// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include <toml++/toml.h>

#include <CiA402/EthercatManager.h>
#include <CiA402/LogComponent.h>

#include <StoreHomePosition/StoreHomePosition.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

using namespace CiA402;
using namespace std::chrono_literals;

static std::tm getLocalTime(const std::time_t& t)
{
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    return tm;
}

// ---- CiA-402 / Synapticon indices ----
static constexpr uint16_t IDX_CONTROLWORD = 0x6040; // uint16
static constexpr uint16_t IDX_STATUSWORD = 0x6041; // uint16
static constexpr uint16_t IDX_OPMODE = 0x6060; // int8  : 6 = Homing
static constexpr uint16_t IDX_POSITION_ACT = 0x6064; // int32 : for logging
static constexpr uint16_t IDX_HOMING_METHOD = 0x6098; // int8  : 35/37
static constexpr uint16_t IDX_HOME_OFFSET = 0x607C; // int32
static constexpr uint16_t IDX_STORE_PARAMS = 0x1010; // uint32: :01 = 'evas'

static constexpr uint16_t IDX_HOME_VENDOR = 0x2005; // Synapticon: :01 Home, :02 Restore-on-load

// ---- Encoder SDO indices ----
static constexpr uint16_t IDX_ENC1_CONFIG = 0x2110; // :03 = resolution (counts/rev)
static constexpr uint16_t IDX_ENC1_DATA   = 0x2111; // :02 = position
static constexpr uint16_t IDX_ENC2_CONFIG = 0x2112; // :03 = resolution (counts/rev)
static constexpr uint16_t IDX_ENC2_DATA   = 0x2113; // :02 = position

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
             int32_t hoff,
             int timeoutMs,
             bool restore,
             const std::string& tomlPath)
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

        bool allOk = true;
        for (int s : slaves)
        {
            allOk = allOk && this->homeAndPersistSingle(s, hm, hoff, timeoutMs, restore);
        }

        // After calibration, collect encoder data and write TOML
        if (allOk)
        {
            allOk = this->writeEncoderToml(slaves, tomlPath);
        }

        return allOk;
    }

private:
    bool writeEncoderToml(const std::vector<int>& slaves, const std::string& tomlPath)
    {
        toml::table root;

        for (int s : slaves)
        {
            const std::string slaveName = "slave_" + std::to_string(s);
            toml::table slaveTable;

            // Read encoder resolutions (counts per revolution)
            uint32_t enc1Res = 0;
            uint32_t enc2Res = 0;
            (void)mgr.readSDO<uint32_t>(s, IDX_ENC1_CONFIG, 0x03, enc1Res);
            (void)mgr.readSDO<uint32_t>(s, IDX_ENC2_CONFIG, 0x03, enc2Res);

            // Read encoder 1 feedback (0x2111)
            uint32_t enc1RawPos = 0;   // 0x2111:01 — raw position (UDINT)
            int32_t  enc1AdjPos = 0;   // 0x2111:02 — adjusted position (DINT)
            (void)mgr.readSDO<uint32_t>(s, IDX_ENC1_DATA, 0x01, enc1RawPos);
            (void)mgr.readSDO<int32_t>(s, IDX_ENC1_DATA, 0x02, enc1AdjPos);

            // Read encoder 2 feedback (0x2113)
            uint32_t enc2RawPos = 0;   // 0x2113:01 — raw position (UDINT)
            int32_t  enc2AdjPos = 0;   // 0x2113:02 — adjusted position (DINT)
            (void)mgr.readSDO<uint32_t>(s, IDX_ENC2_DATA, 0x01, enc2RawPos);
            (void)mgr.readSDO<int32_t>(s, IDX_ENC2_DATA, 0x02, enc2AdjPos);

            // Compute degrees from adjusted position: degrees = adj * (360.0 / resolution)
            const double enc1ResInv = enc1Res ? (1.0 / static_cast<double>(enc1Res)) : 0.0;
            const double enc2ResInv = enc2Res ? (1.0 / static_cast<double>(enc2Res)) : 0.0;
            const double enc1AdjDeg = static_cast<double>(enc1AdjPos) * enc1ResInv * 360.0;
            const double enc2AdjDeg = static_cast<double>(enc2AdjPos) * enc2ResInv * 360.0;

            // Compute degrees from raw position
            const double enc1RawDeg = static_cast<double>(enc1RawPos) * enc1ResInv * 360.0;
            const double enc2RawDeg = static_cast<double>(enc2RawPos) * enc2ResInv * 360.0;

            // Store encoder 1 data
            toml::table enc1Table;
            enc1Table.insert("raw_position", static_cast<int64_t>(enc1RawPos));
            enc1Table.insert("raw_position_degrees", enc1RawDeg);
            enc1Table.insert("adjusted_position", static_cast<int64_t>(enc1AdjPos));
            enc1Table.insert("adjusted_position_degrees", enc1AdjDeg);
            enc1Table.insert("counts_per_revolution", static_cast<int64_t>(enc1Res));
            enc1Table.insert("raw_to_degrees_factor", enc1Res ? (360.0 / static_cast<double>(enc1Res)) : 0.0);
            slaveTable.insert("encoder1", enc1Table);

            // Store encoder 2 data
            toml::table enc2Table;
            enc2Table.insert("raw_position", static_cast<int64_t>(enc2RawPos));
            enc2Table.insert("raw_position_degrees", enc2RawDeg);
            enc2Table.insert("adjusted_position", static_cast<int64_t>(enc2AdjPos));
            enc2Table.insert("adjusted_position_degrees", enc2AdjDeg);
            enc2Table.insert("counts_per_revolution", static_cast<int64_t>(enc2Res));
            enc2Table.insert("raw_to_degrees_factor", enc2Res ? (360.0 / static_cast<double>(enc2Res)) : 0.0);
            slaveTable.insert("encoder2", enc2Table);

            // Also store device name
            slaveTable.insert("name", mgr.getName(s));

            root.insert(slaveName, slaveTable);

            yCInfo(CIA402,
                   "s%02d: enc1 raw=%u adj=%d adj_deg=%.6f (res=%u), "
                   "enc2 raw=%u adj=%d adj_deg=%.6f (res=%u)",
                   s,
                   enc1RawPos,
                   enc1AdjPos,
                   enc1AdjDeg,
                   enc1Res,
                   enc2RawPos,
                   enc2AdjPos,
                   enc2AdjDeg,
                   enc2Res);
        }

        // Write TOML to file
        std::ofstream ofs(tomlPath);
        if (!ofs.is_open())
        {
            yCError(CIA402, "StoreHome37: cannot open %s for writing", tomlPath.c_str());
            return false;
        }
        ofs << root;
        ofs.close();
        yCInfo(CIA402, "StoreHome37: encoder data written to %s", tomlPath.c_str());
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

    int32_t homeOffset = 0;
    if (rf.check("home-offset"))
    {
        homeOffset = rf.find("home-offset").asInt32();
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

    yCInfo(CIA402,
           "StoreHome37: ifname=%s method=%d home-offset=%d timeout-ms=%d restore-on-boot=%s",
           ifname.c_str(),
           int(method),
           homeOffset,
           timeoutMs,
           restoreOnBoot ? "true" : "false");
    std::string tomlPath;
    if (rf.check("toml-output"))
    {
        tomlPath = rf.find("toml-output").asString();
    } else
    {
        // Generate default filename with current date and time
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        const std::tm tm = getLocalTime(t);
        std::ostringstream oss;
        oss << "joint_calibration_"
            << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S")
            << ".toml";
        tomlPath = oss.str();
    }

    yCInfo(CIA402, "Do you want to proceed? (press ENTER to continue)");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    return m_impl->run(ifname, method, homeOffset, timeoutMs, restoreOnBoot, tomlPath);
}
