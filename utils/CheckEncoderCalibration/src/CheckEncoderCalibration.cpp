// SPDX-FileCopyrightText: Generative Bionics
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <toml++/toml.h>

#include <CiA402/EncoderSdoIndices.h>
#include <CiA402/EthercatManager.h>
#include <CiA402/LogComponent.h>
#include <CiA402/TimeUtils.h>

#include <CheckEncoderCalibration/CheckEncoderCalibration.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

using namespace CiA402;

// ---- Helper: format a double with fixed precision ----
static std::string fmtDouble(double v, int prec = 6)
{
    std::ostringstream o;
    o << std::fixed << std::setprecision(prec) << v;
    return o.str();
}

// ---- Helper: format an integer (with explicit sign for deltas) ----
static std::string fmtInt64(int64_t v)
{
    return std::to_string(v);
}

static std::string fmtDeltaInt64(int64_t v)
{
    if (v > 0)
        return "+" + std::to_string(v);
    return std::to_string(v); // negative sign is implicit
}

static std::string fmtDeltaDouble(double v, int prec = 6)
{
    std::ostringstream o;
    if (v > 0.0)
        o << "+";
    o << std::fixed << std::setprecision(prec) << v;
    return o.str();
}

// ---- Data structures for a single encoder channel ----
struct EncoderChannelData
{
    int64_t rawPosition = 0;
    double rawPositionDegrees = 0.0;
    int64_t adjustedPosition = 0;
    double adjustedPositionDegrees = 0.0;
    int64_t countsPerRevolution = 0;
    double rawToDegreesFactor = 0.0;
};

struct CommutationOffsetData
{
    int16_t currentOffset = 0;      // 0x2001:00 — current commutation angle offset [0-4095]
    uint8_t polePairs = 1;           // 0x2003:01 — number of motor pole pairs
    int16_t estimatedNewOffset = 0;  // estimated offset after encoder 1 drift
    double estimatedNewOffsetDeg = 0.0; // estimated offset in electrical degrees
    double currentOffsetDeg = 0.0;   // current offset in electrical degrees
    double enc1DeltaDeg = 0.0;       // encoder 1 mechanical delta in degrees
    double enc1DeltaElecDeg = 0.0;   // encoder 1 delta in electrical degrees
};

struct SlaveReport
{
    int slaveIndex = 0;
    std::string name;

    EncoderChannelData refEnc1;
    EncoderChannelData refEnc2;
    EncoderChannelData liveEnc1;
    EncoderChannelData liveEnc2;

    CommutationOffsetData commOffset;
};

// ---- Read one encoder channel from a TOML table ----
static bool readEncoderFromToml(const toml::table& tbl,
                                const std::string& key,
                                EncoderChannelData& out)
{
    const auto* encTbl = tbl[key].as_table();
    if (!encTbl)
        return false;

    auto warnMissing = [&](const char* field) {
        yCWarning(CIA402,
                  "CheckEncoderCalibration: missing key '%s' in [%s], using default value",
                  field,
                  key.c_str());
    };

    if (auto v = (*encTbl)["raw_position"].value<int64_t>())
        out.rawPosition = *v;
    else
        warnMissing("raw_position");
    if (auto v = (*encTbl)["raw_position_degrees"].value<double>())
        out.rawPositionDegrees = *v;
    else
        warnMissing("raw_position_degrees");
    if (auto v = (*encTbl)["adjusted_position"].value<int64_t>())
        out.adjustedPosition = *v;
    else
        warnMissing("adjusted_position");
    if (auto v = (*encTbl)["adjusted_position_degrees"].value<double>())
        out.adjustedPositionDegrees = *v;
    else
        warnMissing("adjusted_position_degrees");
    if (auto v = (*encTbl)["counts_per_revolution"].value<int64_t>())
        out.countsPerRevolution = *v;
    else
        warnMissing("counts_per_revolution");
    if (auto v = (*encTbl)["raw_to_degrees_factor"].value<double>())
        out.rawToDegreesFactor = *v;
    else
        warnMissing("raw_to_degrees_factor");

    return true;
}

// ---- Read live encoder data from a slave via SDO ----
static bool readEncoderFromSlave(EthercatManager& mgr,
                                 int slave,
                                 uint16_t idxConfig,
                                 uint16_t idxData,
                                 EncoderChannelData& out)
{
    uint32_t resolution = 0;
    uint32_t rawPos = 0;
    int32_t adjPos = 0;

    if (mgr.readSDO<uint32_t>(slave, idxConfig, 0x03, resolution)
        != EthercatManager::Error::NoError)
    {
        yCWarning(CIA402, "s%02d: failed to read resolution from 0x%04X:03", slave, idxConfig);
    }
    if (mgr.readSDO<uint32_t>(slave, idxData, 0x01, rawPos) != EthercatManager::Error::NoError)
    {
        yCWarning(CIA402, "s%02d: failed to read raw position from 0x%04X:01", slave, idxData);
    }
    if (mgr.readSDO<int32_t>(slave, idxData, 0x02, adjPos) != EthercatManager::Error::NoError)
    {
        yCWarning(CIA402, "s%02d: failed to read adjusted position from 0x%04X:02", slave, idxData);
    }

    out.countsPerRevolution = static_cast<int64_t>(resolution);
    out.rawPosition = static_cast<int64_t>(rawPos);
    out.adjustedPosition = static_cast<int64_t>(adjPos);

    const double resInv = resolution ? (1.0 / static_cast<double>(resolution)) : 0.0;
    out.rawPositionDegrees = static_cast<double>(rawPos) * resInv * 360.0;
    out.adjustedPositionDegrees = static_cast<double>(adjPos) * resInv * 360.0;
    out.rawToDegreesFactor = resolution ? (360.0 / static_cast<double>(resolution)) : 0.0;

    return true;
}

// ---- Write a single encoder comparison table in Markdown ----
static void writeEncoderTable(std::ostream& os,
                              const std::string& title,
                              const EncoderChannelData& ref,
                              const EncoderChannelData& live)
{
    os << "#### " << title << "\n\n";

    os << "| Metric | Reference (TOML) | Current (Live) | Delta |\n";
    os << "|:-------|-----------------:|---------------:|------:|\n";

    // Raw position (counts)
    const int64_t dRaw = live.rawPosition - ref.rawPosition;
    os << "| Raw position (counts) | "
       << fmtInt64(ref.rawPosition) << " | "
       << fmtInt64(live.rawPosition) << " | "
       << fmtDeltaInt64(dRaw) << " |\n";

    // Raw position (degrees)
    const double dRawDeg = live.rawPositionDegrees - ref.rawPositionDegrees;
    os << "| Raw position (deg) | "
       << fmtDouble(ref.rawPositionDegrees) << " | "
       << fmtDouble(live.rawPositionDegrees) << " | "
       << fmtDeltaDouble(dRawDeg) << " |\n";

    // Adjusted position (counts)
    const int64_t dAdj = live.adjustedPosition - ref.adjustedPosition;
    os << "| Adjusted position (counts) | "
       << fmtInt64(ref.adjustedPosition) << " | "
       << fmtInt64(live.adjustedPosition) << " | "
       << fmtDeltaInt64(dAdj) << " |\n";

    // Adjusted position (degrees)
    const double dAdjDeg = live.adjustedPositionDegrees - ref.adjustedPositionDegrees;
    os << "| Adjusted position (deg) | "
       << fmtDouble(ref.adjustedPositionDegrees) << " | "
       << fmtDouble(live.adjustedPositionDegrees) << " | "
       << fmtDeltaDouble(dAdjDeg) << " |\n";

    // Counts per revolution
    const int64_t dRes = live.countsPerRevolution - ref.countsPerRevolution;
    os << "| Counts per revolution | "
       << fmtInt64(ref.countsPerRevolution) << " | "
       << fmtInt64(live.countsPerRevolution) << " | "
       << fmtDeltaInt64(dRes) << " |\n";

    // Raw-to-degrees factor
    const double dFactor = live.rawToDegreesFactor - ref.rawToDegreesFactor;
    os << "| Raw-to-degrees factor | "
       << fmtDouble(ref.rawToDegreesFactor) << " | "
       << fmtDouble(live.rawToDegreesFactor) << " | "
       << fmtDeltaDouble(dFactor) << " |\n";

    os << "\n";
}

class CheckEncoderCalibration::Impl
{
public:
    bool run(const std::string& ifname, const std::string& tomlPath, const std::string& reportPath)
    {
        // ---- 1) Check that the TOML file exists and parse it ----
        {
            std::ifstream test(tomlPath);
            if (!test.is_open())
            {
                yCError(CIA402,
                        "CheckEncoderCalibration: file not found: %s",
                        tomlPath.c_str());
                return false;
            }
        }

        toml::table refRoot;
        try
        {
            refRoot = toml::parse_file(tomlPath);
        } catch (const toml::parse_error& err)
        {
            yCError(CIA402,
                    "CheckEncoderCalibration: failed to parse %s: %s",
                    tomlPath.c_str(),
                    err.what());
            return false;
        }
        yCInfo(CIA402, "CheckEncoderCalibration: loaded reference TOML from %s", tomlPath.c_str());

        // ---- 2) Initialize EtherCAT master ----
        yCInfo(CIA402, "CheckEncoderCalibration: initializing EtherCAT on %s", ifname.c_str());
        const auto rc = mgr.init(ifname);
        if (rc != EthercatManager::Error::NoError)
        {
            yCError(CIA402,
                    "CheckEncoderCalibration: init failed on %s (rc=%d)",
                    ifname.c_str(),
                    int(rc));
            return false;
        }

        // ---- 3) Discover slaves ----
        std::vector<int> slaves;
        for (int s = 1;; ++s)
        {
            auto name = mgr.getName(s);
            if (name.empty())
                break;
            yCInfo(CIA402, "CheckEncoderCalibration: found slave %d: %s", s, name.c_str());
            slaves.push_back(s);
        }
        if (slaves.empty())
        {
            yCError(CIA402, "CheckEncoderCalibration: no slaves found");
            return false;
        }

        // ---- 4) For each slave, read reference + live data ----
        std::vector<SlaveReport> reports;
        bool allOk = true;

        for (int s : slaves)
        {
            const std::string slaveKey = "slave_" + std::to_string(s);
            const auto* slaveTbl = refRoot[slaveKey].as_table();

            if (!slaveTbl)
            {
                yCWarning(CIA402,
                          "CheckEncoderCalibration: no entry '%s' in TOML, skipping slave %d",
                          slaveKey.c_str(),
                          s);
                continue;
            }

            SlaveReport rpt;
            rpt.slaveIndex = s;

            // Reference name from TOML
            if (auto v = (*slaveTbl)["name"].value<std::string>())
                rpt.name = *v;
            else
                rpt.name = mgr.getName(s);

            // Read reference encoder data from TOML
            if (!readEncoderFromToml(*slaveTbl, "encoder1", rpt.refEnc1))
            {
                yCWarning(CIA402,
                          "CheckEncoderCalibration: missing encoder1 in TOML for %s",
                          slaveKey.c_str());
            }
            if (!readEncoderFromToml(*slaveTbl, "encoder2", rpt.refEnc2))
            {
                yCWarning(CIA402,
                          "CheckEncoderCalibration: missing encoder2 in TOML for %s",
                          slaveKey.c_str());
            }

            // Read live encoder data from the drive
            readEncoderFromSlave(mgr, s, IDX_ENC1_CONFIG, IDX_ENC1_DATA, rpt.liveEnc1);
            readEncoderFromSlave(mgr, s, IDX_ENC2_CONFIG, IDX_ENC2_DATA, rpt.liveEnc2);

            // Read commutation offset (0x2001:00) and pole pairs (0x2003:01)
            {
                int16_t commOffset = 0;
                uint8_t polePairs = 1;

                if (mgr.readSDO<int16_t>(s, IDX_COMMUTATION_OFFSET, 0x00, commOffset)
                    != EthercatManager::Error::NoError)
                {
                    yCWarning(CIA402,
                              "s%02d: failed to read commutation offset (0x2001:00)",
                              s);
                }
                if (mgr.readSDO<uint8_t>(s, IDX_MOTOR_SETTINGS, 0x01, polePairs)
                    != EthercatManager::Error::NoError)
                {
                    yCWarning(CIA402,
                              "s%02d: failed to read pole pairs (0x2003:01)",
                              s);
                }
                if (polePairs == 0)
                    polePairs = 1;

                rpt.commOffset.currentOffset = commOffset;
                rpt.commOffset.polePairs = polePairs;
                rpt.commOffset.currentOffsetDeg
                    = static_cast<double>(commOffset) * 360.0 / 4096.0;

                // Estimate new commutation offset from encoder 1 drift
                // 0x2001 range [0-4095] maps to [0-360] electrical degrees
                const int64_t enc1Delta
                    = rpt.liveEnc1.rawPosition - rpt.refEnc1.rawPosition;
                const double enc1Res
                    = static_cast<double>(rpt.liveEnc1.countsPerRevolution);

                if (enc1Res > 0.0)
                {
                    const double deltaMechDeg
                        = static_cast<double>(enc1Delta) * 360.0 / enc1Res;
                    const double deltaElecDeg
                        = deltaMechDeg * static_cast<double>(polePairs);
                    const double deltaOffsetUnits = deltaElecDeg * 4096.0 / 360.0;

                    // Wrap into [0, 4096) range
                    int32_t newRaw
                        = static_cast<int32_t>(commOffset)
                          + static_cast<int32_t>(std::round(deltaOffsetUnits));
                    newRaw = ((newRaw % 4096) + 4096) % 4096;

                    rpt.commOffset.estimatedNewOffset = static_cast<int16_t>(newRaw);
                    rpt.commOffset.estimatedNewOffsetDeg
                        = static_cast<double>(newRaw) * 360.0 / 4096.0;
                    rpt.commOffset.enc1DeltaDeg = deltaMechDeg;
                    rpt.commOffset.enc1DeltaElecDeg = deltaElecDeg;
                }
            }

            reports.push_back(rpt);
        }

        // ---- 5) Generate Markdown report ----
        allOk = this->writeMarkdownReport(reports, reportPath, tomlPath);

        return allOk;
    }

private:
    bool writeMarkdownReport(const std::vector<SlaveReport>& reports,
                             const std::string& reportPath,
                             const std::string& tomlPath)
    {
        std::ofstream ofs(reportPath);
        if (!ofs.is_open())
        {
            yCError(CIA402,
                    "CheckEncoderCalibration: cannot open %s for writing",
                    reportPath.c_str());
            return false;
        }

        // ---- Header ----
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        const std::tm tm = getLocalTime(t);
        std::ostringstream tsStr;
        tsStr << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

        ofs << "# Encoder Calibration Check Report\n\n";

        ofs << "| | |\n";
        ofs << "|:--|:--|\n";
        ofs << "| **Date** | " << tsStr.str() << " |\n";
        ofs << "| **Reference TOML** | `" << tomlPath << "` |\n";
        ofs << "| **Slaves checked** | " << reports.size() << " |\n";
        ofs << "\n---\n\n";

        // ---- Summary table ----
        ofs << "## Summary\n\n";
        ofs << "| Slave | Name | Enc1 Raw &Delta; (deg) | Enc2 Raw &Delta; (deg) | Current Offset (0x2001) | Estimated New Offset |\n";
        ofs << "|:-----:|:-----|----------------------:|-----------------------:|------------------------:|---------------------:|\n";

        for (const auto& rpt : reports)
        {
            const double d1
                = rpt.liveEnc1.rawPositionDegrees - rpt.refEnc1.rawPositionDegrees;
            const double d2
                = rpt.liveEnc2.rawPositionDegrees - rpt.refEnc2.rawPositionDegrees;
            ofs << "| " << rpt.slaveIndex << " | " << rpt.name << " | " << fmtDeltaDouble(d1)
                << " | " << fmtDeltaDouble(d2)
                << " | " << rpt.commOffset.currentOffset
                << " | " << rpt.commOffset.estimatedNewOffset << " |\n";
        }
        ofs << "\n---\n\n";

        // ---- Per-slave detailed tables ----
        for (const auto& rpt : reports)
        {
            ofs << "### Slave " << rpt.slaveIndex << " — " << rpt.name << "\n\n";

            writeEncoderTable(ofs, "Encoder 1 (0x2111)", rpt.refEnc1, rpt.liveEnc1);
            writeEncoderTable(ofs, "Encoder 2 (0x2113)", rpt.refEnc2, rpt.liveEnc2);

            // Commutation offset estimation
            ofs << "#### Commutation Offset Estimation (0x2001)\n\n";
            ofs << "| Metric | Value |\n";
            ofs << "|:-------|------:|\n";
            ofs << "| Pole pairs (0x2003:01) | "
                << static_cast<int>(rpt.commOffset.polePairs) << " |\n";
            ofs << "| Current offset (0x2001) | "
                << rpt.commOffset.currentOffset << " |\n";
            ofs << "| Current offset (elec. deg) | "
                << fmtDouble(rpt.commOffset.currentOffsetDeg) << " |\n";
            ofs << "| Enc1 raw &Delta; (mech. deg) | "
                << fmtDeltaDouble(rpt.commOffset.enc1DeltaDeg) << " |\n";
            ofs << "| Enc1 raw &Delta; (elec. deg) | "
                << fmtDeltaDouble(rpt.commOffset.enc1DeltaElecDeg) << " |\n";
            ofs << "| **Estimated new offset** | **"
                << rpt.commOffset.estimatedNewOffset << "** |\n";
            ofs << "| **Estimated new offset (elec. deg)** | **"
                << fmtDouble(rpt.commOffset.estimatedNewOffsetDeg) << "** |\n";

            const int16_t offsetDelta
                = rpt.commOffset.estimatedNewOffset - rpt.commOffset.currentOffset;
            const double offsetDeltaDeg
                = rpt.commOffset.estimatedNewOffsetDeg - rpt.commOffset.currentOffsetDeg;
            ofs << "| **&Delta; (estimated &minus; current)** | **"
                << fmtDeltaInt64(static_cast<int64_t>(offsetDelta)) << "** |\n";
            ofs << "| **&Delta; (elec. deg)** | **"
                << fmtDeltaDouble(offsetDeltaDeg) << "** |\n";
            ofs << "\n";

            ofs << "---\n\n";
        }

        ofs << "*Report generated by `yarp-cia402-check-encoder-calibration`*\n";
        ofs.close();
        if (ofs.fail())
        {
            yCError(CIA402,
                    "CheckEncoderCalibration: error closing report file %s",
                    reportPath.c_str());
            return false;
        }

        yCInfo(CIA402, "CheckEncoderCalibration: report written to %s", reportPath.c_str());
        return true;
    }

    EthercatManager mgr;
};

CheckEncoderCalibration::CheckEncoderCalibration()
{
    m_impl = std::make_unique<Impl>();
}

CheckEncoderCalibration::~CheckEncoderCalibration() = default;

bool CheckEncoderCalibration::run(yarp::os::ResourceFinder& rf)
{
    const std::string ifname
        = rf.check("ifname") ? rf.find("ifname").asString() : std::string("eth0");

    // The reference TOML is required
    if (!rf.check("toml-input"))
    {
        yCError(CIA402,
                "CheckEncoderCalibration: missing required parameter 'toml-input' "
                "(path to the reference TOML file from store-home-position)");
        return false;
    }
    const std::string tomlPath = rf.find("toml-input").asString();

    // Report output path
    std::string reportPath;
    if (rf.check("report-output"))
    {
        reportPath = rf.find("report-output").asString();
    } else
    {
        // Generate default filename with current date and time
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        const std::tm tm = getLocalTime(t);
        std::ostringstream oss;
        oss << "encoder_calibration_check_" << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".md";
        reportPath = oss.str();
    }

    yCInfo(CIA402,
           "CheckEncoderCalibration: ifname=%s toml-input=%s report-output=%s",
           ifname.c_str(),
           tomlPath.c_str(),
           reportPath.c_str());

    return m_impl->run(ifname, tomlPath, reportPath);
}
