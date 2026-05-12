// iKalibr: Unified Targetless Spatiotemporal Calibration Framework
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/iKalibr.git
//
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
//
// Purpose: See .h/.hpp file.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "sensor/radar_data_loader.h"
#include "ikalibr/msg/ainstein_radar_target_array.hpp"
#include "ikalibr/msg/awr1843_radar_scan.hpp"
#include "ikalibr/msg/awr1843_radar_scan_custom.hpp"
#include "util/status.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "spdlog/fmt/fmt.h"

namespace {
bool IKALIBR_UNIQUE_NAME(_2_) = ns_ikalibr::_1_(__FILE__);
inline double stampToSec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}
inline bool stampIsZero(const builtin_interfaces::msg::Time& t) {
  return t.sec == 0 && t.nanosec == 0;
}
}  // namespace

namespace ns_ikalibr {

RadarDataLoader::RadarDataLoader(RadarModelType radarModel)
    : _radarModel(radarModel) {}

RadarDataLoader::Ptr RadarDataLoader::GetLoader(const std::string &radarModelStr) {
    // try extract radar model
    RadarModelType radarModel;
    try {
        radarModel = EnumCast::stringToEnum<RadarModelType>(radarModelStr);
    } catch (...) {
        throw Status(Status::WARNING, RadarModel::UnsupportedRadarModelMsg(radarModelStr));
    }
    RadarDataLoader::Ptr radarDataLoader;
    switch (radarModel) {
        case RadarModelType::AINSTEIN_RADAR:
            radarDataLoader = AinsteinRadarLoader::Create(radarModel);
            break;
        case RadarModelType::AWR1843BOOST_RAW:
            radarDataLoader = AWR1843BOOSTRawLoader::Create(radarModel);
            break;
        case RadarModelType::POINTCLOUD2_POSV:
            radarDataLoader = PointCloud2POSVLoader::Create(radarModel);
            break;
        case RadarModelType::POINTCLOUD2_POSIV:
            radarDataLoader = PointCloud2POSIVLoader::Create(radarModel);
            break;
        case RadarModelType::AWR1843BOOST_CUSTOM:
            radarDataLoader = AWR1843BOOSTCustomLoader::Create(radarModel);
            break;
        case RadarModelType::POINTCLOUD2_XRIO:
            radarDataLoader = PointCloud2XRIOLoader::Create(radarModel);
            break;
        default:
            throw Status(Status::WARNING, RadarModel::UnsupportedRadarModelMsg(radarModelStr));
    }
    return radarDataLoader;
}

RadarModelType RadarDataLoader::GetRadarModel() const { return _radarModel; }

// -------------------
// AinsteinRadarLoader
// -------------------

AinsteinRadarLoader::AinsteinRadarLoader(RadarModelType radarModel)
    : RadarDataLoader(radarModel) {}

AinsteinRadarLoader::Ptr AinsteinRadarLoader::Create(RadarModelType radarModel) {
    return std::make_shared<AinsteinRadarLoader>(radarModel);
}

RadarTargetArray::Ptr AinsteinRadarLoader::UnpackScan(const rosbag::MessageInstance &msgInstance) {
    auto msg = msgInstance.instantiate<ikalibr::msg::AinsteinRadarTargetArray>();

    CheckMessage<ikalibr::msg::AinsteinRadarTargetArray>(msg);

    std::vector<RadarTarget::Ptr> targets;
    targets.reserve(msg->targets.size());

    for (int i = 0; i < static_cast<int>(msg->targets.size()); ++i) {
        const auto &tar = msg->targets.at(i);
        if (tar.range < 0.5) {
            continue;
        }
        targets.push_back(RadarTarget::Create(stampToSec(msg->header.stamp),
                                              {tar.range, tar.azimuth, tar.elevation, tar.speed}));
    }
    if (stampIsZero(msg->header.stamp)) {
        return RadarTargetArray::Create(targets.back()->GetTimestamp(), targets);
    } else {
        return RadarTargetArray::Create(stampToSec(msg->header.stamp), targets);
    }
}

// ---------------------
// AWR1843BOOSTRawLoader
// ---------------------

AWR1843BOOSTRawLoader::AWR1843BOOSTRawLoader(RadarModelType radarModel)
    : RadarDataLoader(radarModel) {}

AWR1843BOOSTRawLoader::Ptr AWR1843BOOSTRawLoader::Create(RadarModelType radarModel) {
    return std::make_shared<AWR1843BOOSTRawLoader>(radarModel);
}

RadarTargetArray::Ptr AWR1843BOOSTRawLoader::UnpackScan(
    const rosbag::MessageInstance &msgInstance) {
    auto msg = msgInstance.instantiate<ikalibr::msg::AWR1843RadarScan>();

    CheckMessage<ikalibr::msg::AWR1843RadarScan>(msg);

    auto target =
        RadarTarget::Create(stampToSec(msg->header.stamp), {msg->x, msg->y, msg->z}, msg->velocity);

    if (target->GetRange() < 0.5) {
        return nullptr;
    }
    if (stampIsZero(msg->header.stamp)) {
        return RadarTargetArray::Create(target->GetTimestamp(), {target});
    } else {
        return RadarTargetArray::Create(stampToSec(msg->header.stamp), {target});
    }
}

// ---------------------
// PointCloud2POSVLoader
// ---------------------

PointCloud2POSVLoader::PointCloud2POSVLoader(RadarModelType radarModel)
    : RadarDataLoader(radarModel) {}

PointCloud2POSVLoader::Ptr PointCloud2POSVLoader::Create(RadarModelType radarModel) {
    return std::make_shared<PointCloud2POSVLoader>(radarModel);
}

RadarTargetArray::Ptr PointCloud2POSVLoader::UnpackScan(
    const rosbag::MessageInstance &msgInstance) {
    auto msg = msgInstance.instantiate<sensor_msgs::msg::PointCloud2>();

    CheckMessage<sensor_msgs::msg::PointCloud2>(msg);

    RadarPOSVCloud radarTargets;
    pcl::fromROSMsg(*msg, radarTargets);

    std::vector<RadarTarget::Ptr> targets;
    targets.reserve(radarTargets.size());

    const double msgTime = stampToSec(msg->header.stamp);
    for (const auto &tar : radarTargets) {
        if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) ||
            std::isnan(tar.velocity)) {
            continue;
        }
        if (Eigen::Vector3f(tar.x, tar.y, tar.z).squaredNorm() < 0.25f) {
            continue;
        }
        targets.push_back(RadarTarget::Create(msgTime, {tar.x, tar.y, tar.z}, tar.velocity));
    }
    if (stampIsZero(msg->header.stamp)) {
        return RadarTargetArray::Create(targets.back()->GetTimestamp(), targets);
    } else {
        return RadarTargetArray::Create(msgTime, targets);
    }
}

// ----------------------
// PointCloud2POSIVLoader
// ----------------------

PointCloud2POSIVLoader::PointCloud2POSIVLoader(RadarModelType radarModel)
    : RadarDataLoader(radarModel) {}

PointCloud2POSIVLoader::Ptr PointCloud2POSIVLoader::Create(RadarModelType radarModel) {
    return std::make_shared<PointCloud2POSIVLoader>(radarModel);
}

RadarTargetArray::Ptr PointCloud2POSIVLoader::UnpackScan(
    const rosbag::MessageInstance &msgInstance) {
    auto msg = msgInstance.instantiate<sensor_msgs::msg::PointCloud2>();

    CheckMessage<sensor_msgs::msg::PointCloud2>(msg);

    RadarPOSIVCloud radarTargets;
    pcl::fromROSMsg(*msg, radarTargets);

    std::vector<RadarTarget::Ptr> targets;
    targets.reserve(radarTargets.size());

    const double msgTime = stampToSec(msg->header.stamp);
    for (const auto &tar : radarTargets) {
        if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) ||
            std::isnan(tar.velocity)) {
            continue;
        }
        if (Eigen::Vector3f(tar.x, tar.y, tar.z).squaredNorm() < 0.25) {
            continue;
        }
        targets.push_back(RadarTarget::Create(msgTime, {tar.x, tar.y, tar.z}, tar.velocity));
    }
    if (stampIsZero(msg->header.stamp)) {
        return RadarTargetArray::Create(targets.back()->GetTimestamp(), targets);
    } else {
        return RadarTargetArray::Create(msgTime, targets);
    }
}

// ------------------------
// AWR1843BOOSTCustomLoader
// ------------------------

AWR1843BOOSTCustomLoader::AWR1843BOOSTCustomLoader(RadarModelType radarModel)
    : RadarDataLoader(radarModel) {}

AWR1843BOOSTCustomLoader::Ptr AWR1843BOOSTCustomLoader::Create(RadarModelType radarModel) {
    return std::make_shared<AWR1843BOOSTCustomLoader>(radarModel);
}

RadarTargetArray::Ptr AWR1843BOOSTCustomLoader::UnpackScan(
    const rosbag::MessageInstance &msgInstance) {
    auto msg = msgInstance.instantiate<ikalibr::msg::AWR1843RadarScanCustom>();

    CheckMessage<ikalibr::msg::AWR1843RadarScanCustom>(msg);

    auto target =
        RadarTarget::Create(stampToSec(msg->header.stamp), {msg->x, msg->y, msg->z}, msg->velocity);

    if (target->GetRange() < 0.5) {
        return nullptr;
    }
    if (stampIsZero(msg->header.stamp)) {
        return RadarTargetArray::Create(target->GetTimestamp(), {target});
    } else {
        return RadarTargetArray::Create(stampToSec(msg->header.stamp), {target});
    }
}

// ---------------------
// PointCloud2XRIOLoader
// ---------------------

PointCloud2XRIOLoader::PointCloud2XRIOLoader(RadarModelType radarModel)
    : RadarDataLoader(radarModel) {}

PointCloud2XRIOLoader::Ptr PointCloud2XRIOLoader::Create(RadarModelType radarModel) {
    return std::make_shared<PointCloud2POSIVLoader>(radarModel);
}

RadarTargetArray::Ptr PointCloud2XRIOLoader::UnpackScan(
    const rosbag::MessageInstance &msgInstance) {
    auto msg = msgInstance.instantiate<sensor_msgs::msg::PointCloud2>();

    CheckMessage<sensor_msgs::msg::PointCloud2>(msg);

    RadarXRIOCloud radarTargets;
    pcl::fromROSMsg(*msg, radarTargets);

    std::vector<RadarTarget::Ptr> targets;
    targets.reserve(radarTargets.size());

    const double msgTime = stampToSec(msg->header.stamp);
    for (const auto &tar : radarTargets) {
        if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) ||
            std::isnan(tar.v_doppler_mps)) {
            continue;
        }
        if (Eigen::Vector3f(tar.x, tar.y, tar.z).squaredNorm() < 0.25) {
            continue;
        }
        targets.push_back(RadarTarget::Create(msgTime, {tar.x, tar.y, tar.z},
                                              tar.v_doppler_mps));
    }

    if (stampIsZero(msg->header.stamp)) {
        return RadarTargetArray::Create(targets.back()->GetTimestamp(), targets);
    } else {
        return RadarTargetArray::Create(msgTime, targets);
    }
}

}  // namespace ns_ikalibr