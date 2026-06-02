#include "unicalib/io/oem7_imu_scale.h"

#include <cmath>

namespace ns_unicalib {
namespace {

constexpr double kOneG = 9.80665;

double arcsecondsToRadians(double arcsecs) {
    return arcsecs * M_PI / (180.0 * 3600.0);
}

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double feetToMeters(double feet) {
    return feet / 3.2808;
}

}  // namespace

int oem7_default_imu_rate_hz(Oem7ImuType type) {
    switch (type) {
        case Oem7ImuType::Hg1700Ag11:
        case Oem7ImuType::Hg1700Ag17:
        case Oem7ImuType::Hg1900Ca29:
        case Oem7ImuType::Hg1700Ag58:
        case Oem7ImuType::Hg1700Ag62:
        case Oem7ImuType::Hg1930Aa99:
        case Oem7ImuType::Hg1900Ca50:
        case Oem7ImuType::Hg1930Ca50:
        case Oem7ImuType::Isa100c:
        case Oem7ImuType::Hg4930An04:
            return 100;
        case Oem7ImuType::Ln200:
        case Oem7ImuType::ImarFsas:
        case Oem7ImuType::KvhCots:
        case Oem7ImuType::Adis16488:
        case Oem7ImuType::Kvh1750:
        case Oem7ImuType::LitefMicroimu:
        case Oem7ImuType::Hg4930An01:
        case Oem7ImuType::EpsonG370:
        case Oem7ImuType::EpsonG320_200Hz:
            return 200;
        case Oem7ImuType::Stim300:
        case Oem7ImuType::Stim300d:
        case Oem7ImuType::EpsonG320:
            return 125;
        case Oem7ImuType::Hg4930An04_400Hz:
            return 400;
        default:
            return 0;
    }
}

bool oem7_get_imu_raw_scale_factors(Oem7ImuType type, int imu_rate_hz, double& gyro_scale, double& acc_scale) {
    if (imu_rate_hz <= 0) {
        return false;
    }
    const int rate = imu_rate_hz;

    switch (type) {
        case Oem7ImuType::Ln200:
            gyro_scale = std::pow(2.0, -19);
            acc_scale = std::pow(2.0, -14);
            break;
        case Oem7ImuType::Hg1900Ca29:
        case Oem7ImuType::Hg1900Ca50:
        case Oem7ImuType::Hg1930Ca50:
        case Oem7ImuType::Hg1930Aa99:
        case Oem7ImuType::Hg1700Ag11:
        case Oem7ImuType::Hg1700Ag58:
            gyro_scale = std::pow(2.0, -33);
            acc_scale = feetToMeters(std::pow(2.0, -27));
            break;
        case Oem7ImuType::Hg1700Ag17:
        case Oem7ImuType::Hg1700Ag62:
            gyro_scale = std::pow(2.0, -33);
            acc_scale = feetToMeters(std::pow(2.0, -26));
            break;
        case Oem7ImuType::ImarFsas:
            gyro_scale = arcsecondsToRadians(0.1 / std::pow(2, 8));
            acc_scale = 0.05 / std::pow(2.0, 15);
            break;
        case Oem7ImuType::Isa100c:
        case Oem7ImuType::LitefMicroimu:
            gyro_scale = 1.0E-9;
            acc_scale = 2.0E-8;
            break;
        case Oem7ImuType::Adis16488:
            gyro_scale = degreesToRadians(720.0 / std::pow(2, 31));
            acc_scale = 200.0 / std::pow(2, 31);
            break;
        case Oem7ImuType::Stim300:
        case Oem7ImuType::Stim300d:
            gyro_scale = degreesToRadians(std::pow(2, -21));
            acc_scale = std::pow(2, -22);
            break;
        case Oem7ImuType::Kvh1750:
            gyro_scale = 0.1 / (3600.0 * 256.0);
            acc_scale = 0.05 * std::pow(2.0, -15);
            break;
        case Oem7ImuType::EpsonG320:
        case Oem7ImuType::EpsonG320_200Hz:
            gyro_scale = degreesToRadians(0.008 / std::pow(2.0, 16)) / rate;
            acc_scale = ((0.2 / std::pow(2.0, 16)) * (kOneG / 1000.0)) / rate;
            break;
        case Oem7ImuType::Hg4930An01:
        case Oem7ImuType::Hg4930An04:
        case Oem7ImuType::Hg4930An04_400Hz:
            gyro_scale = std::pow(2.0, -33);
            acc_scale = std::pow(2.0, -29);
            break;
        case Oem7ImuType::EpsonG370:
            gyro_scale = degreesToRadians(0.0151515 / std::pow(2.0, 16)) / rate;
            acc_scale = ((0.4 / std::pow(2.0, 16)) * (kOneG / 1000.0)) / rate;
            break;
        case Oem7ImuType::KvhCots:
            gyro_scale = 0.1 / (3600.0 * 256.0);
            acc_scale = 0.05 / std::pow(2.0, 15);
            break;
        case Oem7ImuType::Unknown:
        default:
            return false;
    }
    return true;
}

}  // namespace ns_unicalib
