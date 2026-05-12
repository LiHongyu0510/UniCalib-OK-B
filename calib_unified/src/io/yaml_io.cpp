/**
 * UniCalib Unified — YAML/CSV I/O 实现
 * 
 * 工程化改造:
 *   - 使用 Status/ErrorCode 替代 throw std::runtime_error
 *   - 统一错误处理和日志
 */

#include "unicalib/io/yaml_io.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/sensor_types.h"
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace ns_unicalib {

// YAML 安全读取：若 key 存在则 as<T>()，否则返回 default_val
#define YAML_GET_OR(node, key, default_val) \
    ((node)[key] ? (node)[key].as<std::decay_t<decltype(default_val)>>() : (default_val))

// ===================================================================
// 读取系统配置 (YAML)
// 支持两种格式:
//   1) 统一格式 unicalib_example.yaml: sensors[] + reference_imu + data
//   2) 旧格式: imu_sensors / lidar_sensors / camera_sensors
// ===================================================================
SystemConfig YamlIO::load_system_config(const std::string& yaml_path) {
    UNICALIB_INFO("读取系统配置: {}", yaml_path);
    
    if (!fs::exists(yaml_path)) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_NOT_FOUND, 
                           "配置文件不存在: " + yaml_path);
    }
    
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        UNICALIB_THROW_DATA(ErrorCode::DATA_PARSE_ERROR,
                           "YAML 解析失败: " + yaml_path + "\n" + e.what());
    }
    
    SystemConfig cfg;
    cfg.config_file = yaml_path;
    
    // ---------- 统一格式: sensors 列表 (id, type, topic, 及嵌套 imu/lidar/camera) ----------
    if (root["sensors"]) {
        for (const auto& node : root["sensors"]) {
            std::string id_str = YAML_GET_OR(node, "id", std::string(""));
            std::string type_str = YAML_GET_OR(node, "type", std::string(""));
            SensorType st = sensor_type_from_str(type_str);
            SensorDesc d;
            d.sensor_id = id_str;
            d.topic = YAML_GET_OR(node, "topic", std::string(""));
            d.type = st;
            if (st == SensorType::IMU && node["imu"]) {
                const auto& imu = node["imu"];
                d.imu_params = SensorDesc::IMUParams{};
                d.imu_params->rate_hz = YAML_GET_OR(imu, "rate_hz", 200.0);
                std::string model_str = YAML_GET_OR(imu, "model", std::string("scale_misalignment"));
                d.imu_params->model = imu_model_from_str(model_str);
            }
            if (st == SensorType::LiDAR && node["lidar"]) {
                const auto& lidar = node["lidar"];
                d.lidar_params = SensorDesc::LiDARParams{};
                std::string lt_str = YAML_GET_OR(lidar, "type", std::string("spinning"));
                d.lidar_params->lidar_type = lidar_type_from_str(lt_str);
                d.lidar_params->scan_lines = YAML_GET_OR(lidar, "scan_lines", 16);
                d.lidar_params->rate_hz = YAML_GET_OR(lidar, "rate_hz", 10.0);
            }
            if (st == SensorType::CAMERA && node["camera"]) {
                const auto& cam = node["camera"];
                d.camera_params = SensorDesc::CameraParams{};
                std::string cm_str = YAML_GET_OR(cam, "model", std::string("pinhole"));
                d.camera_params->model = camera_model_from_str(cm_str);
                d.camera_params->width = YAML_GET_OR(cam, "width", 0);
                d.camera_params->height = YAML_GET_OR(cam, "height", 0);
                d.camera_params->fps = YAML_GET_OR(cam, "fps", 30.0);
                d.camera_params->is_rolling_shutter = YAML_GET_OR(cam, "rolling_shutter", false);
            }
            cfg.sensors.push_back(d);
        }
    }
    
    // ---------- 旧格式: 独立列表 imu_sensors / lidar_sensors / camera_sensors ----------
    if (cfg.sensors.empty()) {
        if (root["imu_sensors"]) {
            for (const auto& imu : root["imu_sensors"]) {
                SensorDesc d;
                d.sensor_id = YAML_GET_OR(imu, "id", std::string(""));
                d.topic = YAML_GET_OR(imu, "topic", std::string(""));
                d.type = SensorType::IMU;
                d.imu_params = SensorDesc::IMUParams{};
                d.imu_params->rate_hz = YAML_GET_OR(imu, "rate_hz", 200.0);
                cfg.sensors.push_back(d);
            }
        }
        if (root["lidar_sensors"]) {
            for (const auto& lidar : root["lidar_sensors"]) {
                SensorDesc d;
                d.sensor_id = YAML_GET_OR(lidar, "id", std::string(""));
                d.topic = YAML_GET_OR(lidar, "topic", std::string(""));
                d.type = SensorType::LiDAR;
                d.lidar_params = SensorDesc::LiDARParams{};
                cfg.sensors.push_back(d);
            }
        }
        if (root["camera_sensors"]) {
            for (const auto& cam : root["camera_sensors"]) {
                SensorDesc d;
                d.sensor_id = YAML_GET_OR(cam, "id", std::string(""));
                d.topic = YAML_GET_OR(cam, "topic", std::string(""));
                d.type = SensorType::CAMERA;
                d.camera_params = SensorDesc::CameraParams{};
                cfg.sensors.push_back(d);
            }
        }
    }
    
    // reference_imu (统一格式)
    if (root["reference_imu"])
        cfg.reference_imu = root["reference_imu"].as<std::string>();
    // output_dir
    if (root["output_dir"])
        cfg.output_dir = root["output_dir"].as<std::string>();
    // data.bag_file, data.imu, data.lidar, data.camera
    if (root["data"]) {
        const auto& data = root["data"];
        if (data["bag_file"]) cfg.bag_file = data["bag_file"].as<std::string>();
        if (data["imu"]) {
            for (const auto& kv : data["imu"]) {
                std::string id = kv.first.as<std::string>();
                cfg.imu_data_paths[id] = kv.second.as<std::string>();
            }
        }
        if (data["lidar"]) {
            for (const auto& kv : data["lidar"]) {
                std::string id = kv.first.as<std::string>();
                cfg.lidar_data_paths[id] = kv.second.as<std::string>();
            }
        }
        if (data["camera"]) {
            for (const auto& kv : data["camera"]) {
                std::string id = kv.first.as<std::string>();
                const auto& cam = kv.second;
                // 跳过非 map 项（如 camera_num）
                if (!cam.IsMap()) continue;
                if (cam["images_dir"]) cfg.camera_images_dirs[id] = cam["images_dir"].as<std::string>();
                if (cam["intrinsic_yaml"]) cfg.camera_intrinsic_yamls[id] = cam["intrinsic_yaml"].as<std::string>();
            }
        }
    }
    
    size_t n_imu = 0, n_lidar = 0, n_cam = 0;
    for (const auto& s : cfg.sensors) {
        if (s.type == SensorType::IMU) ++n_imu;
        else if (s.type == SensorType::LiDAR) ++n_lidar;
        else if (s.type == SensorType::CAMERA) ++n_cam;
    }
    UNICALIB_INFO("  IMU: {}, LiDAR: {}, Camera: {}",
                  n_imu, n_lidar, n_cam);
    if (cfg.sensors.empty()) {
        UNICALIB_THROW_DATA(ErrorCode::INVALID_CONFIG,
            "系统配置中未定义任何传感器 (sensors 或 imu_sensors/lidar_sensors/camera_sensors 至少一项非空); file=" + yaml_path);
    }
    return cfg;
}

// ===================================================================
// 保存标定结果 (YAML)
// ===================================================================
void YamlIO::save_calib_results(
    const CalibParamManager& params,
    const std::string& yaml_path) {
    
    params.save_yaml(yaml_path);
}

// ===================================================================
// IMU CSV 数据加载
// 格式: timestamp_s, gx, gy, gz, ax, ay, az (逗号或空格分隔)
// ===================================================================
IMURawData YamlIO::load_imu_csv(const std::string& csv_path) {
    UNICALIB_INFO("读取 IMU CSV: {}", csv_path);
    
    if (!fs::exists(csv_path)) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_NOT_FOUND,
                           "IMU CSV 文件不存在: " + csv_path);
    }
    
    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_READ_ERROR,
                           "无法打开 IMU CSV 文件: " + csv_path);
    }
    
    IMURawData data;
    std::string line;
    bool header_skipped = false;
    int skip_count = 0;
    size_t line_no = 0;
    
    while (std::getline(ifs, line)) {
        ++line_no;
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') continue;
        
        // 自动检测并跳过列标题行
        if (!header_skipped &&
            (line.find("timestamp") != std::string::npos ||
             line.find("time") != std::string::npos)) {
            header_skipped = true;
            continue;
        }
        header_skipped = true;
        
        // 替换逗号为空格
        for (char& c : line) if (c == ',') c = ' ';
        
        std::istringstream ss(line);
        IMURawFrame f;
        if (!(ss >> f.timestamp >> f.gyro[0] >> f.gyro[1] >> f.gyro[2]
                       >> f.accel[0] >> f.accel[1] >> f.accel[2])) {
            ++skip_count;
            continue;
        }
        bool finite_gyro = std::isfinite(f.gyro[0]) && std::isfinite(f.gyro[1]) && std::isfinite(f.gyro[2]);
        bool finite_accel = std::isfinite(f.accel[0]) && std::isfinite(f.accel[1]) && std::isfinite(f.accel[2]);
        if (!std::isfinite(f.timestamp) || !finite_gyro || !finite_accel) {
            UNICALIB_WARN("  IMU CSV 第 {} 行含非有限值，已跳过", line_no);
            ++skip_count;
            continue;
        }
        data.push_back(f);
    }
    
    UNICALIB_INFO("  加载 {} 帧 (跳过 {} 行)",
                  data.size(), skip_count);
    if (data.empty()) {
        UNICALIB_THROW_DATA(ErrorCode::INSUFFICIENT_DATA,
            "IMU CSV 未解析到有效帧: " + csv_path + " (共处理 " + std::to_string(line_no) + " 行)");
    }
    if (data.size() > 1) {
        double dur = data.back().timestamp - data.front().timestamp;
        double dt  = dur / (data.size() - 1);
        UNICALIB_INFO("  时长: {:.2f} s, 平均采样率: {:.1f} Hz",
                      dur, 1.0/dt);
    }
    
    return data;
}

// ===================================================================
// 读取相机内参 YAML
// 支持两种格式:
//   1) 扁平: width/height/fx/fy/cx/cy、dist_coeffs 在根节点
//   2) results 格式: projection_parameters{fx,fy,cx,cy}、distortion_parameters{k1,k2,p1,p2}、width/height 在根节点
// ===================================================================
CameraIntrinsics YamlIO::load_camera_intrinsics(const std::string& yaml_path) {
    UNICALIB_INFO("读取相机内参: {}", yaml_path);
    
    if (!fs::exists(yaml_path)) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_NOT_FOUND,
                           "相机内参文件不存在: " + yaml_path);
    }
    
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        UNICALIB_THROW_DATA(ErrorCode::DATA_PARSE_ERROR,
                           "YAML 解析失败: " + yaml_path + "\n" + e.what());
    }
    
    CameraIntrinsics I;
    I.width  = YAML_GET_OR(root, "width",  0);
    I.height = YAML_GET_OR(root, "height", 0);
    // 优先从 projection_parameters 读 fx/fy/cx/cy（与 results/camera_intrinsic/*.yaml 一致）
    YAML::Node proj = root["projection_parameters"];
    const YAML::Node& ref = (proj && proj.IsDefined()) ? proj : root;
    I.fx = YAML_GET_OR(ref, "fx", 0.0);
    I.fy = YAML_GET_OR(ref, "fy", 0.0);
    I.cx = YAML_GET_OR(ref, "cx", 0.0);
    I.cy = YAML_GET_OR(ref, "cy", 0.0);
    
    std::string model_str = YAML_GET_OR(root, "model", std::string("pinhole"));
    if (model_str == "fisheye") I.model = CameraIntrinsics::Model::FISHEYE;
    else                         I.model = CameraIntrinsics::Model::PINHOLE;
    
    if (root["dist_coeffs"]) {
        for (const auto& v : root["dist_coeffs"]) {
            I.dist_coeffs.push_back(v.as<double>());
        }
    } else if (root["distortion_parameters"]) {
        const auto& dp = root["distortion_parameters"];
        if (dp["k1"]) I.dist_coeffs.push_back(dp["k1"].as<double>());
        if (dp["k2"]) I.dist_coeffs.push_back(dp["k2"].as<double>());
        if (dp["p1"]) I.dist_coeffs.push_back(dp["p1"].as<double>());
        if (dp["p2"]) I.dist_coeffs.push_back(dp["p2"].as<double>());
        if (dp["k3"]) I.dist_coeffs.push_back(dp["k3"].as<double>());
    }
    
    UNICALIB_INFO("  fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f} dist={}",
                  I.fx, I.fy, I.cx, I.cy, I.dist_coeffs.size());
    return I;
}

// ===================================================================
// 保存相机内参到 YAML（与 results/camera_intrinsic/*.yaml 格式一致）
// 格式: sensor_id, model, method, width, height, distortion_model,
//       distortion_parameters{k1,k2,p1,p2[,k3]}, projection_parameters{fx,fy,cx,cy},
//       rms_reproj_error, num_images
// ===================================================================
void YamlIO::save_camera_intrinsics(
    const CameraIntrinsics& intrin,
    const std::string& yaml_path) {
    
    // 确保目录存在
    fs::path p(yaml_path);
    if (p.has_parent_path() && !fs::exists(p.parent_path())) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
        if (ec) {
            UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                               "无法创建目录: " + p.parent_path().string());
        }
    }
    // 从路径推导 sensor_id：camera_intrinsic_<sensor_id>.yaml
    std::string sensor_id = "camera_0";
    std::string stem = p.stem().string();
    if (stem.size() > 18 && stem.substr(0, 18) == "camera_intrinsic_") {
        sensor_id = stem.substr(18);
    }
    const std::string model_str = (intrin.model == CameraIntrinsics::Model::FISHEYE ? "fisheye" : "pinhole");
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    if (intrin.dist_coeffs.size() >= 4) {
        k1 = intrin.dist_coeffs[0];
        k2 = intrin.dist_coeffs[1];
        p1 = intrin.dist_coeffs[2];
        p2 = intrin.dist_coeffs[3];
        if (intrin.dist_coeffs.size() >= 5) k3 = intrin.dist_coeffs[4];
    }
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "sensor_id" << YAML::Value << sensor_id;
    out << YAML::Key << "model" << YAML::Value << model_str;
    out << YAML::Key << "method" << YAML::Value << "unicalib";
    out << YAML::Key << "width" << YAML::Value << intrin.width;
    out << YAML::Key << "height" << YAML::Value << intrin.height;
    out << YAML::Key << "distortion_model" << YAML::Value << "radial-tangential";
    out << YAML::Key << "distortion_parameters" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "k1" << YAML::Value << k1;
    out << YAML::Key << "k2" << YAML::Value << k2;
    out << YAML::Key << "p1" << YAML::Value << p1;
    out << YAML::Key << "p2" << YAML::Value << p2;
    if (intrin.dist_coeffs.size() >= 5)
        out << YAML::Key << "k3" << YAML::Value << k3;
    out << YAML::EndMap;
    out << YAML::Key << "projection_parameters" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "fx" << YAML::Value << intrin.fx;
    out << YAML::Key << "fy" << YAML::Value << intrin.fy;
    out << YAML::Key << "cx" << YAML::Value << intrin.cx;
    out << YAML::Key << "cy" << YAML::Value << intrin.cy;
    out << YAML::EndMap;
    out << YAML::Key << "rms_reproj_error" << YAML::Value << intrin.rms_reproj_error;
    out << YAML::Key << "num_images" << YAML::Value << intrin.num_images_used;
    out << YAML::EndMap;
    std::ofstream ofs(yaml_path);
    if (!ofs.is_open()) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                           "无法写入文件: " + yaml_path);
    }
    ofs << out.c_str() << "\n";
    UNICALIB_INFO("相机内参已保存: {}", yaml_path);
}

// ===================================================================
// 加载 IMU 内参 (支持 unicalib_imu_intrinsic 输出格式与 YamlIO 保存格式)
// ===================================================================
IMUIntrinsics YamlIO::load_imu_intrinsics(const std::string& yaml_path) {
    if (!fs::exists(yaml_path)) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_NOT_FOUND,
                           "IMU 内参文件不存在: " + yaml_path);
    }
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        UNICALIB_THROW_DATA(ErrorCode::DATA_PARSE_ERROR,
                           "IMU 内参 YAML 解析失败: " + yaml_path + "\n" + e.what());
    }
    IMUIntrinsics out;
    // 格式1: unicalib_imu_intrinsic app (noise_gyro, bias_instab_gyro, bias_gyro, bias_accel 等)
    if (root["noise_gyro"]) {
        out.noise_gyro       = root["noise_gyro"].as<double>();
        out.bias_instab_gyro = root["bias_instab_gyro"] ? root["bias_instab_gyro"].as<double>() : out.bias_instab_gyro;
        out.noise_acce       = root["noise_accel"] ? root["noise_accel"].as<double>() : (root["noise_acce"] ? root["noise_acce"].as<double>() : out.noise_acce);
        out.bias_instab_acce = root["bias_instab_accel"] ? root["bias_instab_accel"].as<double>() : out.bias_instab_acce;
        if (root["bias_gyro"] && root["bias_gyro"].IsSequence() && root["bias_gyro"].size() >= 3) {
            out.bias_gyro[0] = root["bias_gyro"][0].as<double>();
            out.bias_gyro[1] = root["bias_gyro"][1].as<double>();
            out.bias_gyro[2] = root["bias_gyro"][2].as<double>();
        }
        if (root["bias_accel"] && root["bias_accel"].IsSequence() && root["bias_accel"].size() >= 3) {
            out.bias_acce[0] = root["bias_accel"][0].as<double>();
            out.bias_acce[1] = root["bias_accel"][1].as<double>();
            out.bias_acce[2] = root["bias_accel"][2].as<double>();
        }
        if (root["num_samples_used"]) out.num_samples_used = root["num_samples_used"].as<int>();
        if (root["allan_fit_rms"])   out.allan_fit_rms    = root["allan_fit_rms"].as<double>();
        // 读取 T_Body_Imu 外参矩阵
        if (root["T_Body_Imu"]) {
            const auto& t_node = root["T_Body_Imu"];
            if (t_node["rows"] && t_node["cols"] && t_node["data"]) {
                int rows = t_node["rows"].as<int>();
                int cols = t_node["cols"].as<int>();
                if (rows == 4 && cols == 4 && t_node["data"].IsSequence() && t_node["data"].size() >= 16) {
                    const auto& data = t_node["data"];
                    for (int i = 0; i < 4; ++i) {
                        for (int j = 0; j < 4; ++j) {
                            out.T_Body_Imu(i, j) = data[i * 4 + j].as<double>();
                        }
                    }
                }
            }
        }
        return out;
    }
    // 格式2: YamlIO / Kalibr 风格 (gyroscope_noise_density, gyroscope_random_walk, ...)
    if (root["gyroscope_noise_density"])
        out.noise_gyro = root["gyroscope_noise_density"].as<double>();
    if (root["gyroscope_random_walk"])
        out.bias_instab_gyro = root["gyroscope_random_walk"].as<double>();
    if (root["accelerometer_noise_density"])
        out.noise_acce = root["accelerometer_noise_density"].as<double>();
    if (root["accelerometer_random_walk"])
        out.bias_instab_acce = root["accelerometer_random_walk"].as<double>();
    if (root["gyroscope_bias_init"] && root["gyroscope_bias_init"].IsSequence() && root["gyroscope_bias_init"].size() >= 3) {
        out.bias_gyro[0] = root["gyroscope_bias_init"][0].as<double>();
        out.bias_gyro[1] = root["gyroscope_bias_init"][1].as<double>();
        out.bias_gyro[2] = root["gyroscope_bias_init"][2].as<double>();
    }
    if (root["accelerometer_bias_init"] && root["accelerometer_bias_init"].IsSequence() && root["accelerometer_bias_init"].size() >= 3) {
        out.bias_acce[0] = root["accelerometer_bias_init"][0].as<double>();
        out.bias_acce[1] = root["accelerometer_bias_init"][1].as<double>();
        out.bias_acce[2] = root["accelerometer_bias_init"][2].as<double>();
    }
    if (root["num_samples_used"]) out.num_samples_used = root["num_samples_used"].as<int>();
    if (root["allan_fit_rms"])   out.allan_fit_rms    = root["allan_fit_rms"].as<double>();
    // 格式3: pipeline save (gyroscope.noise_density, accelerometer.bias_instability 等嵌套)
    if (root["gyroscope"]) {
        const auto& g = root["gyroscope"];
        if (g["noise_density"]) out.noise_gyro = g["noise_density"].as<double>();
        if (g["bias_instability"]) out.bias_instab_gyro = g["bias_instability"].as<double>();
    }
    if (root["accelerometer"]) {
        const auto& a = root["accelerometer"];
        if (a["noise_density"]) out.noise_acce = a["noise_density"].as<double>();
        if (a["bias_instability"]) out.bias_instab_acce = a["bias_instability"].as<double>();
    }
    return out;
}

// ===================================================================
// 保存 IMU 内参到 YAML (Kalibr / iKalibr 兼容格式)
// ===================================================================
void YamlIO::save_imu_intrinsics(
    const IMUIntrinsics& intrin,
    const std::string& yaml_path) {
    
    // 确保目录存在
    fs::path p(yaml_path);
    if (p.has_parent_path() && !fs::exists(p.parent_path())) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
        if (ec) {
            UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                               "无法创建目录: " + p.parent_path().string());
        }
    }
    
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "gyroscope_noise_density"
        << YAML::Value << intrin.noise_gyro;
    out << YAML::Key << "gyroscope_random_walk"
        << YAML::Value << intrin.bias_instab_gyro;
    out << YAML::Key << "accelerometer_noise_density"
        << YAML::Value << intrin.noise_acce;
    out << YAML::Key << "accelerometer_random_walk"
        << YAML::Value << intrin.bias_instab_acce;
    
    out << YAML::Key << "gyroscope_bias_init"
        << YAML::Value << YAML::Flow
        << std::vector<double>{intrin.bias_gyro[0], intrin.bias_gyro[1], intrin.bias_gyro[2]};
    out << YAML::Key << "accelerometer_bias_init"
        << YAML::Value << YAML::Flow
        << std::vector<double>{intrin.bias_acce[0], intrin.bias_acce[1], intrin.bias_acce[2]};
    
    out << YAML::Key << "num_samples_used" << YAML::Value << intrin.num_samples_used;
    out << YAML::Key << "allan_fit_rms"    << YAML::Value << intrin.allan_fit_rms;
    out << YAML::EndMap;
    
    std::ofstream ofs(yaml_path);
    if (!ofs.is_open()) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                           "无法写入文件: " + yaml_path);
    }
    ofs << out.c_str() << "\n";
    UNICALIB_INFO("IMU 内参已保存: {}", yaml_path);
}

// ===================================================================
// 解析 YAML 4×4 行优先矩阵节点
// ===================================================================
std::vector<double> parse_4x4_matrix_from_yaml(const YAML::Node& node) {
    if (!node || !node["rows"] || !node["cols"] || !node["data"] || !node["data"].IsSequence())
        return {};
    if (node["rows"].as<int>() != 4 || node["cols"].as<int>() != 4)
        return {};
    const auto& data = node["data"];
    if (data.size() < 16)
        return {};
    std::vector<double> v;
    v.reserve(16);
    for (size_t i = 0; i < 16; ++i)
        v.push_back(data[i].as<double>());
    return v;
}

}  // namespace ns_unicalib
