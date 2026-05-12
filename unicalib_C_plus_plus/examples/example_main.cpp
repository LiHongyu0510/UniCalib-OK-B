/**
 * UniCalib C++ 示例：加载配置、列举传感器与标定对、可选运行流水线（含异常处理）
 */
#include <unicalib/system.hpp>
#include <unicalib/transforms.hpp>
#include <unicalib/allan_variance.hpp>
#include <unicalib/exceptions.hpp>
#include <iostream>
#include <cstdlib>

int main(int argc, char** argv) {
  std::string config_path = (argc > 1) ? argv[1] : "config/sensors.yaml";
  std::string data_path = (argc > 2) ? argv[2] : ".";

  std::cout << "UniCalib C++ Example\n";
  std::cout << "  config: " << config_path << "\n  data:   " << data_path << "\n";

  try {
    unicalib::UniCalibSystem system(config_path);

    std::cout << "\nSensors:\n";
    for (const auto& s : system.sensors_list()) {
      std::cout << "  " << s.sensor_id << " (" << unicalib::to_string(s.sensor_type)
                << ") topic=" << s.topic << "\n";
    }
    std::cout << "\nCalib pairs:\n";
    for (const auto& p : system.calib_pairs()) {
      std::cout << "  [" << p.sensor_a << "] <-> [" << p.sensor_b
                << "] coarse=" << p.method_coarse << " fine=" << p.method_fine
                << " priority=" << p.priority << "\n";
    }

    // 几何工具示例
    Eigen::Matrix3d R = unicalib::euler_to_rotation_matrix(Eigen::Vector3d(0, 0, 30), true);
    Eigen::Vector3d euler = unicalib::rotation_matrix_to_euler(R, true);
    std::cout << "\nTransform demo: euler(0,0,30)deg -> R -> euler = ("
              << euler(0) << ", " << euler(1) << ", " << euler(2) << ") deg\n";

    bool run_pipeline = (argc > 2 || std::getenv("RUN_PIPELINE"));
    if (run_pipeline) {
      std::cout << "\nRunning full pipeline...\n";
      system.run_full_pipeline(data_path);
    } else {
      std::cout << "\nSkip pipeline (set RUN_PIPELINE=1 or pass data path to run).\n";
    }

    return 0;
  } catch (const unicalib::UniCalibException& e) {
    std::cerr << "UniCalib error: " << e.what() << "\n";
    if (!e.context().suggestion.empty())
      std::cerr << "Suggestion: " << e.context().suggestion << "\n";
    return static_cast<int>(e.error_code());
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "Unknown error.\n";
    return 1;
  }
}
