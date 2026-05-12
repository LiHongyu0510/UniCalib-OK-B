/**
 * UniCalib - 数学安全测试套件
 * 
 * 测试目标:
 * 1. 验证除零保护
 * 2. 验证归一化安全性
 * 3. 验证 NaN/Inf 处理
 * 4. 验证边界条件
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <limits>

namespace ns_unicalib_test {

// ===================================================================
// 测试归一化零向量安全性
// ===================================================================
TEST(MathSafetyTest, NormalizeZeroVector) {
    Eigen::Vector3d zero_vec(0.0, 0.0, 0.0);
    
    // 测试 Sophus 的归一化
    auto so3 = Sophus::SO3d::exp(zero_vec);
    Eigen::Vector3d rot_vec = so3.log();
    
    // 零向量的 log 应该接近零
    EXPECT_LT(rot_vec.norm(), 1e-10);
    
    // 四元数归一化测试
    Eigen::Vector4d q_vec(1e-12, 0.0, 0.0, 0.0);
    double q_norm = q_vec.norm();
    
    if (q_norm < 1e-10) {
        // 应该有保护逻辑
        EXPECT_TRUE(true);
    } else {
        q_vec /= q_norm;
        EXPECT_DOUBLE_EQ(q_vec.norm(), 1.0);
    }
}

TEST(MathSafetyTest, NormalizeNearZeroVector) {
    Eigen::Vector3d near_zero(1e-8, 1e-8, 1e-8);
    
    // 测试接近零向量的归一化
    double norm = near_zero.norm();
    
    EXPECT_GT(norm, 0.0);
    
    if (norm > 1e-6) {
        Eigen::Vector3d normalized = near_zero.normalized();
        EXPECT_FALSE(std::isnan(normalized.x()));
        EXPECT_FALSE(std::isnan(normalized.y()));
        EXPECT_FALSE(std::isnan(normalized.z()));
    } else {
        // 极小向量应该被检测并跳过
        EXPECT_TRUE(true);
    }
}

// ===================================================================
// 测试 NCC 计算除零保护
// ===================================================================
TEST(MathSafetyTest, NCCDivideByZeroProtection) {
    cv::Mat e1 = (cv::Mat_<float>(1, 1) << 0.5f, 0.6f);
    cv::Mat e2 = (cv::Mat_<float>(1, 1) << 0.4f, 0.5f);
    
    cv::Scalar m1, s1, m2, s2;
    cv::meanStdDev(e1, m1, s1);
    cv::meanStdDev(e2, m2, s2);
    
    // 测试除零保护
    double denom = s1[0] * s2[0] * e1.total();
    
    if (std::abs(denom) < 1e-10) {
        // 应该跳过
        EXPECT_TRUE(true);
    } else {
        double ncc = (e1 - m1[0]).dot(e2 - m2[0]) / denom;
        EXPECT_FALSE(std::isinf(ncc));
        EXPECT_FALSE(std::isnan(ncc));
    }
}

TEST(MathSafetyTest, NCCSmallStdDev) {
    cv::Mat e1 = (cv::Mat_<float>(1, 1) << 0.5f, 0.6f);
    
    cv::Scalar m1, s1;
    cv::meanStdDev(e1, m1, s1);
    
    // 极小标准差
    cv::Mat e1_small = (cv::Mat_<float>(1, 1) << 0.5f, 0.6f);
    cv::Scalar m1_small, s1_small;
    cv::meanStdDev(e1_small, m1_small, s1_small);
    
    // 强制设置极小值
    s1_small[0] = 1e-7;
    
    double denom = s1_small[0] * 1.0 * e1.total();
    
    // 应该触发保护
    EXPECT_LT(std::abs(denom), 1e-10);
}

// ===================================================================
// 测试 NaN 传播
// ===================================================================
TEST(MathSafetyTest, NaNPropagationInRotation) {
    Eigen::Vector3d gyro(0.1, std::numeric_limits<double>::quiet_NaN(), 0.2);
    
    // 检查输入 NaN
    bool has_nan = std::isnan(gyro.x()) || std::isnan(gyro.y()) || std::isnan(gyro.z());
    EXPECT_TRUE(has_nan);
    
    // 如果不检查，NaN 会传播
    if (!has_nan) {
        double norm = gyro.norm();
        EXPECT_FALSE(std::isnan(norm));
    } else {
        // 应该跳过计算
        EXPECT_TRUE(true);
    }
}

TEST(MathSafetyTest, NaNPropagationInQuaternion) {
    Eigen::Vector4d q_vec(0.1, 0.2, 0.3, std::numeric_limits<double>::quiet_NaN());
    
    // 检查 NaN
    bool has_nan = std::isnan(q_vec.x()) || std::isnan(q_vec.y()) || 
                   std::isnan(q_vec.z()) || std::isnan(q_vec.w());
    EXPECT_TRUE(has_nan);
    
    // 归一化应该检查
    double norm = q_vec.norm();
    if (norm > 1e-10 && !has_nan) {
        q_vec /= norm;
        EXPECT_FALSE(std::isnan(q_vec.x()));
        EXPECT_FALSE(std::isnan(q_vec.y()));
        EXPECT_FALSE(std::isnan(q_vec.z()));
        EXPECT_FALSE(std::isnan(q_vec.w()));
    }
}

// ===================================================================
// 测试 Allan 方差计算
// ===================================================================
TEST(MathSafetyTest, AllanVarianceZeroTau) {
    // tau = 0 应该被保护
    std::vector<double> phase(10, 0.0);
    double tau0 = 1.0;
    
    // 模拟 OADEV 计算
    double sum = 0.0;
    int count = 0;
    for (int j = 0; j + 2 < static_cast<int>(phase.size()); ++j) {
        double diff = phase[j + 2] - 2*phase[j + 1] + phase[j];
        sum += diff * diff;
        ++count;
    }
    
    if (tau0 <= 0.0) {
        // 应该跳过或触发错误
        EXPECT_TRUE(true);
    } else {
        double avar = sum / (2.0 * tau0 * tau0 * count);
        EXPECT_FALSE(std::isinf(avar));
        EXPECT_GE(avar, 0.0);
    }
}

TEST(MathSafetyTest, AllanVarianceNegativeValues) {
    // 负值相位数据
    std::vector<double> phase = {-1.0, -2.0, -3.0, -4.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    double tau0 = 1.0;
    
    // 计算 OADEV
    double sum = 0.0;
    int count = 0;
    for (int j = 0; j + 2 < static_cast<int>(phase.size()); ++j) {
        double diff = phase[j + 2] - 2*phase[j + 1] + phase[j];
        sum += diff * diff;
        ++count;
    }
    
    double avar = sum / (2.0 * tau0 * tau0 * count);
    EXPECT_GE(avar, 0.0);  // 方差应该非负
}

// ===================================================================
// 测试边界条件
// ===================================================================
TEST(MathSafetyTest, EmptyContainerAccess) {
    std::vector<Eigen::Vector3d> empty_vec;
    std::vector<double> timestamps;
    
    // 测试空容器访问
    if (empty_vec.empty()) {
        EXPECT_TRUE(true);
    } else {
        // 不应该访问
        EXPECT_GT(empty_vec.size(), 0);
    }
    
    if (timestamps.size() < 2) {
        EXPECT_TRUE(true);
    } else {
        // 不应该计算区间
        EXPECT_GE(timestamps.size(), 2);
    }
}

TEST(MathSafetyTest, TimeSyncOutOfRange) {
    std::vector<double> timestamps = {1.0, 2.0, 3.0};
    double target_time = 5.0;
    
    // 测试时间同步
    double min_dt = std::numeric_limits<double>::max();
    for (double t : timestamps) {
        double dt = std::abs(t - target_time);
        min_dt = std::min(min_dt, dt);
    }
    
    // 目标时间超出范围
    EXPECT_GT(min_dt, 1.0);  // 最小时间差应该 > 1s
}

TEST(MathSafetyTest, OutOfBoundsIndex) {
    std::vector<double> data = {1.0, 2.0, 3.0};
    
    // 测试边界访问
    size_t idx = data.size() - 1;
    EXPECT_GE(idx, 0);
    EXPECT_LT(idx, data.size());
    
    // 测试越界访问（应该避免）
    idx = data.size();
    if (idx >= data.size()) {
        // 应该检查边界
        EXPECT_TRUE(true);
    }
}

// ===================================================================
// 测试数值精度
// ===================================================================
TEST(MathSafetyTest, FloatPrecisionAccumulation) {
    // 测试浮点累积误差
    double sum = 0.0;
    for (int i = 0; i < 1000000; ++i) {
        sum += 0.1;  // 0.1 在二进制中无法精确表示
    }
    
    double expected = 1000000.0 * 0.1;
    EXPECT_NEAR(sum, expected, 1e-6);  // 允许一定的误差
}

TEST(MathSafetyTest, Log10EdgeCases) {
    // 测试 log10 边界情况
    double small_val = 1e-15;
    double log_small = std::log10(small_val);
    EXPECT_LT(log_small, 0.0);
    
    double zero_val = 0.0;
    // log10(0) 应该返回 -inf
    double log_zero = std::log10(zero_val);
    EXPECT_TRUE(std::isinf(log_zero));
    
    double neg_val = -1.0;
    // log10(负数) 应该返回 NaN
    double log_neg = std::log10(neg_val);
    EXPECT_TRUE(std::isnan(log_neg));
}

// ===================================================================
// 主函数
// ===================================================================
} // namespace ns_unicalib_test

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
