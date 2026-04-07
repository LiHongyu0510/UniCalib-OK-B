/**
 * UniCalib Unified — Pangolin 3D渲染引擎
 * 
 * 功能:
 *   1. 点云渲染 (参考/目标点云)
 *   2. 相机视锥体绘制
 *   3. 坐标系轴显示
 *   4. 网格平面
 *   5. 重投影误差可视化
 *   6. 实时交互
 */

#pragma once

// 仅在启用 Pangolin 时包含相关头文件
#if UNICALIB_WITH_PANGOLIN

// 包含 Pangolin 各组件头文件 (Pangolin 0.9.0 没有单一的 pangolin.h)
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/windowing/window.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/glsl_utilities.h>
#include <pangolin/handler/handler.h>

// Pangolin 0.9.0 兼容层
#include <unicalib/viz/pangolin_compat.h>

#endif  // UNICALIB_WITH_PANGOLIN

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <memory>
#include <map>
#include <vector>
#include <string>
#include <optional>

namespace ns_unicalib {

// ===========================================================================
// 数据结构
// ===========================================================================

struct PointCloudRenderData {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::Vector3f color = Eigen::Vector3f(0.5f, 0.5f, 0.5f);
    float opacity = 0.5f;
    float point_size = 2.0f;
    std::string name;
};

struct CameraFrustumData {
    Eigen::Matrix3d K;           // 内参矩阵
    int width, height;           // 图像尺寸
    Eigen::Matrix4d T_cam_lidar; // 相机到LiDAR的外参
    float scale = 1.0f;
    Eigen::Vector3f color = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
};

struct CoordinateFrameData {
    Eigen::Matrix4d T;           // 变换矩阵
    float scale = 0.3f;
};

// ===========================================================================
// PangolinRender 类
// ===========================================================================

#if UNICALIB_WITH_PANGOLIN

class PangolinRender {
public:
    using Ptr = std::shared_ptr<PangolinRender>;
    
    PangolinRender();
    ~PangolinRender();
    
    /**
     * @brief 初始化窗口
     */
    void init(const std::string& title = "UniCalib 3D Viewer", 
              int width = 1280, int height = 720);
    
    /**
     * @brief 是否运行中
     */
    bool is_running() const;

    /**
     * @brief 关闭窗口并释放 Pangolin 上下文（标定结束后自动关闭界面时调用）
     */
    void close();

    /**
     * @brief 渲染一帧并处理输入
     * @return true 继续, false 退出
     */
    bool render();
    
    // -------------------------------------------------------------------------
    // 数据设置
    // -------------------------------------------------------------------------
    
    /**
     * @brief 设置参考点云
     */
    void set_ref_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                       const Eigen::Vector3f& color = Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    
    /**
     * @brief 设置目标点云
     */
    void set_target_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          const Eigen::Vector3f& color = Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    
    /**
     * @brief 添加点云
     */
    void add_point_cloud(const std::string& name, 
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                         const Eigen::Vector3f& color);
    
    /**
     * @brief 设置当前外参 (用于渲染配准后的点云)
     */
    void set_extrinsic(const Eigen::Matrix4d& T);
    
    /**
     * @brief 设置相机视锥体
     */
    void set_camera(const CameraFrustumData& cam);
    
    /**
     * @brief 设置坐标系
     */
    void set_coordinate_frame(const CoordinateFrameData& frame);
    
    /**
     * @brief 显示/隐藏网格
     */
    void set_show_grid(bool show);
    
    /**
     * @brief 设置点大小
     */
    void set_point_size(float size);
    
    // -------------------------------------------------------------------------
    // 交互控制
    // -------------------------------------------------------------------------
    
    /**
     * @brief 获取当前相机姿态 (用于轨道控制)
     */
    Eigen::Matrix4d get_camera_pose() const;
    
    /**
     * @brief 设置相机姿态
     */
    void set_camera_pose(const Eigen::Matrix4d& T);
    
    /**
     * @brief 重置视角
     */
    void reset_view();
    
    /**
     * @brief 截图
     */
    void save_screenshot(const std::string& filename);
    
private:
    // 内部渲染方法
    void render_point_clouds();
    void render_camera_frustum();
    void render_coordinate_frames();
    void render_grid();
    void render_axis();
    
    // 点云VBOs - 使用指针因为 pangolin::GlBuffer 不可复制
    struct PointCloudVBOs {
        pangolin::GlVertexBuffer vbo;
        size_t num_points = 0;
        Eigen::Vector3f color;
        float opacity;
        
        PointCloudVBOs() = default;
        
        // 显式移动构造函数
        PointCloudVBOs(PointCloudVBOs&& other) noexcept
            : vbo(std::move(other.vbo)),
              num_points(other.num_points),
              color(other.color),
              opacity(other.opacity) {
            other.num_points = 0;
        }
        
        // 显式移动赋值运算符
        PointCloudVBOs& operator=(PointCloudVBOs&& other) noexcept {
            if (this != &other) {
                vbo = std::move(other.vbo);
                num_points = other.num_points;
                color = other.color;
                opacity = other.opacity;
                other.num_points = 0;
            }
            return *this;
        }
        
        // 删除复制操作
        PointCloudVBOs(const PointCloudVBOs&) = delete;
        PointCloudVBOs& operator=(const PointCloudVBOs&) = delete;
    };
    
    // 使用 std::unique_ptr 存储点云数据
    std::map<std::string, std::unique_ptr<PointCloudVBOs>> point_clouds_;

    // 延迟上传：add_point_cloud 时仅缓存 xyz，在 render_point_clouds 内（GL 上下文 current）再创建/上传 VBO，用 Pangolin RenderVbo 绘制避免崩溃
    struct PendingPointCloud {
        std::vector<float> data;  // 3 * num_points (x,y,z)
        size_t num_points = 0;
        Eigen::Vector3f color;
        float opacity = 0.5f;
    };
    std::map<std::string, PendingPointCloud> pending_uploads_;
    
    // 状态
    bool initialized_ = false;
    bool running_ = false;
    std::string title_;
    int width_, height_;
    int render_frame_count_ = 0;  // 总渲染帧数，用于日志与排查
    
    // 视口和相机 - View 在 Pangolin 0.9.0 中是 struct
    pangolin::OpenGlRenderState cam_state_;
    pangolin::View* view3d_ = nullptr;
    pangolin::Handler3D* handler3d_ = nullptr;
    
    // 外参
    Eigen::Matrix4d current_extrinsic_ = Eigen::Matrix4d::Identity();
    
    // 显示选项
    bool show_grid_ = true;
    bool show_axis_ = true;
    float point_size_ = 2.0f;
    
    // 相机数据
    std::optional<CameraFrustumData> camera_data_;
    std::vector<CoordinateFrameData> frames_;
};

// ===========================================================================
// 便捷函数
// ===========================================================================

/**
 * @brief 从点云创建OpenGL顶点buffer
 */
pangolin::GlVertexBuffer CreatePointCloudVBO(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

/**
 * @brief 绘制相机视锥体
 */
void DrawCameraFrustum(const Eigen::Matrix3d& K, int width, int height, 
                       const Eigen::Matrix4d& T, float scale, const Eigen::Vector3f& color);

/**
 * @brief 绘制坐标系轴
 */
void DrawCoordinateFrame(float scale);

/**
 * @brief 绘制网格
 */
void DrawGrid(float size, int divisions, const Eigen::Vector3f& color);

#endif  // UNICALIB_WITH_PANGOLIN

}  // namespace ns_unicalib
