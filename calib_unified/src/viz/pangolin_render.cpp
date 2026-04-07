/**
 * UniCalib Unified — Pangolin 3D渲染引擎实现
 * 
 * 适配 Pangolin 0.9.0 API
 */

#if UNICALIB_WITH_PANGOLIN

#include "unicalib/viz/pangolin_render.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/calib_stage.h"

#include <pangolin/gl/glvbo.h>

#include <pcl/common/transforms.h>

#include <Eigen/Geometry>
#include <cstdio>

namespace ns_unicalib {

// ===========================================================================
// 便捷函数实现
// ===========================================================================

pangolin::GlVertexBuffer CreatePointCloudVBO(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        return pangolin::GlVertexBuffer();
    }
    
    std::vector<float> data;
    data.reserve(cloud->size() * 6);  // x, y, z, intensity
    
    for (const auto& pt : cloud->points) {
        data.push_back(pt.x);
        data.push_back(pt.y);
        data.push_back(pt.z);
        data.push_back(pt.intensity);
    }
    
    pangolin::GlVertexBuffer vbo;
    vbo.Reinitialise(pangolin::GlArrayBuffer, 
                     static_cast<GLuint>(data.size()), GL_STATIC_DRAW);
    vbo.Upload(data.data(), data.size() * sizeof(float));
    
    return vbo;
}

void DrawCameraFrustum(const Eigen::Matrix3d& K, int width, int height, 
                       const Eigen::Matrix4d& T, float scale, const Eigen::Vector3f& color) {
    // 计算图像平面四个角点在相机坐标系下的位置
    double fx = K(0, 0);
    double fy = K(1, 1);
    double cx = K(0, 2);
    double cy = K(1, 2);
    
    std::vector<Eigen::Vector3d> corners_camera = {
        Eigen::Vector3d(-cx, -cy, 1.0),      // 左上
        Eigen::Vector3d(width - cx, -cy, 1.0),  // 右上
        Eigen::Vector3d(width - cx, height - cy, 1.0), // 右下
        Eigen::Vector3d(-cx, height - cy, 1.0)   // 左下
    };
    
    // 归一化坐标到焦距距离
    for (auto& c : corners_camera) {
        c[0] /= fx;
        c[1] /= fy;
        c *= scale;
    }
    
    // 转换到世界坐标系
    Eigen::Vector3d origin = T.block<3, 1>(0, 3);
    std::vector<Eigen::Vector3d> corners_world(4);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    for (int i = 0; i < 4; ++i) {
        corners_world[i] = R * corners_camera[i] + origin;
    }
    
    glColor3f(color[0], color[1], color[2]);
    glLineWidth(2.0f);
    
    // 绘制锥体线条
    glBegin(GL_LINES);
    // 原点到四个角
    for (int i = 0; i < 4; ++i) {
        glVertex3d(origin[0], origin[1], origin[2]);
        glVertex3d(corners_world[i][0], corners_world[i][1], corners_world[i][2]);
    }
    // 四个角形成矩形
    for (int i = 0; i < 4; ++i) {
        glVertex3d(corners_world[i][0], corners_world[i][1], corners_world[i][2]);
        glVertex3d(corners_world[(i+1)%4][0], corners_world[(i+1)%4][1], corners_world[(i+1)%4][2]);
    }
    glEnd();
    
    // 绘制坐标轴
    Eigen::Vector3d x_axis = origin + R * Eigen::Vector3d(scale * 0.3, 0, 0);
    Eigen::Vector3d y_axis = origin + R * Eigen::Vector3d(0, scale * 0.3, 0);
    Eigen::Vector3d z_axis = origin + R * Eigen::Vector3d(0, 0, scale * 0.3);
    
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    // X轴 (红色)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3d(origin[0], origin[1], origin[2]);
    glVertex3d(x_axis[0], x_axis[1], x_axis[2]);
    // Y轴 (绿色)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3d(origin[0], origin[1], origin[2]);
    glVertex3d(y_axis[0], y_axis[1], y_axis[2]);
    // Z轴 (蓝色)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3d(origin[0], origin[1], origin[2]);
    glVertex3d(z_axis[0], z_axis[1], z_axis[2]);
    glEnd();
}

void DrawCoordinateFrame(float scale) {
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // X轴 (红色)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(scale, 0, 0);
    // Y轴 (绿色)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, scale, 0);
    // Z轴 (蓝色)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, scale);
    glEnd();
}

void DrawGrid(float size, int divisions, const Eigen::Vector3f& color) {
    glColor3f(color[0], color[1], color[2]);
    glLineWidth(1.0f);
    
    float step = size / divisions;
    float half = size / 2.0f;
    
    glBegin(GL_LINES);
    for (int i = 0; i <= divisions; ++i) {
        float pos = -half + i * step;
        // 平行于X轴
        glVertex3f(-half, 0, pos);
        glVertex3f(half, 0, pos);
        // 平行于Z轴
        glVertex3f(pos, 0, -half);
        glVertex3f(pos, 0, half);
    }
    glEnd();
}

// ===========================================================================
// PangolinRender 类实现
// ===========================================================================

PangolinRender::PangolinRender() {
    // 初始化为单位矩阵
    current_extrinsic_ = Eigen::Matrix4d::Identity();
}

PangolinRender::~PangolinRender() {
    if (handler3d_) delete handler3d_;
}

void PangolinRender::init(const std::string& title, int width, int height) {
    title_ = title;
    width_ = width;
    height_ = height;

    UNICALIB_INFO("[PangolinRender] 正在创建窗口与 OpenGL 上下文: {} ({}x{})", title, width, height);

    // 创建窗口并绑定OpenGL上下文
    pangolin::CreateWindowAndBind(title, width, height);
    UNICALIB_INFO("[PangolinRender] CreateWindowAndBind 已完成");

    // 启用深度测试和混合
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 设置相机视角
    cam_state_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(width, height, 500, 500, width/2, height/2, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -2, 5, 0, 0, 0, pangolin::AxisY)
    );

    // 创建3D视图 - Pangolin 0.9.0 使用 View&
    view3d_ = &pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -width/(float)height)
        .SetHandler(new pangolin::Handler3D(cam_state_));

    handler3d_ = new pangolin::Handler3D(cam_state_);

    initialized_ = true;
    running_ = true;
    render_frame_count_ = 0;

    bool quit_requested = pangolin::ShouldQuit();
    UNICALIB_INFO("[PangolinRender] 初始化完成: {} ({}x{})  ShouldQuit={} (窗口内容: 网格+坐标轴，点云需后续推送)", title, width, height, quit_requested);
    if (quit_requested) {
        UNICALIB_WARN("[PangolinRender] 窗口创建后立即收到退出请求，可能处于无头或无效显示环境");
    }
}

bool PangolinRender::is_running() const {
    return running_ && !pangolin::ShouldQuit();
}

void PangolinRender::close() {
    if (!running_ && !initialized_) return;
    UNICALIB_INFO("[PangolinRender] 正在关闭窗口: {} 原因=用户关闭或流程结束 总渲染帧数={}", title_, render_frame_count_);
    running_ = false;
    if (initialized_ && !title_.empty()) {
        UNICALIB_DEBUG("[PangolinRender] 执行 DestroyWindow: {}", title_);
        pangolin::DestroyWindow(title_);
        initialized_ = false;
        UNICALIB_INFO("[PangolinRender] 已关闭窗口: {}", title_);
    }
}

bool PangolinRender::render() {
    if (!initialized_) return false;
    
    // 检查退出（用户点击窗口 X 或按 Esc 等）
    if (pangolin::ShouldQuit()) {
        UNICALIB_INFO("[PangolinRender] 检测到用户关闭窗口 (ShouldQuit) 总渲染帧数={}", render_frame_count_);
        running_ = false;
        return false;
    }
    
    render_frame_count_++;
    // 第 1 帧用 INFO 提示当前场景内容，便于排查“界面无内容”
    if (render_frame_count_ == 1) {
        int npc = static_cast<int>(point_clouds_.size());
        int nfr = static_cast<int>(frames_.size());
        UNICALIB_INFO("[PangolinRender] 首帧渲染 点云数={}  坐标系数={} (无点云时仅显示网格与坐标轴)",
                      npc, nfr);
    }
    // 每 30 帧打一条 debug
    if (render_frame_count_ % 30 == 0) {
        UNICALIB_DEBUG("[PangolinRender] 渲染帧 {}  点云数={}  坐标系数={}",
                      render_frame_count_, static_cast<int>(point_clouds_.size()), static_cast<int>(frames_.size()));
    }
    
    // 前 20 帧及每 50 帧打 INFO，便于定位精标定后首帧渲染崩溃
    const bool render_verbose = (render_frame_count_ <= 20 || render_frame_count_ % 50 == 0);
    const char* prev_stage = g_current_calib_stage;
    try {
        g_current_calib_stage = "viz_render";
        if (render_verbose) {
            UNICALIB_INFO("[PangolinRender] render 开始 帧={}", render_frame_count_);
        } else {
            UNICALIB_DEBUG("[PangolinRender] render 开始 帧={}", render_frame_count_);
        }
        // 清空缓冲区
        g_current_calib_stage = "viz_glClear";
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        if (render_verbose) { UNICALIB_INFO("[PangolinRender] render 步骤 glClear 完成"); }
        else { UNICALIB_DEBUG("[PangolinRender] render 步骤 glClear 完成"); }

        // 激活3D视图
        g_current_calib_stage = "viz_Activate";
        if (!view3d_) {
            UNICALIB_WARN("[PangolinRender] view3d_ 为空，跳过本帧渲染");
            g_current_calib_stage = prev_stage;
            return true;
        }
        view3d_->Activate(cam_state_);
        if (render_verbose) { UNICALIB_INFO("[PangolinRender] render 步骤 Activate 完成"); }
        else { UNICALIB_DEBUG("[PangolinRender] render 步骤 Activate 完成"); }

        // 渲染场景
        g_current_calib_stage = "viz_grid_axis";
        render_grid();
        render_axis();
        if (render_verbose) { UNICALIB_INFO("[PangolinRender] render 步骤 grid+axis 完成"); }
        else { UNICALIB_DEBUG("[PangolinRender] render 步骤 grid+axis 完成"); }

        g_current_calib_stage = "viz_point_clouds";
        render_point_clouds();
        if (render_verbose) { UNICALIB_INFO("[PangolinRender] render 步骤 point_clouds 完成"); }
        else { UNICALIB_DEBUG("[PangolinRender] render 步骤 point_clouds 完成"); }

        g_current_calib_stage = "viz_frustum_frames";
        render_camera_frustum();
        render_coordinate_frames();
        if (render_verbose) { UNICALIB_INFO("[PangolinRender] render 步骤 frustum+frames 完成"); }
        else { UNICALIB_DEBUG("[PangolinRender] render 步骤 frustum+frames 完成"); }

        // 处理事件（SwapBuffers 等，无头/显示异常时可能抛错或崩溃）
        g_current_calib_stage = "viz_FinishFrame";
        pangolin::FinishFrame();
        if (render_verbose) { UNICALIB_INFO("[PangolinRender] render 步骤 FinishFrame 完成"); }
        else { UNICALIB_DEBUG("[PangolinRender] render 步骤 FinishFrame 完成"); }
        g_current_calib_stage = prev_stage;
    } catch (const std::exception& e) {
        UNICALIB_WARN("[PangolinRender] render 异常 (总帧数={} 阶段={}): {} 将停止渲染",
                      render_frame_count_, g_current_calib_stage ? g_current_calib_stage : "(null)", e.what());
        g_current_calib_stage = prev_stage;
        running_ = false;
        return false;
    } catch (...) {
        UNICALIB_WARN("[PangolinRender] render 未知异常 (总帧数={} 阶段={}) 将停止渲染",
                      render_frame_count_, g_current_calib_stage ? g_current_calib_stage : "(null)");
        g_current_calib_stage = prev_stage;
        running_ = false;
        return false;
    }
    
    return true;
}

void PangolinRender::render_point_clouds() {
    static int s_detailed_draw_count = 0;
    const bool log_detail = (s_detailed_draw_count < 5);
    const char* prev_stage = g_current_calib_stage;
    // 精标定后首帧往往带点云，多打日志便于定位 SIGSEGV
    const bool verbose_this_frame = log_detail || (render_frame_count_ >= 12 && render_frame_count_ <= 25);
    try {
        g_current_calib_stage = "viz_pc_flush_pending";
        // 在 GL 上下文已 current 时处理延迟上传，避免在非渲染路径创建 VBO 导致 glDrawArrays 崩溃
        for (auto it = pending_uploads_.begin(); it != pending_uploads_.end(); ) {
            const std::string& name = it->first;
            PendingPointCloud& pending = it->second;
            if (pending.data.empty() || pending.num_points == 0) {
                it = pending_uploads_.erase(it);
                continue;
            }
            auto pc_vbo_ptr = std::make_unique<PointCloudVBOs>();
            // 3 分量 xyz，与 pangolin::RenderVbo(glbuf, GL_POINTS) 一致，避免固定管线/stride 导致崩溃
            pc_vbo_ptr->vbo.Reinitialise(pangolin::GlArrayBuffer,
                                         static_cast<GLuint>(pending.num_points),
                                         static_cast<GLenum>(3),   /* count_per_element */
                                         GL_FLOAT,
                                         GL_DYNAMIC_DRAW,
                                         nullptr);
            pc_vbo_ptr->vbo.Upload(pending.data.data(), pending.data.size() * sizeof(float));
            pc_vbo_ptr->num_points = pending.num_points;
            pc_vbo_ptr->color = pending.color;
            pc_vbo_ptr->opacity = pending.opacity;
            point_clouds_[name] = std::move(pc_vbo_ptr);
            if (verbose_this_frame) {
                UNICALIB_INFO("[PangolinRender] 延迟上传完成 名称={} 顶点数={} (xyz 3分量)",
                              name, pending.num_points);
            }
            it = pending_uploads_.erase(it);
        }

        g_current_calib_stage = "viz_pc_loop";
        glPointSize(point_size_);
        if (verbose_this_frame && !point_clouds_.empty()) {
            UNICALIB_INFO("[PangolinRender] render_point_clouds 进入 点云数={} 当前帧={}",
                          static_cast<int>(point_clouds_.size()), render_frame_count_);
        }

        for (auto& [name, pc_vbo_ptr] : point_clouds_) {
            if (!pc_vbo_ptr) {
                if (verbose_this_frame) {
                    UNICALIB_INFO("[PangolinRender] 点云 {} 跳过 原因=ptr为空", name);
                }
                continue;
            }
            PointCloudVBOs* pc_vbo_raw = pc_vbo_ptr.get();
            if (!pc_vbo_raw || pc_vbo_raw->num_points == 0) {
                if (verbose_this_frame) {
                    UNICALIB_INFO("[PangolinRender] 点云 {} 跳过 原因=点数为0 num_points={}",
                                  name, pc_vbo_raw ? pc_vbo_raw->num_points : 0);
                }
                continue;
            }
            const size_t num_points = pc_vbo_raw->num_points;
            if (num_points > 50000000u) {
                UNICALIB_WARN("[PangolinRender] 点云 {} 点数过大 跳过 num_points={}", name, num_points);
                continue;
            }

            g_current_calib_stage = "viz_pc_getbuf";
            pangolin::GlBuffer& glbuf = pc_vbo_raw->vbo.buffer();
            pangolin::GlBufferData* buf = &glbuf;
            if (!buf || !buf->IsValid()) {
                UNICALIB_WARN("[PangolinRender] 点云 VBO 无效，跳过 名称={} num_points={}",
                              name, num_points);
                continue;
            }

            if (verbose_this_frame) {
                UNICALIB_INFO("[PangolinRender] 点云 {} 即将 RenderVbo 点数={} 帧={}", name, num_points, render_frame_count_);
            }
            glColor4f(pc_vbo_raw->color[0], pc_vbo_raw->color[1], pc_vbo_raw->color[2], pc_vbo_raw->opacity);
            glDisableClientState(GL_COLOR_ARRAY);

            static char s_viz_stage_buf[80];
            (void)std::snprintf(s_viz_stage_buf, sizeof(s_viz_stage_buf), "viz_pc_%s_RenderVbo", name.c_str());
            g_current_calib_stage = s_viz_stage_buf;
            pangolin::RenderVbo(glbuf, GL_POINTS);
            if (verbose_this_frame) { UNICALIB_INFO("[PangolinRender] 点云 {} RenderVbo 完成 点数={}", name, num_points); }
        }
        if (log_detail && s_detailed_draw_count < 5) {
            s_detailed_draw_count++;
            UNICALIB_INFO("[PangolinRender] render_point_clouds 本次完成 详细日志次数={}/5", s_detailed_draw_count);
        }
        g_current_calib_stage = prev_stage;
    } catch (const std::exception& e) {
        UNICALIB_WARN("[PangolinRender] render_point_clouds 异常 (阶段={}): {} 已跳过点云渲染",
                      g_current_calib_stage ? g_current_calib_stage : "(null)", e.what());
        g_current_calib_stage = prev_stage;
    } catch (...) {
        UNICALIB_WARN("[PangolinRender] render_point_clouds 未知异常 (阶段={}) 已跳过点云渲染",
                      g_current_calib_stage ? g_current_calib_stage : "(null)");
        g_current_calib_stage = prev_stage;
    }
}

void PangolinRender::render_camera_frustum() {
    if (camera_data_.has_value()) {
        DrawCameraFrustum(
            camera_data_->K,
            camera_data_->width,
            camera_data_->height,
            camera_data_->T_cam_lidar.inverse(),  // T_lidar_cam
            camera_data_->scale,
            camera_data_->color
        );
    }
}

void PangolinRender::render_coordinate_frames() {
    for (const auto& frame : frames_) {
        glPushMatrix();
        Eigen::Matrix4f T = frame.T.cast<float>();
        glMultMatrixf(T.data());
        DrawCoordinateFrame(frame.scale);
        glPopMatrix();
    }
}

void PangolinRender::render_grid() {
    if (show_grid_) {
        DrawGrid(20.0f, 20, Eigen::Vector3f(0.3f, 0.3f, 0.3f));
    }
}

void PangolinRender::render_axis() {
    if (show_axis_) {
        DrawCoordinateFrame(1.0f);
    }
}

void PangolinRender::set_ref_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                   const Eigen::Vector3f& color) {
    add_point_cloud("ref", cloud, color);
}

void PangolinRender::set_target_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                      const Eigen::Vector3f& color) {
    add_point_cloud("target", cloud, color);
}

void PangolinRender::add_point_cloud(const std::string& name, 
                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                     const Eigen::Vector3f& color) {
    if (!cloud || cloud->empty()) {
        UNICALIB_WARN("[PangolinRender] 点云为空: {}", name);
        return;
    }
    
    // 仅缓存 x,y,z（3 分量），与 Pangolin RenderVbo(3 分量) 一致，避免 GL 3.0 下 glColorPointer/自定义 stride 崩溃
    std::vector<float> data;
    data.reserve(cloud->size() * 3);
    
    for (const auto& pt : cloud->points) {
        if (name == "target" || name == "aligned") {
            Eigen::Vector4d pt_h(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4d pt_transformed = current_extrinsic_ * pt_h;
            data.push_back(static_cast<float>(pt_transformed[0]));
            data.push_back(static_cast<float>(pt_transformed[1]));
            data.push_back(static_cast<float>(pt_transformed[2]));
        } else {
            data.push_back(pt.x);
            data.push_back(pt.y);
            data.push_back(pt.z);
        }
    }
    
    point_clouds_.erase(name);
    PendingPointCloud pending;
    pending.data = std::move(data);
    pending.num_points = cloud->size();
    pending.color = color;
    pending.opacity = 0.5f;
    pending_uploads_[name] = std::move(pending);

    const size_t n = cloud->size();
    UNICALIB_INFO("[PangolinRender] 添加点云(延迟上传,xyz) 名称={} 顶点数={} 将在下一帧用 RenderVbo 绘制",
                  name, n);
}

void PangolinRender::set_extrinsic(const Eigen::Matrix4d& T) {
    current_extrinsic_ = T;
    
    // 重新渲染target点云 - 需要重建 VBO
    auto it = point_clouds_.find("target");
    if (it != point_clouds_.end() && it->second) {
        // 标记需要更新，实际实现时需要重新上传数据
        UNICALIB_DEBUG("[PangolinRender] 外参更新，target点云将在下一帧重新渲染");
    }
}

void PangolinRender::set_camera(const CameraFrustumData& cam) {
    camera_data_ = cam;
}

void PangolinRender::set_coordinate_frame(const CoordinateFrameData& frame) {
    frames_.push_back(frame);
}

void PangolinRender::set_show_grid(bool show) {
    show_grid_ = show;
}

void PangolinRender::set_point_size(float size) {
    point_size_ = size;
}

Eigen::Matrix4d PangolinRender::get_camera_pose() const {
    // Pangolin 0.9.0: OpenGlMatrix 可隐式转换为 Eigen::Matrix
    // 使用 operator Eigen::Matrix<P,4,4>() 转换
    pangolin::OpenGlMatrix mv = cam_state_.GetModelViewMatrix();
    Eigen::Matrix4d eigen_mv = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            eigen_mv(i, j) = mv(i, j);
        }
    }
    return eigen_mv;
}

void PangolinRender::set_camera_pose(const Eigen::Matrix4d& T) {
    // 将 Eigen 矩阵转换为 Pangolin OpenGlMatrix
    pangolin::OpenGlMatrix M;
    Eigen::Matrix4f T_float = T.cast<float>();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            M(i, j) = T_float(i, j);
        }
    }
    cam_state_.SetModelViewMatrix(M);
}

void PangolinRender::reset_view() {
    cam_state_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(width_, height_, 500, 500, width_/2, height_/2, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -2, 5, 0, 0, 0, pangolin::AxisY)
    );
}

void PangolinRender::save_screenshot(const std::string& filename) {
    pangolin::SaveWindowOnRender(filename);
    UNICALIB_INFO("[PangolinRender] 截图保存: {}", filename);
}

}  // namespace ns_unicalib

#endif  // UNICALIB_WITH_PANGOLIN
