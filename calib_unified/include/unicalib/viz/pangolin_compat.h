/**
 * UniCalib — Pangolin 0.9.0 兼容层
 * 
 * 提供与新版 Pangolin API 的向后兼容实现
 * Pangolin 0.9.0 使用 GlBuffer，而非新版中的 GlVertexBuffer
 * 部分系统/ROS 自带 Pangolin 无 varextra.h，此处提供 Pushed(Var<bool>&)
 * 
 * 使用方式: 在包含 Pangolin 头文件之后，包含此兼容层
 */

#pragma once

#include <pangolin/gl/gl.h>
#include <pangolin/var/var.h>

namespace pangolin {

/** 兼容: 若系统 Pangolin 无 varextra.h 的 Pushed，则由此提供 */
inline bool Pushed(Var<bool>& button) {
    return static_cast<bool>(button) && !(button = false);
}

}

// 兼容常量 - Pangolin 0.9.0 中已有 GlBufferType 枚举，使用原生 OpenGL 常量
#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif

#ifndef GL_DYNAMIC_DRAW
#define GL_DYNAMIC_DRAW 0x88E8
#endif

// 在 pangolin 命名空间中定义兼容类型
namespace pangolin {

// 兼容类型定义 - 使用 GlBuffer 替代新版中的 GlVertexBuffer
// 在 Pangolin 0.9.0 中，GlBuffer 没有 Bind/Unbind 方法
// 需要使用原生 OpenGL 函数 glBindBuffer
struct GlVertexBuffer {
    GlBuffer vbo;
    
    GlVertexBuffer() = default;
    
    // 兼容较新版本 Pangolin 的 Reinitialise 签名
    // 参数: buffer_type, num_elements, usage
    void Reinitialise(GlBufferType buffer_type, 
                      GLuint num_elements,
                      GLenum usage = GL_DYNAMIC_DRAW) {
        vbo.Reinitialise(buffer_type, num_elements, GL_FLOAT, 4, usage);
    }
    
    // 兼容较新版本 Pangolin 的 Reinitialise 签名 (完整版)
    void Reinitialise(GlBufferType buffer_type, 
                      GLuint num_elements, 
                      GLenum count_per_element,
                      GLenum datatype,
                      GLenum gluse = GL_DYNAMIC_DRAW,
                      const void* data = nullptr) {
        vbo.Reinitialise(buffer_type, num_elements, datatype, count_per_element, gluse, data);
    }
    
    // Upload data
    void Upload(const void* data, size_t size_bytes) {
        vbo.Upload(data, size_bytes);
    }
    
    // 获取底层 GlBuffer 引用 (用于原生 OpenGL 调用)
    GlBuffer& buffer() { return vbo; }
    const GlBuffer& buffer() const { return vbo; }
    
    // 兼容: 获取内部 vbo (用于引用)
    GlBuffer& operator()() { return vbo; }
    const GlBuffer& operator()() const { return vbo; }
};

}  // namespace pangolin
