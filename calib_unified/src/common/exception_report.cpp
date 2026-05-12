/**
 * UniCalib — 未处理异常报告实现
 * 供 UNICALIB_MAIN_TRY_END 宏使用，将异常信息输出到日志或 stderr，便于精确定位问题
 */
#include "unicalib/common/exception.h"
#include "unicalib/common/logger.h"
#include <iostream>
#include <typeinfo>

namespace ns_unicalib {

void reportUnhandledUniCalibException(const UniCalibException& e) {
    if (Logger::isInitialized()) {
        UNICALIB_CRITICAL("未处理的标定异常:\n{}", e.toString());
        UNICALIB_CRITICAL("  错误码: {} [{}] 分类: {}", errorCodeName(e.code()), e.codeValue(), e.category());
        Logger::flush();
    } else {
        std::cerr << "[UniCalib] 未处理的标定异常:\n" << e.toString() << "\n";
        std::cerr << "  错误码: " << errorCodeName(e.code()) << " [" << e.codeValue() << "] 分类: " << e.category() << "\n";
    }
}

void reportUnhandledStdException(const std::exception& e) {
    if (Logger::isInitialized()) {
        UNICALIB_CRITICAL("未处理的标准异常: {} (type={})", e.what(), typeid(e).name());
        Logger::flush();
    } else {
        std::cerr << "[UniCalib] 未处理的标准异常: " << e.what() << " (type=" << typeid(e).name() << ")\n";
    }
}

void reportUnhandledUnknownException() {
    if (Logger::isInitialized()) {
        UNICALIB_CRITICAL("未处理的未知异常 (非 std::exception 派生)");
        Logger::flush();
    } else {
        std::cerr << "[UniCalib] 未处理的未知异常 (非 std::exception)\n";
    }
}

}  // namespace ns_unicalib
