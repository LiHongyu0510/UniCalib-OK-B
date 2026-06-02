#ifdef ENABLE_CYBER

#include "modules/record/record_component.h"
#include "unicalib/common/logger.h"

namespace ns_unicalib {
namespace modules {

bool RecordComponent::Init() {
    UNICALIB_INFO("[RecordComponent] 初始化完成（Stage 3 占位实现）");
    // TODO: 创建 Cyber Recorder，订阅关键 topic
    return true;
}

bool RecordComponent::Proc() {
    return true;
}

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER