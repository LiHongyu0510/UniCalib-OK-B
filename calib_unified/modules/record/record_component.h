#ifdef ENABLE_CYBER

#pragma once

#include "cyber/component/component.h"
#include "cyber/cyber.h"

namespace ns_unicalib {
namespace modules {

/**
 * RecordComponent
 * 负责录制标定过程数据（Cyber Record）
 */
class RecordComponent : public apollo::cyber::Component<> {
public:
    bool Init() override;
    bool Proc() override;
};

DECLARE_COMPONENT(RecordComponent);

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER