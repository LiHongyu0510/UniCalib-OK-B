#ifdef ENABLE_CYBER

#pragma once
#include "cyber/component/component.h"

namespace ns_unicalib {
namespace modules {

class LidarComponent : public apollo::cyber::Component<> {
public:
    bool Init() override;
    bool Proc() override;
};

DECLARE_COMPONENT(LidarComponent);

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER