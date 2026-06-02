#ifdef ENABLE_CYBER
#pragma once
#include "cyber/component/component.h"
namespace ns_unicalib { namespace modules {
class ImuComponent : public apollo::cyber::Component<> {
public:
    bool Init() override { return true; }
    bool Proc() override { return true; }
};
DECLARE_COMPONENT(ImuComponent);
} }
#endif // ENABLE_CYBER