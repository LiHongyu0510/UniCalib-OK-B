#ifdef ENABLE_CYBER
#pragma once
#include "cyber/component/component.h"
#include "unicalib/pipeline/calib_pipeline.h"

namespace ns_unicalib {
namespace modules {

class CalibComponent : public apollo::cyber::Component<> {
public:
    bool Init() override;
    bool Proc() override;
private:
    std::unique_ptr<CalibPipeline> pipeline_;
    std::shared_ptr<apollo::cyber::Writer<unicalib::cyber::CalibStatus>> status_writer_;
};
DECLARE_COMPONENT(CalibComponent);

} // namespace modules
} // namespace ns_unicalib
#endif // ENABLE_CYBER