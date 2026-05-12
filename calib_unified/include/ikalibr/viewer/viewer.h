// iKalibr: Viewer interface. Pangolin replaces tiny-viewer when available.
#ifndef IKALIBR_VIEWER_H
#define IKALIBR_VIEWER_H

#if defined(UNICALIB_WITH_PANGOLIN) && !defined(IKALIBR_VIEWER_DISABLED)
#  include "viewer/viewer_pangolin.h"
#else
#  include "viewer/viewer_stub.h"
#endif

#endif  // IKALIBR_VIEWER_H
