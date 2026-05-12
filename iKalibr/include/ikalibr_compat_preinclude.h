// Pre-include compatibility header for iKalibr build.
// This is force-included to fix missing includes in prebuilt ctraj headers (read-only).
#pragma once
#ifndef IKALIBR_COMPAT_PREINCLUDE_H
#define IKALIBR_COMPAT_PREINCLUDE_H

#include <fstream>
#include <sstream>
#include <string>

// Cereal archives (needed by ctraj/utils/utils.hpp which uses JSONOutputArchive
// without including cereal/archives/json.hpp)
#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>

#endif // IKALIBR_COMPAT_PREINCLUDE_H
