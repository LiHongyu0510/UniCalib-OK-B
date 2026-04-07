// Minimal Pangolin fallback: provides pangolin::QuitAll() as no-op.
// This header is shadowed by the real Pangolin include path when
// UNICALIB_WITH_PANGOLIN=1 (pango_display target is linked).
// Only takes effect when Pangolin is not available.
#ifndef PANGOLIN_DISPLAY_DISPLAY_H
#define PANGOLIN_DISPLAY_DISPLAY_H

namespace pangolin {

inline void QuitAll() {}

inline bool ShouldQuit() { return false; }

inline void FinishFrame() {}

}  // namespace pangolin

#endif  // PANGOLIN_DISPLAY_DISPLAY_H
