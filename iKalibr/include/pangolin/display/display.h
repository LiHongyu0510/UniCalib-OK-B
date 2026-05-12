// Stub for pangolin display functionality - replaces actual Pangolin GUI library
// iKalibr originally used Pangolin for visualization; since tiny-viewer has been removed,
// this stub allows the code to compile while disabling all GUI functionality.
#pragma once
#ifndef PANGOLIN_DISPLAY_DISPLAY_H
#define PANGOLIN_DISPLAY_DISPLAY_H

namespace pangolin {

// Signals all Pangolin windows to close (no-op in stub)
inline void QuitAll() {}

// Minimal window management stubs
inline void CreateWindowAndBind(const std::string& /*title*/, int /*w*/ = 640, int /*h*/ = 480) {}
inline void FinishFrame() {}
inline bool ShouldQuit() { return true; }

} // namespace pangolin

#endif // PANGOLIN_DISPLAY_DISPLAY_H
