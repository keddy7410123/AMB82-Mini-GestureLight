#pragma once
#include "Config.h"

// No Arduino dependencies: deterministic gesture timing for replay tests.
// candidate: -1 invalid/ambiguous, 0 OPEN, 1 CLOSE.
class GestureGate {
public:
    bool ready = false; // require an observed empty ROI after boot/arm/stale input
    int candidate = -1;
    uint32_t holdStart = 0, heldMs = 0, lastFrame = 0, lastFire = 0;
    unsigned samples = 0;
    bool seen = false, fired = false, releasing = false;
    uint32_t releaseStart = 0;

    void reset() { *this = GestureGate(); }
    void invalidate() {
        ready = false; candidate = -1; samples = 0; heldMs = 0;
        releasing = false; seen = false;
    }
    int update(uint32_t now, int next, bool empty, bool enabled) {
        if (!enabled) { invalidate(); return -1; }
        if (seen && uint32_t(now - lastFrame) > cfg::MAX_FRAME_GAP_MS) invalidate();
        seen = true; lastFrame = now;
        if (!ready) {
            candidate = -1; samples = 0; heldMs = 0;
            if (!empty) { releasing = false; return -1; }
            if (!releasing) { releasing = true; releaseStart = now; }
            if (uint32_t(now - releaseStart) >= cfg::RELEASE_MS &&
                (!fired || uint32_t(now - lastFire) >= cfg::COOLDOWN_MS)) {
                ready = true; releasing = false;
            }
            return -1;
        }
        if (next < 0) { candidate = -1; samples = 0; heldMs = 0; return -1; }
        if (candidate != next) { candidate = next; holdStart = now; samples = 0; }
        ++samples;
        heldMs = uint32_t(now - holdStart);
        if (heldMs < cfg::HOLD_MS || samples < cfg::MIN_SAMPLES) return -1;
        int result = candidate;
        ready = false; candidate = -1; samples = 0; heldMs = 0;
        releasing = false; fired = true; lastFire = now;
        return result;
    }
};
