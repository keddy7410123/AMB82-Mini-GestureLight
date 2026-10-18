#pragma once
#include "Config.h"

constexpr unsigned BOX_CAP = 14;
struct Box { int id, score; float l, t, r, b; };
struct Frame {
    uint32_t seq = 0, ms = 0;
    unsigned count = 0;
    bool overflow = false;
    Box boxes[BOX_CAP];
};

inline bool targetClass(int id) {
    return id == cfg::OPEN_CLASS || id == cfg::CLOSE_CLASS;
}
inline bool insideRoi(const Box &b) {
    // Reject invalid/NaN coordinates and boxes crossing any ROI boundary.
    return b.l >= cfg::ROI_L && b.r <= cfg::ROI_R &&
           b.t >= cfg::ROI_T && b.b <= cfg::ROI_B && b.l < b.r && b.t < b.b;
}
inline bool intersects(const Box &b) {
    return b.r > cfg::ROI_L && b.l < cfg::ROI_R && b.b > cfg::ROI_T && b.t < cfg::ROI_B;
}
inline int classify(const Frame &f, bool &empty) {
    unsigned inside = 0; int selected = -1;
    bool malformed = false;
    for (unsigned i = 0; i < f.count; ++i) {
        const Box &b = f.boxes[i];
        // Comparisons also reject NaN coordinates.
        if (!(b.l >= 0 && b.t >= 0 && b.r <= 1 && b.b <= 1 && b.l < b.r && b.t < b.b)) {
            malformed = true; continue;
        }
        if (!intersects(b)) continue;
        ++inside;
        bool fits = insideRoi(b);
        if (fits && b.score >= cfg::CONFIDENCE) {
            if (b.id == cfg::OPEN_CLASS) selected = 0;
            if (b.id == cfg::CLOSE_CLASS) selected = 1;
        }
    }
    empty = inside == 0 && !f.overflow && !malformed;
    return inside == 1 && !f.overflow && !malformed ? selected : -1;
}

