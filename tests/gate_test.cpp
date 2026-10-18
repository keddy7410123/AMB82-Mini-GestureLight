#include "../GestureLight/GestureGate.h"
#include "../GestureLight/DetectionPolicy.h"
#include <assert.h>
#include <stdio.h>

static void clearROI(GestureGate &g, uint32_t t) {
    g.update(t, -1, true, true);
    g.update(t+250, -1, true, true);
    g.update(t+500, -1, true, true);
    assert(g.ready);
}
static int hold(GestureGate &g, uint32_t t, int c) {
    for (unsigned i=0;i<4;++i) assert(g.update(t+i*250,c,false,true)==-1);
    return g.update(t+1000,c,false,true);
}
int main() {
    Frame frame; bool empty=false;
    assert(classify(frame,empty)==-1 && empty);
    frame.count=1;frame.boxes[0]={4,95,.3f,.3f,.6f,.7f};
    assert(classify(frame,empty)==0 && !empty);
    frame.boxes[0].id=0;assert(classify(frame,empty)==1);
    for(int id=1;id<=3;++id){frame.boxes[0].id=id;assert(classify(frame,empty)==-1 && !empty);}
    frame.boxes[0].id=4;frame.boxes[0].score=74;
    assert(classify(frame,empty)==-1 && !empty);
    frame.boxes[0].score=95;frame.boxes[0].l=.1f; // straddles ROI
    assert(classify(frame,empty)==-1 && !empty);
    frame.boxes[0]={4,95,0,0,.1f,.1f}; // fully outside ROI
    assert(classify(frame,empty)==-1 && empty);
    frame.count=2;frame.boxes[0]={4,95,.3f,.3f,.6f,.7f};frame.boxes[1]={0,99,.4f,.4f,.7f,.8f};
    assert(classify(frame,empty)==-1 && !empty);
    frame.count=1;frame.overflow=true;
    assert(classify(frame,empty)==-1 && !empty);
    frame.overflow=false;frame.boxes[0].r=-.1f; // malformed
    assert(classify(frame,empty)==-1 && !empty);
    GestureGate g;
    assert(hold(g,0,0)==-1); // boot with hand present: no command
    clearROI(g,1250);
    assert(hold(g,2000,0)==0);
    for (uint32_t t=3250;t<=8250;t+=250) assert(g.update(t,0,false,true)==-1);
    // A low-confidence/unknown hand must NOT release the latch.
    for (uint32_t t=8500;t<=9500;t+=250) g.update(t,-1,false,true);
    assert(!g.ready);
    clearROI(g,9750);
    assert(hold(g,10500,1)==1);
    // Release alone cannot bypass cooldown.
    g.update(11750,-1,true,true);g.update(12000,-1,true,true);g.update(12250,-1,true,true);
    assert(!g.ready);
    g.update(12750,-1,true,true);g.update(13500,-1,true,true);assert(g.ready);
    // Invalid detection interrupts continuity.
    g.update(13750,0,false,true);g.update(14000,0,false,true);
    g.update(14250,-1,false,true);assert(g.heldMs==0);
    assert(hold(g,14500,1)==1);
    // A long gap resets even if the next frame has the same gesture.
    g.reset();clearROI(g,0);
    g.update(750,0,false,true);g.update(1000,0,false,true);
    assert(g.update(2500,0,false,true)==-1);assert(!g.ready);
    // Repeated calls at the same timestamp cannot complete a one-second hold.
    g.reset();clearROI(g,0);
    for(unsigned i=0;i<500;++i)assert(g.update(750,0,false,true)==-1);
    // Disable and stale invalidate require a new empty interval.
    g.update(1000,0,false,false);assert(!g.ready);
    assert(hold(g,1250,0)==-1);
    g.invalidate();assert(!g.ready);
    // Unsigned time arithmetic survives millis() rollover.
    g.reset();uint32_t t=0xfffffe00U;clearROI(g,t);
    assert(hold(g,t+750,0)==0);
    puts("PASS: class 4 OPEN / class 0 CLOSE, ignored classes 1..3, ROI boundaries, ambiguity, confidence, malformed/overflow boxes, boot latch, sustained-hand latch, cooldown, interrupted hold, frame gap, duplicate timestamp, disable, rollover");
}
