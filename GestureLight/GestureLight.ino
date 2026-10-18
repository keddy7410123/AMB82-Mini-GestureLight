#include <Arduino.h>
#include <WiFi.h>
#include "VideoStream.h"
#include "StreamIO.h"
#include "NNObjectDetection.h"
#include <AmebaServo.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Config.h"
#include "GestureGate.h"
#include "DetectionPolicy.h"
#include "WebPage.h"
#include "lwip/sockets.h"
#include "lwip/errno.h"
// Avoid POSIX compatibility macros rewriting Arduino Stream methods.
#undef read
#undef write
#undef close


constexpr unsigned LOG_CAP = 12;
struct Event { uint32_t ms; const char *text; };
struct Shared {
    Frame frame;
    bool armed = false, busy = false, ready = false;
    int candidate = -1, lastCommand = -1, duplicateCommand = -1, neutral = cfg::STOP_US;
    uint32_t held = 0, commands = 0, eventCount = 0;
    Event events[LOG_CAP];
} shared;
SemaphoreHandle_t mutex;
VideoSetting preview(cfg::NN_WIDTH, cfg::NN_HEIGHT, 10, VIDEO_JPEG, 1);
VideoSetting nnVideo(cfg::NN_WIDTH, cfg::NN_HEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection detector;
StreamIO nnLink(1, 1);
AmebaServo motor;
// Use native lwIP send counts: SDK 4.1.0 WiFiClient.write() loses them.
int httpListenFd = -1;

// All shared fields are accessed under mutex; never hold it while sending HTTP.
void logEvent(uint32_t now, const char *text) {
    shared.events[shared.eventCount % LOG_CAP] = {now, text};
    ++shared.eventCount;
}
void detectionCallback(std::vector<ObjectDetectionResult> results) {
    Frame f;
    f.ms = millis();
    for (ObjectDetectionResult item : results) {
        // SDK ROI mode already maps crop results back to full-frame coordinates.
        Box box = {item.type(), item.score(), item.xMin(), item.yMin(), item.xMax(), item.yMax()};
        if (!insideRoi(box)) continue;
        // Keep other classes internally to reject ambiguous/multiple gestures.
        // Only target classes are published to the web API.
        if (f.count == BOX_CAP) { f.overflow = true; continue; }
        f.boxes[f.count++] = box;
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    f.seq = shared.frame.seq + 1;
    shared.frame = f;
    xSemaphoreGive(mutex);
}
// Motor/control task owns PWM. No socket or camera capture calls in this task.
void controlTask(void *) {
    GestureGate gate;
    uint32_t processed = 0, phaseStart = 0;
    int phase = 0, action = -1; // 0 idle, 1 gesture stroke, 4 manual jog
    uint32_t duration = 0;
    bool recognitionLedOn = false;
    uint32_t recognitionLedStart = 0;
    for (;;) {
        uint32_t now = millis();
        xSemaphoreTake(mutex, portMAX_DELAY);
        // Service motor deadlines BEFORE processing commands or inference.
        if (phase && uint32_t(now - phaseStart) >= duration) {
            motor.writeMicroseconds(shared.neutral);
            if (phase == 1) logEvent(now, "stroke_finished_unverified");
            phase = 0;
            phaseStart = now;
        }
        // Non-blocking indication: keep motor deadlines and inference running.
        if (recognitionLedOn && uint32_t(now - recognitionLedStart) >= cfg::RECOGNITION_LED_MS) {
            digitalWrite(LED_B, LOW);
            recognitionLedOn = false;
        }
        bool fresh = shared.frame.seq && uint32_t(now - shared.frame.ms) <= cfg::STALE_MS;
        bool wifi = WiFi.status() == WL_CONNECTED;
        if (!fresh || !wifi) {
            gate.invalidate();
            if (shared.armed) {
                shared.armed = false;
                motor.writeMicroseconds(shared.neutral); phase = 0;
                logEvent(now, wifi ? "inference_stale_disarmed" : "wifi_lost_disarmed");
            }
        }
        int requested = -1;
        // One byte per iteration avoids serial floods delaying the motor deadline.
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'x' || c == 'd' || c == 'n') {
                shared.armed = false; phase = 0; gate.reset();
                motor.writeMicroseconds(shared.neutral);
                logEvent(now, "stopped_disarmed");
            } else if (c == 'a' && !phase && fresh && wifi) {
                if (cfg::MOTOR_CALIBRATED) {
                    shared.armed = true; gate.reset(); processed = shared.frame.seq;
                    logEvent(now, "armed_remove_hand_first");
                } else logEvent(now, "arm_rejected_calibrate_Config_first");
            } else if (!shared.armed && !phase && (c == '+' || c == '-')) {
                int next = shared.neutral + (c == '+' ? 5 : -5);
                if (next >= 1400 && next <= 1600) shared.neutral = next;
                motor.writeMicroseconds(shared.neutral);
                logEvent(now, "neutral_adjusted_save_to_Config");
            } else if (!shared.armed && !phase && (c == 'j' || c == 'k')) {
                phase = 4; phaseStart = now; duration = cfg::JOG_MS;
                motor.writeMicroseconds(c == 'j' ? cfg::OPEN_US : cfg::CLOSE_US);
                logEvent(now, c == 'j' ? "jog_open_direction" : "jog_close_direction");
            } else if (!shared.armed && !phase && cfg::MOTOR_CALIBRATED && (c == 'o' || c == 'f')) {
                requested = c == 'o' ? 0 : 1;
            }
        }
        if (shared.frame.seq != processed) {
            processed = shared.frame.seq;
            bool empty = false;
            int candidate = classify(shared.frame, empty);
            int result = gate.update(shared.frame.ms, candidate, empty, shared.armed && fresh && !phase);
            if (result >= 0) {
                requested = result;
                // Confirmed gestures flash even when a duplicate is ignored.
                // Serial/manual commands do not count as recognition.
                digitalWrite(LED_B, HIGH);
                recognitionLedOn = true;
                recognitionLedStart = now;
            }
        }
        if (requested >= 0 && !phase) {
            // Preserve lastCommand across stop/re-arm and Wi-Fi/inference loss.
            // -1 at boot means unknown: either first gesture is accepted.
            if (requested == shared.lastCommand) {
                shared.duplicateCommand = requested;
                logEvent(now, requested == 0 ? "already_open_ignored" : "already_closed_ignored");
                Serial.println(requested == 0 ? "Already ON: duplicate OPEN ignored" : "Already OFF: duplicate CLOSE ignored");
            } else {
                shared.duplicateCommand = -1;
                action = requested; phase = 1; phaseStart = now;
                duration = action == 0 ? cfg::OPEN_MS : cfg::CLOSE_MS;
                motor.writeMicroseconds(action == 0 ? cfg::OPEN_US : cfg::CLOSE_US);
                shared.lastCommand = action; ++shared.commands;
                logEvent(now, action == 0 ? "OPEN_command_sent_unverified" : "CLOSE_command_sent_unverified");
            }
        }
        shared.busy = phase != 0; shared.ready = gate.ready;
        shared.candidate = gate.candidate; shared.held = gate.heldMs;
        xSemaphoreGive(mutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Native lwIP returns real byte counts, including partial writes and EAGAIN.
// WiFiClient/ServerDrv in SDK 4.1.0 converts send() to bool, so it cannot be
// used with byte-offset retry logic. Keep all socket operations in HTTP task.
bool sendAll(int socketFd, const uint8_t *data, size_t length) {
    size_t offset = 0; uint32_t start = millis();
    while (offset < length && uint32_t(millis() - start) < 1800) {
        size_t chunk = length - offset;
        if (chunk > 1460) chunk = 1460;
        int sent = lwip_send(socketFd, data + offset, chunk, 0);
        if (sent > 0) offset += (size_t)sent;
        else if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return offset == length;
}
void respond(int socketFd, const char *type, const uint8_t *body, size_t length, int code = 200) {
    char h[320];
    snprintf(h, sizeof(h), "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %lu\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nConnection: close\r\n\r\n", code, code == 200 ? "OK" : "Error", type, (unsigned long)length);
    if (sendAll(socketFd, (const uint8_t *)h, strlen(h))) sendAll(socketFd, body, length);
}
int beginHttpListener() {
    int fd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return -1;
    int enabled = 1;
    lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
    struct sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(80);
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    if (lwip_bind(fd, (struct sockaddr *)&address, sizeof(address)) < 0 ||
        lwip_listen(fd, 2) < 0 || lwip_ioctl(fd, FIONBIO, &enabled) < 0) {
        lwip_close(fd); return -1;
    }
    return fd;
}
String statusJson() {
    Shared s;
    xSemaphoreTake(mutex, portMAX_DELAY); s = shared; xSemaphoreGive(mutex);
    uint32_t now = millis();
    String j; j.reserve(6000);
    j += "{\"uptime_ms\":"; j += now;
    j += ",\"seq\":"; j += s.frame.seq;
    j += ",\"age_ms\":"; j += uint32_t(now - s.frame.ms);
    j += ",\"stale\":"; j += (!s.frame.seq || uint32_t(now - s.frame.ms) > cfg::STALE_MS) ? "true" : "false";
    j += ",\"armed\":"; j += s.armed ? "true" : "false";
    j += ",\"busy\":"; j += s.busy ? "true" : "false";
    j += ",\"ready\":"; j += s.ready ? "true" : "false";
    j += ",\"calibrated\":"; j += cfg::MOTOR_CALIBRATED ? "true" : "false";
    j += ",\"neutral_us\":"; j += s.neutral;
    j += ",\"candidate\":"; j += s.candidate;
    j += ",\"held_ms\":"; j += s.held;
    j += ",\"hold_ms\":"; j += cfg::HOLD_MS;
    j += ",\"last_command\":"; j += s.lastCommand;
    j += ",\"duplicate_command\":"; j += s.duplicateCommand;
    j += ",\"confidence_threshold\":"; j += cfg::CONFIDENCE;
    j += ",\"commands\":"; j += s.commands;
    j += ",\"open_class\":"; j += cfg::OPEN_CLASS;
    j += ",\"close_class\":"; j += cfg::CLOSE_CLASS;
    j += ",\"roi\":["; j += String(cfg::ROI_L, 6); j += ','; j += String(cfg::ROI_T, 6);
    j += ','; j += String(cfg::ROI_R, 6); j += ','; j += String(cfg::ROI_B, 6);
    j += "],\"boxes\":[";
    unsigned published = 0;
    for (unsigned i = 0; i < s.frame.count; ++i) {
        const Box &b = s.frame.boxes[i];
        if (!targetClass(b.id) || !insideRoi(b)) continue;
        if (published++) j += ',';
        bool valid = b.l >= 0 && b.t >= 0 && b.r <= 1 && b.b <= 1 && b.l < b.r && b.t < b.b;
        j += "{\"id\":"; j += b.id; j += ",\"score\":"; j += b.score;
        j += ",\"box\":[";
        j += String(valid ? b.l : 0, 6); j += ','; j += String(valid ? b.t : 0, 6);
        j += ','; j += String(valid ? b.r : 0, 6); j += ','; j += String(valid ? b.b : 0, 6); j += "]}";
    }
    j += "],\"events\":[";
    uint32_t count = s.eventCount < LOG_CAP ? s.eventCount : LOG_CAP;
    for (uint32_t i = 0; i < count; ++i) {
        const Event &e = s.events[(s.eventCount - 1 - i) % LOG_CAP];
        if (i) j += ',';
        j += "{\"ms\":"; j += e.ms; j += ",\"text\":\""; j += e.text; j += "\"}";
    }
    j += "]}";
    return j;
}
void httpTask(void *) {
    uint32_t retry = 0, lastAddressPrint = 0;
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            if (httpListenFd >= 0) { lwip_close(httpListenFd); httpListenFd = -1; }
            if (!retry || uint32_t(millis() - retry) >= 15000) {
                retry = millis(); WiFi.begin(cfg::SSID, cfg::PASSWORD);
            }
            vTaskDelay(pdMS_TO_TICKS(100)); continue;
        }
        IPAddress localAddress = WiFi.localIP();
        if (!localAddress[0]) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        if (httpListenFd < 0) {
            httpListenFd = beginHttpListener();
            if (httpListenFd < 0) {
                Serial.println("HTTP listener failed; retrying");
                vTaskDelay(pdMS_TO_TICKS(1000)); continue;
            }
            Serial.print("Monitor: http://"); Serial.println(localAddress);
            lastAddressPrint = millis();
        }
        // Repeat address so opening Serial Monitor after boot still finds it.
        if (uint32_t(millis() - lastAddressPrint) >= 30000) {
            Serial.print("Monitor: http://"); Serial.println(localAddress);
            lastAddressPrint = millis();
        }
        int c = lwip_accept(httpListenFd, NULL, NULL);
        if (c < 0) {
            // EAGAIN means no pending browser, not a server failure.
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                lwip_close(httpListenFd); httpListenFd = -1;
            }
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }
        int enabled = 1;
        if (lwip_ioctl(c, FIONBIO, &enabled) < 0) { lwip_close(c); continue; }
        char request[1025] = {0}; size_t used = 0;
        uint32_t begin = millis(); bool complete = false;
        while (used < sizeof(request) - 1 && uint32_t(millis() - begin) < 800) {
            int count = lwip_recv(c, request + used, sizeof(request) - 1 - used, 0);
            if (count > 0) {
                used += (size_t)count; request[used] = 0;
                if (strstr(request, "\r\n\r\n")) { complete = true; break; }
            } else if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) break;
            else vTaskDelay(pdMS_TO_TICKS(1));
        }
        char method[8] = {0}, path[96] = {0};
        if (complete && sscanf(request, "%7s %95s", method, path) == 2 && !strcmp(method, "GET")) {
            char *query = strchr(path, '?'); if (query) *query = 0;
            if (!strcmp(path, "/")) respond(c, "text/html; charset=utf-8", (const uint8_t *)WEB_PAGE, strlen(WEB_PAGE));
            else if (!strcmp(path, "/api/status")) {
                String j = statusJson(); respond(c, "application/json", (const uint8_t *)j.c_str(), j.length());
            } else if (!strcmp(path, "/frame.jpg")) {
                uint32_t address = 0, length = 0;
                Camera.getImage(0, &address, &length);
                if (address && length) respond(c, "image/jpeg", (const uint8_t *)address, length);
                else respond(c, "text/plain", (const uint8_t *)"No frame", 8, 503);
            } else respond(c, "text/plain", (const uint8_t *)"Not found", 9, 404);
        } else respond(c, "text/plain", (const uint8_t *)"Bad request", 11, 400);
        lwip_close(c);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
void fatal(const char *message) {
    motor.writeMicroseconds(cfg::STOP_US);
    Serial.println(message);
    for (;;) delay(1000);
}
void setup() {
    pinMode(LED_B, OUTPUT);
    digitalWrite(LED_B, LOW);
    Serial.begin(115200);
    motor.attach(cfg::SERVO_PIN, 1000, 2000);
    motor.writeMicroseconds(cfg::STOP_US);
    mutex = xSemaphoreCreateMutex();
    if (!mutex) fatal("Mutex allocation failed");
    Camera.configVideoChannel(0, preview);
    Camera.configVideoChannel(3, nnVideo);
    Camera.videoInit();
    detector.configVideo(nnVideo);
    // Configure AFTER configVideo (which resets ROI) and BEFORE begin.
    detector.configRegionOfInterest(cfg::ROI_XMIN, cfg::ROI_XMAX, cfg::ROI_YMIN, cfg::ROI_YMAX);
    Serial.println("GESTURE_ROI280 input_crop=116,48,460,272 classes=4,0 stroke_ms=280 return_ms=0");
    detector.configThreshold(cfg::DETECT_THRESHOLD, 0.45f);
    detector.setResultCallback(detectionCallback);
    detector.modelSelect(OBJECT_DETECTION, CUSTOMIZED_YOLOV7TINY, NA_MODEL, NA_MODEL);
    detector.begin();
    nnLink.registerInput(Camera.getStream(3));
    nnLink.registerOutput(detector);
    nnLink.setStackSize(); nnLink.setTaskPriority();
    if (nnLink.begin() != 0) fatal("NN StreamIO failed");
    Camera.channelBegin(0); Camera.channelBegin(3);
    if (xTaskCreate(controlTask, "motor", 4096, NULL, 3, NULL) != pdPASS) fatal("Motor task failed");
    if (xTaskCreate(httpTask, "monitor", 12288, NULL, 1, NULL) != pdPASS) fatal("HTTP task failed");
    Serial.println("Paused. j/k:280ms jog; +/-:neutral trim; n/x/d:stop; a:arm; o/f:manual stroke (calibrated only).");
}
void loop() { delay(1000); }
