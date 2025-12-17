#pragma once
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

struct ControllerButtons {
    bool square   = false;
    bool cross    = false;
    bool circle   = false;
    bool triangle = false;
    bool l1       = false;
    bool r1       = false;
    bool l3       = false;
    bool r3       = false;
    bool share    = false;
    bool options  = false;
    bool ps       = false;
    bool touchpad = false;
    bool up       = false;
    bool down     = false;
    bool left     = false;
    bool right    = false;
};

struct ControllerData {
    int16_t state      = 0;
    uint16_t buttonsRaw = 0;

    float lx = 0.0f;
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
    float l2 = 0.0f;
    float r2 = 0.0f;

    ControllerButtons buttons{};
};

class ControllerInput {
public:
    // USBSerial 型で受けるように変更
    explicit ControllerInput(usb_serial_class &serial) : serial(serial) {}

    void begin(uint32_t baud) {
        serial.begin(baud);    // USB CDC では実質意味ないが互換のため残す
        while (!serial) { }    // USB が接続されるまで待機
        serial.setTimeout(5);
    }

    bool poll() {
        if (!serial.available()) return false;

        String line = serial.readStringUntil('\n');
        if (line.length() == 0) return false;

        char buf[line.length() + 1];
        line.toCharArray(buf, sizeof(buf));

        // state
        char *tok = strtok(buf, ",");
        if (!tok) return false;
        latest.state = static_cast<int16_t>(atoi(tok));

        // buttonsRaw
        tok = strtok(nullptr, ",");
        if (!tok) return false;
        latest.buttonsRaw = static_cast<uint16_t>(strtoul(tok, nullptr, 10));

        // lx, ly, rx, ry, l2, r2
        float *targets[6] = {
            &latest.lx, &latest.ly,
            &latest.rx, &latest.ry,
            &latest.l2, &latest.r2
        };

        for (int i = 0; i < 6; ++i) {
            tok = strtok(nullptr, ",");
            if (!tok) return false;
            *targets[i] = atof(tok);
        }

        decodeButtons(latest.buttonsRaw);
        return true;
    }

    const ControllerData &data() const { return latest; }

private:
    usb_serial_class &serial;
    ControllerData latest{};

    inline void decodeButtons(uint16_t raw) {
        latest.buttons.square   = raw & (1 << 0);
        latest.buttons.cross    = raw & (1 << 1);
        latest.buttons.circle   = raw & (1 << 2);
        latest.buttons.triangle = raw & (1 << 3);
        latest.buttons.l1       = raw & (1 << 4);
        latest.buttons.r1       = raw & (1 << 5);
        latest.buttons.l3       = raw & (1 << 6);
        latest.buttons.r3       = raw & (1 << 7);
        latest.buttons.share    = raw & (1 << 8);
        latest.buttons.options  = raw & (1 << 9);
        latest.buttons.ps       = raw & (1 << 10);
        latest.buttons.touchpad = raw & (1 << 11);
        latest.buttons.up       = raw & (1 << 12);
        latest.buttons.down     = raw & (1 << 13);
        latest.buttons.left     = raw & (1 << 14);
        latest.buttons.right    = raw & (1 << 15);
    }
};
