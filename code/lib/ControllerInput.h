#pragma once
#include <Arduino.h>
#include <string.h>

struct ControllerButtons {
    bool square = false;
    bool cross = false;
    bool circle = false;
    bool triangle = false;
    bool l1 = false;
    bool r1 = false;
    bool options = false;
    bool share = false;
};

struct ControllerData {
    int16_t state = 0;
    uint16_t buttonsRaw = 0;
    int16_t lx = 0;
    int16_t ly = 0;
    int16_t rx = 0;
    int16_t ry = 0;
    int16_t l2 = 0;
    int16_t r2 = 0;
    ControllerButtons buttons{};
};

class ControllerInput {
public:
    explicit ControllerInput(HardwareSerial &serial) : serial(serial) {}

    void begin(uint32_t baud) { serial.begin(baud); }

    // Read a line if available; return true only when data updated.
    bool poll() {
        if (!serial.available()) return false;

        String line = serial.readStringUntil('\n');
        int values[8] = {0};
        int idx = 0;

        char buf[line.length() + 1];
        line.toCharArray(buf, sizeof(buf));
        for (char *tok = strtok(buf, ","); tok && idx < 8; tok = strtok(nullptr, ",")) {
            values[idx++] = atoi(tok);
        }
        if (idx < 8) return false; // incomplete line

        latest.state      = values[0];
        latest.buttonsRaw = static_cast<uint16_t>(values[1]);
        latest.lx         = values[2];
        latest.ly         = values[3];
        latest.rx         = values[4];
        latest.ry         = values[5];
        latest.l2         = values[6];
        latest.r2         = values[7];

        decodeButtons(latest.buttonsRaw);
        return true;
    }

    const ControllerData &data() const { return latest; }

private:
    HardwareSerial &serial;
    ControllerData latest{};

    inline void decodeButtons(uint16_t raw) {
        latest.buttons.square   = raw & (1 << 0);
        latest.buttons.cross    = raw & (1 << 1);
        latest.buttons.circle   = raw & (1 << 2);
        latest.buttons.triangle = raw & (1 << 3);
        latest.buttons.l1       = raw & (1 << 4);
        latest.buttons.r1       = raw & (1 << 5);
        latest.buttons.share    = raw & (1 << 8);
        latest.buttons.options  = raw & (1 << 9);
    }
};
