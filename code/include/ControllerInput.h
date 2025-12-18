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
    int16_t  state = 0;
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
    // USB/Hardware どちらでも渡せる
    explicit ControllerInput(Stream &stream) : stream(stream) {}

    // beginは HardwareSerial / usb_serial_class のときだけ呼べばOK
    // （呼ばなくてもコンパイル通るようにテンプレート化）
    template <typename SerialLike>
    void begin(SerialLike &s, uint32_t baud) {
        s.begin(baud);
        // USB系だけ「接続待ち」が有効な環境があるので、必要なら呼び出し側で待つのが安全
        stream.setTimeout(5);
    }

    // beginを使わない場合でも timeout だけは設定できる
    void setTimeout(uint32_t ms) { stream.setTimeout(ms); }

    bool poll() {
        // Stream共通APIで1行読む（\nまで）
        if (stream.available() <= 0) return false;

        String line = stream.readStringUntil('\n');
        if (line.length() == 0) return false;

        // \r\n 対策
        if (line.endsWith("\r")) line.remove(line.length() - 1);

        // strtok用バッファ
        char buf[line.length() + 1];
        line.toCharArray(buf, sizeof(buf));

        char *tok = strtok(buf, ",");
        if (!tok) return false;
        latest.state = (int16_t)atoi(tok);

        tok = strtok(nullptr, ",");
        if (!tok) return false;
        latest.buttonsRaw = (uint16_t)strtoul(tok, nullptr, 10);

        // lx, ly, rx, ry, l2, r2
        float *targets[6] = {
            &latest.lx, &latest.ly,
            &latest.rx, &latest.ry,
            &latest.l2, &latest.r2
        };

        for (int i = 0; i < 6; ++i) {
            tok = strtok(nullptr, ",");
            if (!tok) return false;
            *targets[i] = (float)atof(tok);
        }

        decodeButtons(latest.buttonsRaw);
        return true;
    }

    const ControllerData &data() const { return latest; }

private:
    Stream &stream;
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
