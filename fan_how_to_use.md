# Fan Controller

CAN 通信経由でモーター(ファン)を PWM 制御するためのプロトコル仕様と実装例。

`src/main.cpp` は Teensy 4.0 向けのテストコード。

## CAN プロトコル仕様

| 項目 | 値 |
|------|-----|
| ボーレート | 1,000,000 bps |
| CAN ID | `0x100` |
| データ長 | 2 バイト |
| バイトオーダー | リトルエンディアン |

### データフォーマット

```
buf[0]: PWM 値の下位バイト (LSB)
buf[1]: PWM 値の上位バイト (MSB)
```

### PWM 値の範囲

| PWM 値 | 動作 |
|--------|------|
| 1100 | 停止 |
| 1100 - 1940 | 値が大きいほど高速回転 |
| 1940 | 最大速度 |

## 実装例 (Teensy 4.0 / FlexCAN_T4)

```cpp
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void setup() {
    can1.begin();
    can1.setBaudRate(1000000);
}

void sendPWM(uint16_t pwmVal) {
    CAN_message_t msg;
    msg.id = 0x100;
    msg.len = 2;
    msg.buf[0] = pwmVal & 0xFF;        // 下位バイト
    msg.buf[1] = (pwmVal >> 8) & 0xFF;  // 上位バイト
    can1.write(msg);
}

void loop() {
    sendPWM(1500); // 中速で回転
    delay(1000);
}
```
