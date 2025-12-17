#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "DjiMotor.hpp"
#include "PID.h"
#include "ControllerInput.h"
#include <array>
#include <cmath>

constexpr size_t NUM_WHEELS = 4;

static std::array<float, NUM_WHEELS> prevAngle{{0.f, 0.f, 0.f, 0.f}};

constexpr float kWheelRadius = 0.042f;   // [m] wheel radius for speed conversion
constexpr float kMaxLinear   = 2.0f;      // [m/s] max chassis translational speed
constexpr float kMaxYawRate  = 5.0f;     // [rad/s] max chassis yaw rate
constexpr float kStickMax    = 127.0f;     // joystick full scale
constexpr float kTriggerMax = 255.0f;   // trigger full scale
constexpr float kBaseRadius  = 0.25f;     // = WHEEL_BASE_RADIUS

struct WheelPosition {
    float x;
    float y;
};

constexpr std::array<WheelPosition, NUM_WHEELS> wheelPositions{{
    { kBaseRadius,  -kBaseRadius},   // wheel 0
    {kBaseRadius,  kBaseRadius},   // wheel 1
    {-kBaseRadius, kBaseRadius},   // wheel 2
    { -kBaseRadius, -kBaseRadius},   // wheel 3
}};

// LSピン定義 (ステアリングモーター用)
#define LSPIN11 2
#define LSPIN12 3
#define LSPIN21 4
#define LSPIN22 5
#define LSPIN31 6
#define LSPIN32 7
#define LSPIN41 8
#define LSPIN42 9

// DIPスイッチピン定義 (ID設定用)
#define DIP_PIN1 12
#define DIP_PIN2 13

//オドメトリのエンコーダーのピンの定義
#define ODOMETRY_X_ENCODER_A_PIN 10
#define ODOMETRY_X_ENCODER_B_PIN 11
#define ODOMETRY_Y_ENCODER_A_PIN 14
#define ODOMETRY_Y_ENCODER_B_PIN 15

//オドメトリエンコーダー変数
volatile int32_t encoderX_count = 0;
volatile int32_t encoderY_count = 0;
volatile uint32_t lastSpeedUpdate = 0;
volatile int32_t lastEncoderX_count = 0;
volatile int32_t lastEncoderY_count = 0;
float speedX = 0.0f;  // [m/s] X方向の線速度
float speedY = 0.0f;  // [m/s] Y方向の線速度

// エンコーダー仕様と計算定数
const float ENCODER_RESOLUTION = 360.0f;    // パルス/回転 (PPR)
const float WHEEL_DIAMETER = 0.038f;        // ホイール直径 [m] (38mm)
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER; // 円周 [m]
const uint32_t SPEED_UPDATE_INTERVAL = 10; // 速度更新間隔 [ms]

// オドメトリ速度送信フレーム（ID: 0x200）の構造体
struct OdometrySpeedFrame {
  float speedX;             // 4バイト
  float speedY;             // 4バイト
} __attribute__((packed));

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
DjiMotorCan<CAN2> steer_motors(can2, 72.0f);  // 72:1 ギア比で初期化
DjiMotorCan<CAN1> wheel_motors(can1, 19.0f);  // 19:1 ギア比で初期化

ControllerInput controller(Serial);
IntervalTimer motorControlTimer;

uint8_t myBoardId = 1;

struct PidParam {
    float kp, ki, kd;
    float outMin, outMax;
    uint32_t sampleMs;
};

// 「基準値」
// constexpr PidParam AnglePidParam      {30,  0, 0, -22.0,   22.0,   10};
// constexpr PidParam SteerSpeedPidParam {1500,0, 0, -8000.0, 8000.0, 1};
// constexpr PidParam WheelSpeedPidParam {305, 0, 0, -16000.0,16000.0,1};

// 角度用 PID
Pid AnglePid[4] = {
    Pid(30, 0, 0, -22.0, 22.0, 10),  // wheel 0
    Pid(30, 0, 0, -22.0, 22.0, 10),  // wheel 1
    Pid(30, 0, 0, -22.0, 22.0, 10),  // wheel 2
    Pid(30, 0, 0, -22.0, 22.0, 10),  // wheel 3
};

// ステア速度用 PID
Pid SteerSpeedPid[4] = {
    Pid(1000, 0, 0, -8000.0, 8000.0, 1), // wheel 0
    Pid(1000, 0, 0, -8000.0, 8000.0, 1), // wheel 1
    Pid(1000, 0, 0, -8000.0, 8000.0, 1), // wheel 2
    Pid(1000, 0, 0, -8000.0, 8000.0, 1), // wheel 3
};

// ホイール速度用 PID
Pid WheelSpeedPid[4] = {
    Pid(1600, 0, 10, -14000.0, 14000.0, 1), // wheel 0
    Pid(1600, 0, 10, -14000.0, 14000.0, 1), // wheel 1
    Pid(1600, 0, 10, -14000.0, 14000.0, 1), // wheel 2
    Pid(1600, 0, 10, -14000.0, 14000.0, 1), // wheel 3
};

inline void steer_angleControl(size_t idx, float targetAngle, bool bounded, DjiMotorCan<CAN2> &motors) {
    const auto &fb = motors.feedback(idx + 1);
    const float fbAngle = fb.getAngleRadiansWrapped();
    const float fbSpeed = fb.getSpeedRadiansPerSec();

    if(bounded) {
        targetAngle = constrain(targetAngle, -PI, PI);
    }
    else {
        if(targetAngle - fbAngle > PI) {
            targetAngle -= DjiConstants::PI_2;
        } else if(targetAngle - fbAngle < -PI) {
            targetAngle += DjiConstants::PI_2;
        }
    }

    int16_t angleCmd  = AnglePid[idx].compute(fbAngle, targetAngle);
    int16_t speedCmd  = SteerSpeedPid[idx].compute(fbSpeed, angleCmd);

    motors.sendCurrent(idx + 1, speedCmd);
}


inline void steer_speedControl(size_t idx, float targetSpeed, DjiMotorCan<CAN2> &motors) {
    const auto &fb = motors.feedback(idx + 1);

    int16_t speedCmd = SteerSpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed); // ← 修正
    motors.sendCurrent(idx + 1, speedCmd);
}


inline void wheel_speedControl(size_t idx, float targetSpeed, DjiMotorCan<CAN1> &motors) {
    const auto &fb = motors.feedback(idx + 1);

    int16_t speedCmd = WheelSpeedPid[idx].compute(fb.getSpeedRadiansPerSec(), targetSpeed); // ← 修正
    motors.sendCurrent(idx + 1, speedCmd);
}


bool readLimitSwitch1(uint8_t steerIndex) {
    uint8_t lsPin1;
    
    switch(steerIndex) {
        case 0: lsPin1 = LSPIN11; break;
        case 1: lsPin1 = LSPIN21; break;
        case 2: lsPin1 = LSPIN31; break;
        case 3: lsPin1 = LSPIN41; break;
        default: return false;
    }
    
    return digitalRead(lsPin1);
}

bool readLimitSwitch2(uint8_t steerIndex) {
    uint8_t lsPin2;
    
    switch(steerIndex) {
        case 0: lsPin2 = LSPIN12; break;
        case 1: lsPin2 = LSPIN22; break;
        case 2: lsPin2 = LSPIN32; break;
        case 3: lsPin2 = LSPIN42; break;
        default: return false;
    }
    
    return digitalRead(lsPin2);
}

inline float wrapPi(float ang) {
    while (ang > PI)  ang -= 2.0f * PI;
    while (ang < -PI) ang += 2.0f * PI;
    return ang;
}


struct WheelCommand {
    float speed;
    float angle;
};

std::array<WheelCommand, NUM_WHEELS> ik(float vx, float vy, float wz) {
    std::array<WheelCommand, NUM_WHEELS> wheelCommands{};

    for (size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto &pos = wheelPositions[i];

        const float wheelVx = vx + wz * pos.y;
        const float wheelVy = vy + wz * pos.x;

        wheelCommands[i].speed = sqrtf(wheelVx * wheelVx + wheelVy * wheelVy);
        wheelCommands[i].angle = atan2f(wheelVy, wheelVx);

        if (wheelCommands[i].speed < 1e-4f) {
            wheelCommands[i].speed = 0.0f;
            wheelCommands[i].angle = prevAngle[i];
        } else {
            float diff = wrapPi(wheelCommands[i].angle - prevAngle[i]);

            if (fabsf(diff) > PI / 2.0f) {
                wheelCommands[i].angle += (diff > 0.0f) ? -PI : PI;
                wheelCommands[i].speed = -wheelCommands[i].speed;
            }

            wheelCommands[i].angle = wrapPi(wheelCommands[i].angle);
            prevAngle[i] = wheelCommands[i].angle;
        }
    }

    return wheelCommands;
}

float vx = 0;
float vy = 0;
float wz = 0;

void ISR() {
    // Inverse kinematics to per-wheel targets.
    const auto wheelCommands = ik(vx, vy, wz);

    // Monitor IK outputs.
    // Serial.printf("vx:%.3f vy:%.3f wz:%.3f\n", vx, vy, wz);
    // for (size_t i = 0; i < NUM_WHEELS; ++i) {
    //     Serial.printf("wheel[%u] angle:%.3f speed:%.3f\n",
    //                     (unsigned)i, wheelCommands[i].angle, wheelCommands[i].speed);
    // }

    // Apply steering and drive commands.
    for (size_t i = 0; i < NUM_WHEELS; ++i) {
        steer_angleControl(i, wheelCommands[i].angle, false, steer_motors);
        const float wheelAngular = (wheelCommands[i].speed / kWheelRadius);
        wheel_speedControl(i, wheelAngular, wheel_motors);
    }

    steer_motors.flush();
    wheel_motors.flush();
}

void homing(){
    const float homeAngles[4] = {PI/2.0f, 0.0f, -PI/2.0f, PI};
    
    motorControlTimer.end();
    
    bool homingComplete[4] = {false};
    while(!homingComplete[0] || !homingComplete[1] || !homingComplete[2] || !homingComplete[3]) {
        for(int i=0; i<4; i++) {
            bool ls1 = readLimitSwitch1(i);
            bool ls2 = readLimitSwitch2(i);

            if((ls1 && ls2) || homingComplete[i] ) {
                steer_motors.resetAngle(i+1, homeAngles[i]);
                steer_speedControl(i, 0, steer_motors);
                homingComplete[i] = true;
            }
            else if(ls1) {
                steer_speedControl(i, 2, steer_motors);
            }
            else{
                steer_speedControl(i, 20, steer_motors);
            }
        }
        steer_motors.flush();
        delay(1);
    }
    
    motorControlTimer.begin(ISR, 1000);
    motorControlTimer.priority(0);
}

// エンコーダー割り込み処理 (X方向)
void encoderX_ISR() {
    int stateA = digitalRead(ODOMETRY_X_ENCODER_A_PIN);
    int stateB = digitalRead(ODOMETRY_X_ENCODER_B_PIN);

    // A相の変化でカウント、B相の状態で方向判定 (A=Bなら順方向、A!=Bなら逆方向)
    if (stateA == stateB) {
        encoderX_count++;
    } else {
        encoderX_count--;
    }
}

// エンコーダー割り込み処理 (Y方向)
void encoderY_ISR() {
    int stateA = digitalRead(ODOMETRY_Y_ENCODER_A_PIN);
    int stateB = digitalRead(ODOMETRY_Y_ENCODER_B_PIN);

    // A相の変化でカウント、B相の状態で方向判定
    if (stateA == stateB) {
        encoderY_count++;
    } else {
        encoderY_count--;
    } 
}

// 速度計算関数
void updateOdometrySpeed() {
    uint32_t currentTime = millis();
    if (currentTime - lastSpeedUpdate >= SPEED_UPDATE_INTERVAL) {
        // 時間間隔を計算 [s]
        float deltaTime = (currentTime - lastSpeedUpdate) / 1000.0f;
        
        // エンコーダーカウントの差分を計算
        int32_t deltaX = encoderX_count - lastEncoderX_count;
        int32_t deltaY = encoderY_count - lastEncoderY_count;
        
        // 回転数に変換 [回転/s]
        // rps = (delta_count / ENCODER_RESOLUTION) / deltaTime
        float rpsX = deltaX / (ENCODER_RESOLUTION * deltaTime);
        float rpsY = deltaY / (ENCODER_RESOLUTION * deltaTime);
        
        // 線速度に変換 [m/s]
        // speed = rps * WHEEL_CIRCUMFERENCE
        speedX = rpsX * WHEEL_CIRCUMFERENCE;
        speedY = rpsY * WHEEL_CIRCUMFERENCE;
        
        // 前回値を更新
        lastEncoderX_count = encoderX_count;
        lastEncoderY_count = encoderY_count;
        lastSpeedUpdate = currentTime;
    }
}

// オドメトリ速度取得関数
float getSpeedX() {
    return speedX;
}

float getSpeedY() {
    return speedY;
}


uint8_t readBoardId() {
    uint8_t id = 0;
    if (!digitalRead(DIP_PIN1)) id += 1;
    if (!digitalRead(DIP_PIN2)) id += 2;
    return id + 1; // 0-3 → 1-4
}


void setup() {
//   Serial.begin(57600);
  controller.begin(115200);
  
  // DIPスイッチピン設定
  pinMode(DIP_PIN1, INPUT_PULLUP);
  pinMode(DIP_PIN2, INPUT_PULLUP);

  delay(100);

// Board ID読み取り
  myBoardId = readBoardId();
  Serial.printf("Board ID: %d\n", myBoardId);  

  pinMode(LSPIN11, INPUT_PULLUP);
  pinMode(LSPIN12, INPUT_PULLUP);
  pinMode(LSPIN21, INPUT_PULLUP);
  pinMode(LSPIN22, INPUT_PULLUP);
  pinMode(LSPIN31, INPUT_PULLUP);
  pinMode(LSPIN32, INPUT_PULLUP);
  pinMode(LSPIN41, INPUT_PULLUP);
  pinMode(LSPIN42, INPUT_PULLUP);
  pinMode(ODOMETRY_X_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ODOMETRY_X_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ODOMETRY_Y_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ODOMETRY_Y_ENCODER_B_PIN, INPUT_PULLUP);

  // エンコーダー割り込み設定（立ち上がりのみ）
  // RISINGのみに設定されているが、ISRの実装は両方の変化を捕捉する四倍速エンコーディングに基づいているため、注意が必要
  attachInterrupt(digitalPinToInterrupt(ODOMETRY_X_ENCODER_A_PIN), encoderX_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ODOMETRY_Y_ENCODER_A_PIN), encoderY_ISR, RISING);

// モーター制御タイマー開始 (1kHz = 1000μs間隔)
  motorControlTimer.begin(ISR, 1000);
  motorControlTimer.priority(128);

    // homing();  
}

void loop() {
    // オドメトリ速度更新とCAN送信
    updateOdometrySpeed();

    // シリアルモニタへ速度を出力
    // Serial.printf("SpeedX: %.3f m/s, SpeedY: %.3f m/s\n", getSpeedX(), getSpeedY());
    static uint32_t last_tx = 0;
    if (millis() - last_tx >= 10) {            // 100 Hz
    last_tx = millis();
    if (Serial.availableForWrite() > 64) {   // 送れるときだけ
        Serial.printf("%.3f,%.3f,%.3f\n", getSpeedX(), getSpeedY(), wz);
    }
    }

    if (controller.poll()) {
        const ControllerData &d = controller.data();

        // Monitor controller input.
        // Serial.printf("ctrl state:%d lx:%d ly:%d rx:%d ry:%d l2:%d r2:%d buttons:0x%04X\n",
        //                 d.state, d.lx, d.ly, d.rx, d.ry, d.l2, d.r2, d.buttonsRaw);

        // Scale controller inputs to chassis velocities.
        vx = d.ly* kMaxLinear;
        vy = d.lx* kMaxLinear;
        wz = d.rx* kMaxYawRate;


        if (d.buttons.triangle) {
            Serial.println("Homing restart requested");
            homing();
        }
    }
    
    delay(10);
}
