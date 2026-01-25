#!/usr/bin/env python3
import time
import serial
import pygame
from enum import IntEnum


class SystemState(IntEnum):
    READY = 0
    OVERCURRENT = 1
    UNDERVOLTAGE = 2
    NOT_CONNECTED = 3
    E_STOP = 4


def normalize_stick_float(v: float) -> float:
    if v > 1.0:
        v = 1.0
    if v < -1.0:
        v = -1.0
    return v


def apply_deadzone(x: float, deadzone: float = 0.1) -> float:
    return 0.0 if abs(x) < deadzone else x


def main():
    ser = serial.Serial(
        #port="COM5",
        port='/dev/ttyACM0',
        baudrate=115200,
        timeout=0.01,       # 完全ノンブロッキング
        write_timeout=0.01,
    )
    ser.reset_input_buffer()   # 起動時に古いゴミを捨てる

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick found. Connect PS5 controller via Bluetooth.")
        return

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using joystick: {js.get_name()}")

    power_on = True
    last_touchpad_pressed = False

    BTN_SQUARE   = 0
    BTN_CROSS    = 1
    BTN_CIRCLE   = 2
    BTN_TRIANGLE = 3
    BTN_L1       = 4
    BTN_R1       = 5
    BTN_L3       = 6
    BTN_R3       = 7
    BTN_SHARE    = 8
    BTN_OPTIONS  = 9
    BTN_PS       = 10
    BTN_TOUCHPAD = 11

    AXIS_LX = 0
    AXIS_LY = 1
    AXIS_RX = 2
    AXIS_RY = 3
    AXIS_L2 = 4
    AXIS_R2 = 5

    DEADZONE = 0.1

    # 送信レート制御用（例: 100 Hz = 0.01 s 間隔）
    SEND_INTERVAL = 0.01
    last_send_time = 0.0

    # 受信バッファ（生バイト列）
    rx_buf = b""

    try:
        while True:
            now = time.time()
            pygame.event.pump()

            # 状態
            if not js.get_init():
                state = SystemState.NOT_CONNECTED
            else:
                state = SystemState.E_STOP if not power_on else SystemState.READY

            # ボタン
            buttons = 0
            if js.get_init():
                def b(i: int) -> int:
                    try:
                        return js.get_button(i)
                    except IndexError:
                        return 0

                buttons |= (b(BTN_SQUARE)   & 1) << 0
                buttons |= (b(BTN_CROSS)    & 1) << 1
                buttons |= (b(BTN_CIRCLE)   & 1) << 2
                buttons |= (b(BTN_TRIANGLE) & 1) << 3
                buttons |= (b(BTN_L1)       & 1) << 4
                buttons |= (b(BTN_R1)       & 1) << 5
                buttons |= (b(BTN_L3)       & 1) << 6
                buttons |= (b(BTN_R3)       & 1) << 7
                buttons |= (b(BTN_SHARE)    & 1) << 8
                buttons |= (b(BTN_OPTIONS)  & 1) << 9
                buttons |= (b(BTN_PS)       & 1) << 10
                buttons |= (b(BTN_TOUCHPAD) & 1) << 11

                hat_x, hat_y = (0, 0)
                if js.get_numhats() > 0:
                    hat_x, hat_y = js.get_hat(0)

                if hat_y > 0: buttons |= 1 << 12
                if hat_y < 0: buttons |= 1 << 13
                if hat_x < 0: buttons |= 1 << 14
                if hat_x > 0: buttons |= 1 << 15

            # 軸
            lx = ly = rx = ry = 0.0
            l2 = r2 = 0.0

            if js.get_init():
                def safe_axis(i: int) -> float:
                    try:
                        return js.get_axis(i)
                    except (IndexError, pygame.error):
                        return 0.0

                lx = apply_deadzone(normalize_stick_float(safe_axis(AXIS_LX)), DEADZONE)
                ly = apply_deadzone(normalize_stick_float(safe_axis(AXIS_LY)), DEADZONE)
                rx = apply_deadzone(normalize_stick_float(safe_axis(AXIS_RX)), DEADZONE)
                ry = apply_deadzone(normalize_stick_float(safe_axis(AXIS_RY)), DEADZONE)

                raw_l2 = safe_axis(AXIS_L2)
                raw_r2 = safe_axis(AXIS_R2)
                l2 = (raw_l2 + 1.0) * 0.5
                r2 = (raw_r2 + 1.0) * 0.5
                if l2 < DEADZONE: l2 = 0.0
                if r2 < DEADZONE: r2 = 0.0

                touchpad_pressed = bool((buttons >> 11) & 0x1)
                if touchpad_pressed and not last_touchpad_pressed:
                    power_on = not power_on
                last_touchpad_pressed = touchpad_pressed

                if not power_on:
                    state = SystemState.E_STOP

            # ==========================
            #  送信：間隔を絞る（100 Hz）
            # ==========================
            if now - last_send_time >= SEND_INTERVAL:
                last_send_time = now
                line = f"{int(state)},{buttons}," \
                       f"{lx:.3f},{ly:.3f},{rx:.3f},{ry:.3f},{l2:.3f},{r2:.3f}\n"
                try:
                    ser.write(line.encode("ascii"))
                    #print(line,flush=True)
                except serial.SerialTimeoutException:
                    # 無理に詰め込まない
                    pass

            # ==========================
            #  受信：最新行だけ使う
            # ==========================
            n = ser.in_waiting
            if n:
                rx_buf += ser.read(n)

                latest_line = None
                while True:
                    idx = rx_buf.find(b"\n")
                    if idx < 0:
                        break
                    raw = rx_buf[:idx].rstrip(b"\r")
                    rx_buf = rx_buf[idx+1:]
                    if raw:
                        latest_line = raw  # 最後の1行だけ保持

                if latest_line is not None:
                    cols = latest_line.split(b",")
                    # デバッグ表示（必要なら削る）
                    print("Recv latest CSV:",
                          [c.decode("ascii", "ignore") for c in cols],lx,ly,rx,ry,l2,r2)

            # 軽くCPUを休ませる
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        js.quit()
        pygame.joystick.quit()
        pygame.quit()
        ser.close()


if __name__ == "__main__":
    main()
