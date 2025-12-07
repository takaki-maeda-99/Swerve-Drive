# Swerve UART/USB Serial Control

Teensy 4.0 firmware receives one line of CSV text via USB serial or UART (Serial4) and converts it to chassis commands.

## Physical link
- USB: default `Serial` (same format as UART)
- UART: `Serial4`, 115200 bps, 8-N-1

## Frame format (one line)
```
state,buttons,lx,ly,rx,ry,l2,r2\n
```
- Comma-separated decimal integers, terminated by `\n` (no extra spaces)
- All fields required; incomplete lines are ignored

## Fields and ranges
- `state` : connection/state flag (int, pass through as-is)
- `buttons` : 16-bit bitfield (see below)
- `lx`,`ly`,`rx`,`ry` : left/right stick axes, approx. -127 to 127 (center ? 0)
- `l2`,`r2` : trigger values, approx. 0 to 255

### Button bit assignment (`buttons` field)
- bit0: square
- bit1: cross
- bit2: circle
- bit3: triangle (holding triggers homing in loop)
- bit4: L1
- bit5: R1
- bit8: Share
- bit9: Options

## Scaling inside firmware (see `src/main.cpp`)
- `vx = (ly / 127.0f) * 2.0f`   // m/s, forward/back
- `vy = (lx / 127.0f) * 2.0f`   // m/s, left/right
- `wz = ((r2 - l2) / 255.0f) * 5.0f` // rad/s, yaw (R2 minus L2)
- Triangle button (`bit3`) triggers `homing()` when pressed

## Example frames
- Idle (sticks centered): `0,0,0,0,0,0,0,0`
- Forward ~50%: `0,0,0,64,0,0,0,0`
- Strafe right ~50%: `0,0,64,0,0,0,0,0`
- Rotate CCW ~40%: `0,0,0,0,0,0,0,102`
- Homing request: `0,8,0,0,0,0,0,0`  (bit3=1 -> value 8)

Send one line per update; faster rates improve responsiveness but avoid flooding the port (e.g., 50?100 Hz is sufficient).
