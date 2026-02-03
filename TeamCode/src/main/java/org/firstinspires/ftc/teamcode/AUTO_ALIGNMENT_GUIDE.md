# Auto-Alignment Guide

## Single-Button Launch Mode

Press **Right Bumper** once to execute the full launch sequence:

1. **ALIGNING** - Robot auto-rotates to face goal, launcher spins up
2. **FIRING_LEFT** - Feeds left artifact
3. **FIRING_RIGHT** - Feeds right artifact (0.3s delay)
4. **IDLE** - Returns to normal driving

Press **B** at any time to abort.

## Controls
| Button | Function |
|--------|----------|
| **RB** | Start launch sequence |
| **B** | Abort launch / Stop launcher |
| A | Manual spin up (backup) |
| X/Y | Manual feed left/right (backup) |
| Gamepad2 D-Pad | Manual speed adjust ±50 RPM |

## Workflow
1. Drive into launch zone
2. Press Right Bumper
3. Wait for sequence to complete (auto-align → fire → done)
4. Resume driving/intaking

## Telemetry Indicators
- **IDLE** = Ready to launch (press RB)
- **ALIGNING...** = Rotating to goal, spinning up
- **FIRING LEFT/RIGHT** = Feeding artifacts
- **Aligned: YES** = Facing goal (within 2°)
- **Launcher Ready: YES** = At target speed

## Auto-Velocity Table (Tuned 2026-02-03)
| Distance | RPM |
|----------|-----|
| 60" | 1090 |
| 70" | 1170 |
| 80-90" | 1180 |
| 101" | 1230 |
| 113" | 1250 |
| 120" | 1300 |
| 130" | 1320 |
| 143" | 1420 |
| 150" | 1490 |
| 157" | 1480 |

## Tuning Constants (LauncherAssist.java)
```java
ALIGNMENT_TOLERANCE_DEGREES = 2.0  // How close to target angle
ROTATION_SPEED = 0.6               // Max rotation speed
MIN_ROTATION_POWER = 0.18          // Overcomes static friction
KP = 1.5, KD = 0.1                 // PD controller constants
```

## Pose Transfer (Auton → TeleOp)
- Auton saves final pose to `RobotState`
- TeleOp reads it automatically
- If no valid pose (>5 min old), falls back to manual selection
- Correct pose is critical for accurate distance/angle calculations

## Troubleshooting
| Problem | Solution |
|---------|----------|
| Wrong rotation direction | Check alliance is set correctly |
| Distance shows wrong | Verify pose transferred from Auton |
| Oscillating back and forth | Increase tolerance or reduce KP |
| Never aligns | Check if goal position is correct for alliance |
| Sequence stuck in ALIGNING | Will auto-timeout after 5 sec, or press B |

## Fallback
If auto-launch fails, press B to abort and use manual controls:
- A to spin up manually
- Right stick to aim
- X/Y to feed
