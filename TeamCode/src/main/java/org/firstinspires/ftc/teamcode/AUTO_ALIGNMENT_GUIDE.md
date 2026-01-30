# Auto-Alignment Guide

## Features
- **Auto-Align**: Robot rotates to face goal (LB toggle)
- **Auto-Velocity**: Adjusts RPM based on distance (RB toggle)

## Controls
| Button | Function |
|--------|----------|
| LB | Toggle auto-align |
| RB | Toggle auto-velocity |
| A/B | Start/stop launchers |
| X/Y | Feed left/right |
| D-Pad Up/Down | Manual speed adjust |

## Workflow
1. Drive into launch zone
2. Press LB to enable auto-align
3. Press RB to enable auto-velocity
4. Wait for "Aligned: YES"
5. Press A to spin up
6. Wait for "Ready: YES"
7. Press X or Y to feed

## Telemetry Indicators
- **Aligned: YES** = Facing goal (within 2°)
- **Ready: YES** = Launchers at target speed

## Tuning (LauncherAssist.java)
```java
ALIGNMENT_TOLERANCE_DEGREES = 2.0  // How close to target angle
ROTATION_SPEED = 0.6               // Rotation speed
KP = 1.5, KD = 0.1                 // PID constants
```

## Velocity Table
| Distance | RPM |
|----------|-----|
| 24" | 1000 |
| 36" | 1150 |
| 48" | 1250 |
| 60" | 1350 |
| 72" | 1450 |
| 84" | 1550 |

## Troubleshooting
- **Wrong rotation**: Check alliance is set correctly
- **Distance wrong**: Verify starting pose in TeleOp
- **Oscillating**: Increase tolerance or reduce rotation speed

## Fallback
If auto fails, release LB and use right stick for manual rotation.
