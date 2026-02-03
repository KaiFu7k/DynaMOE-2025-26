# Launcher Speed Guide

## Single-Button Launch (Recommended)
Press **Right Bumper** to auto-launch:
1. Robot auto-aligns to goal
2. Launcher spins up to correct velocity (based on distance)
3. Fires LEFT then RIGHT automatically
4. Press **B** to abort

## Manual Controls (Backup)
| Button | Action |
|--------|--------|
| A | Spin up launchers (manual speed) |
| B | Stop launchers / Abort launch |
| X | Feed LEFT |
| Y | Feed RIGHT |
| Gamepad2 D-Pad U/D | Adjust speed ±50 RPM |

## Speed Range
- Min: 800 RPM | Default: 1200 RPM | Max: 1600 RPM

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

## Telemetry
- "Distance to Goal: XX.X in" = current distance
- "Auto Velocity: XXXX RPM" = calculated speed for this distance
- "Aligned: YES/NO" = robot facing goal
- "Launcher Ready: YES" = safe to feed

## Troubleshooting
| Problem | Solution |
|---------|----------|
| Speed won't change | Hit min/max limit |
| Not reaching target | Low battery - reduce RPM |
| Never shows ready | Target too high for battery |
| Wrong auto-velocity | Check pose is set correctly (Auton → TeleOp) |
| Launch sequence stuck | Press B to abort, check alignment |
