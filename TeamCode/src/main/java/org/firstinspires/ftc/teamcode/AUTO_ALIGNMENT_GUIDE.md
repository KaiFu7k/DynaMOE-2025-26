# Auto-Alignment and Auto-Velocity System Guide

## Overview

The DynaMOE robot now has automatic positioning and velocity adjustment for the launcher system. This guide explains how to use, test, and tune these features.

## Features Added

### 1. **Auto-Alignment**
- Automatically rotates robot to face the goal
- Works from anywhere in the launch zone
- Driver can still control forward/backward/strafe movement
- Shows real-time distance and angle error

### 2. **Auto-Velocity**
- Automatically adjusts launcher RPM based on distance to goal
- Uses lookup table with linear interpolation
- Updates in real-time as robot moves
- Can be toggled on/off during match

---

## Controls

### **Gamepad 1 (Driver)**

| Button | Function | Description |
|--------|----------|-------------|
| **Left Bumper** | Auto-Align | Hold to rotate toward goal |
| **Right Bumper** | Toggle Auto-Velocity | Enable/disable distance-based speed |
| Left Stick | Drive | Normal forward/backward/strafe (works during auto-align) |
| Right Stick | Manual Rotate | Only works when auto-align is OFF |
| A | Spin Up Launchers | Uses current speed (manual or auto) |
| B | Stop Launchers | Stop flywheels |
| X | Feed Left | Launch from left side |
| Y | Feed Right | Launch from right side |
| D-Pad Up/Down | Adjust Speed | Manual speed ±50 RPM (when auto-velocity OFF) |

---

## Usage Workflow

### **Option 1: Full Auto (Recommended for Competition)**

1. Drive robot into launch zone (diamond area in center of field)
2. **Hold Left Bumper** - Robot rotates to face goal
3. **Tap Right Bumper** - Enable auto-velocity mode
4. Watch telemetry until you see:
   - "Aligned: YES ✓"
   - "In Launch Zone: YES"
5. **Press A** - Spin up launchers (at auto-calculated speed)
6. Wait for "Ready: YES"
7. **Press X or Y** - Launch artifact
8. Repeat steps 6-7 for additional artifacts

### **Option 2: Auto-Align with Manual Speed**

1. Drive into launch zone
2. **Use D-Pad Up/Down** to set desired speed manually
3. **Hold Left Bumper** - Auto-align to goal
4. **Press A** - Spin up at manual speed
5. Wait for alignment and "Ready: YES"
6. **Press X or Y** - Launch

### **Option 3: Manual Positioning with Auto-Velocity**

1. Drive and manually rotate to face goal
2. **Tap Right Bumper** - Enable auto-velocity
3. **Press A** - Launchers spin at recommended speed
4. **Press X or Y** - Launch

---

## Telemetry Display

When auto-align or auto-velocity is active, you'll see:

```
=== DYNAMOE 19889 TELEOP ===

Drive Mode: Field-Centric
Heading: 45.0°

Auto-Align: ACTIVE
Auto-Velocity: ENABLED
  Distance: 52.3 in
  Angle Error: +2.1°
  Aligned: YES ✓
  In Launch Zone: YES

Launcher: ACTIVE
Target Speed: 1285 RPM
  Left: 1282 RPM
  Right: 1287 RPM
  Ready: YES
```

### **Key Indicators**
- **Aligned: YES ✓** = Robot is facing goal (within ±3°)
- **In Launch Zone: YES** = Robot is in valid scoring area
- **Ready: YES** = Launchers at target speed, safe to feed

---

## Testing and Calibration

### **Phase 1: Test Auto-Alignment**

1. Place robot at known position in launch zone
2. Note current heading on telemetry
3. Hold Left Bumper
4. Verify robot rotates toward goal
5. Check "Aligned: YES ✓" appears when facing goal
6. Release Left Bumper and verify rotation stops

**Expected Behavior:**
- Robot should rotate smoothly (not jerky)
- Should stop within ±3° of goal
- Distance should match measured distance to goal

### **Phase 2: Calibrate Velocity Table**

The default velocity table is in `LauncherAssist.java`:

```java
private static final double[][] VELOCITY_TABLE = {
    {24, 1000},   // 24 inches -> 1000 RPM
    {36, 1150},   // 36 inches -> 1150 RPM
    {48, 1250},   // 48 inches -> 1250 RPM
    {60, 1350},   // 60 inches -> 1350 RPM
    {72, 1450},   // 72 inches -> 1450 RPM
    {84, 1550}    // 84 inches -> 1550 RPM
};
```

**To Calibrate:**

1. Measure exact distances from goal: 24", 36", 48", 60", 72"
2. Mark these positions on practice field
3. Position robot at each distance
4. Enable auto-velocity mode
5. Note recommended RPM on telemetry
6. Manually test launching at that RPM
7. Adjust RPM up/down until artifacts score reliably
8. Record successful RPM values
9. Update VELOCITY_TABLE in `LauncherAssist.java`

**Example Testing Log:**

| Distance | Auto RPM | Test Result | Adjusted RPM | Notes |
|----------|----------|-------------|--------------|-------|
| 24 in | 1000 | Too weak | 1100 | Barely reached |
| 36 in | 1150 | Perfect ✓ | 1150 | No change needed |
| 48 in | 1250 | Too strong | 1200 | Overshot goal |
| 60 in | 1350 | Perfect ✓ | 1350 | - |

### **Phase 3: Test Full Workflow**

1. Start match simulation (2 min timer)
2. Drive to launch zone
3. Use auto-align + auto-velocity
4. Launch all preloaded artifacts
5. Drive to intake more artifacts
6. Return to launch zone
7. Repeat launching

**Metrics to Track:**
- Time to align (should be < 2 seconds)
- Scoring accuracy (% of launches that score)
- Battery voltage effect (does low battery change needed RPM?)

---

## Tuning Parameters

### **In `LauncherAssist.java`**

```java
// How close is "aligned" (smaller = more precise, harder to achieve)
private static final double ALIGNMENT_TOLERANCE_DEGREES = 3.0;

// Speed for auto-rotation (higher = faster rotation)
private static final double ROTATION_SPEED = 0.3;
```

### **Recommended Adjustments**

**If robot rotates too slowly:**
- Increase `ROTATION_SPEED` to 0.4 or 0.5

**If robot oscillates (wobbles back and forth):**
- Increase `ALIGNMENT_TOLERANCE_DEGREES` to 4.0 or 5.0
- Or reduce `ROTATION_SPEED` to 0.2

**If robot overshoots goal:**
- Reduce `ROTATION_SPEED` to 0.2
- Or adjust the proportional scaling in `getRotationPower()`

---

## Field Coordinate Configuration

### **Setting Alliance Color**

In `DynaMOE_19889_TeleOp.java`, line ~68:

```java
private FieldPositions.Alliance alliance = FieldPositions.Alliance.BLUE;
```

**IMPORTANT:** Change this to `RED` when on red alliance!

### **Verifying Goal Positions**

Goals are defined in `FieldPositions.java`:

```java
public static final Pose BLUE_GOAL_POSITION = new Pose(-60, 60, 0);
public static final Pose RED_GOAL_POSITION = new Pose(60, 60, 0);
```

These assume:
- Field origin (0,0) at center
- 144" × 144" field
- Goals in top-left (-60, 60) and top-right (60, 60)

**To verify:** Use telemetry distance when robot is at known positions.

### **Launch Zone Boundaries**

Defined in `FieldPositions.java`:

```java
private static final double LAUNCH_ZONE_MIN_X = -40;
private static final double LAUNCH_ZONE_MAX_X = 40;
private static final double LAUNCH_ZONE_MIN_Y = 20;
private static final double LAUNCH_ZONE_MAX_Y = 65;
```

Adjust these if the diamond-shaped launch zone doesn't match the field.

---

## Troubleshooting

### **Problem: "Auto-Align: Off" never changes**

**Cause:** LauncherAssist not initialized

**Fix:** Verify in `DynaMOE_19889_TeleOp.java` line ~115:
```java
robot.init(hardwareMap, follower, alliance);
```
Make sure `follower` and `alliance` are passed correctly.

---

### **Problem: Robot rotates wrong direction**

**Cause:** Incorrect goal coordinates or heading calculation

**Fix:**
1. Check telemetry "Angle Error" value
2. If sign is wrong, check goal position matches your alliance
3. Verify alliance variable is set correctly

---

### **Problem: Distance shown is wrong**

**Cause:** Pedro Pathing odometry drift or incorrect goal coordinates

**Fix:**
1. Measure actual distance with tape measure
2. Compare to telemetry distance
3. If consistently off, adjust goal coordinates in `FieldPositions.java`
4. Reset Pedro Pathing pose if odometry has drifted

---

### **Problem: Recommended RPM doesn't score**

**Cause:** Velocity table needs calibration

**Fix:** Follow "Phase 2: Calibrate Velocity Table" above

---

### **Problem: Auto-align works but launcher velocity wrong**

**Cause:** Auto-velocity might be OFF

**Fix:** Press Right Bumper to toggle auto-velocity ON

---

## Competition Checklist

### **Before Match**
- [ ] Set correct alliance color in code (`BLUE` or `RED`)
- [ ] Verify battery is fully charged (low voltage affects RPM)
- [ ] Test auto-align from 2-3 positions
- [ ] Confirm velocity table is calibrated
- [ ] Practice full workflow at least once

### **During Match**
- [ ] Use auto-align for consistency
- [ ] Monitor "Aligned: YES ✓" before launching
- [ ] Check "In Launch Zone: YES" to avoid penalties
- [ ] If auto-velocity fails, switch to manual mode (D-Pad Up/Down)

### **Fallback Strategy**
If auto-align/velocity fails during match:
1. Release Left Bumper (disable auto-align)
2. Tap Right Bumper (disable auto-velocity)
3. Use manual controls as before:
   - Right stick to rotate
   - D-Pad Up/Down for speed
   - Visual alignment to goal

---

## Advanced: Physics-Based Velocity (Future Enhancement)

Currently using lookup table. For more accuracy, consider implementing projectile motion formula:

```
velocity = sqrt( g * distance² / (2 * cos²(angle) * (distance*tan(angle) - height_diff)) )
```

Where:
- g = gravity (386 in/s²)
- angle = launcher angle (~35°)
- height_diff = goal height - launcher height (46" - 17" = 29")

This requires tuning drag coefficient and spin effects.

---

## File Changes Summary

### **New Files**
1. `subsystems/LauncherAssist.java` - Core auto-alignment logic

### **Modified Files**
1. `util/FieldPositions.java` - Added goal positions and helper methods
2. `robot/RobotHardware.java` - Integrated LauncherAssist subsystem
3. `opmodes/DynaMOE_19889_TeleOp.java` - Added controls and telemetry

### **No Changes Needed**
- `subsystems/Launcher.java` - Already supports `setTargetVelocity()`
- `subsystems/Drivetrain.java` - Works with auto-rotation
- Autonomous OpModes - Not affected

---

## Questions or Issues?

If you encounter problems:
1. Check telemetry for error messages
2. Review this guide's troubleshooting section
3. Test individual components (alignment only, velocity only)
4. Use manual mode as fallback during matches

Good luck at competition! 🚀
