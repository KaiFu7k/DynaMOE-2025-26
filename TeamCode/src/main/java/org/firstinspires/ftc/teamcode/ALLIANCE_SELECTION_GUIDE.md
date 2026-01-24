# Alliance Selection Guide - TeleOp

## Problem Solved

**Before:** Alliance was hardcoded in the code - required rebuilding/redeploying between matches
**Now:** Alliance can be selected via gamepad during initialization phase

---

## How Alliance Selection Works

### During Init Phase (Before Match Starts)

When you initialize the TeleOp OpMode, you'll see this screen:

```
=== ALLIANCE SELECTION ===

Current Alliance: BLUE

Press X for BLUE alliance
Press B for RED alliance

Then press DPAD UP when ready to initialize
```

### Button Controls

| Button | Action |
|--------|--------|
| **X** | Select BLUE alliance 🔵 |
| **B** | Select RED alliance 🔴 |
| **DPAD UP** | Confirm and continue with current selection |

### Selection Steps

1. **Power on robot** and select TeleOp OpMode
2. **Press INIT** (not START yet!)
3. **Alliance selection screen appears**
4. **Press X for BLUE** or **B for RED**
5. Screen confirms: "✓ BLUE Alliance Selected" or "✓ RED Alliance Selected"
6. **Robot initializes** with selected alliance
7. **Final status screen** shows alliance and "Press START to begin"
8. **Press START** when ready to begin match

---

## What Alliance Selection Affects

### 1. Auto-Alignment Goal Target
When you press **Left Bumper** to auto-align:
- **BLUE alliance** → Robot rotates toward BLUE basket (top-left corner)
- **RED alliance** → Robot rotates toward RED basket (top-right corner)

### 2. Distance-Based Velocity
When **Auto-Velocity mode** is enabled (Right Bumper):
- Distance calculated to correct goal based on alliance
- Launcher velocity adjusted for distance to YOUR goal

### 3. LauncherAssist Calculations
All LauncherAssist features use the selected alliance:
```java
// FieldPositions.java:38-39
BLUE_GOAL_POSITION = (-60, 60)  // Top-left corner
RED_GOAL_POSITION = (60, 60)    // Top-right corner
```

---

## Emergency Alliance Switch (During Match)

**USE CAREFULLY:** If alliance was selected incorrectly, you can switch during the match.

### Emergency Override Controls

**Hold DPAD LEFT + DPAD RIGHT simultaneously**

- Must hold BOTH buttons for 0.5 seconds
- Alliance toggles: BLUE ↔ RED
- LauncherAssist updates to new goal
- Log message: "Alliance SWITCHED to [NEW_ALLIANCE]"

**⚠️ WARNING:** Only use this if you selected wrong alliance by mistake!

### Why This Exists
- **Safety net** for human error during alliance selection
- **Prevents shooting at wrong goal** if selection was wrong
- **Requires deliberate action** (both buttons) to prevent accidental switching

---

## Visual Confirmation

### During Init
```
=== DynaMOE 19889 TELEOP ===
Alliance: BLUE 🔵
Robot initialized and ready!

LauncherAssist: ✓ Ready

Press START to begin
```

### During Match (Telemetry)
```
=== DYNAMOE 19889 TELEOP ===
Alliance: RED 🔴

Drive Mode: Field-Centric
Heading: 45.2°

Auto-Align: ACTIVE
Auto-Velocity: ENABLED
  Distance: 48.3 in
  Angle Error: 2.1°
  Aligned: YES ✓
  In Launch Zone: YES
```

---

## Pre-Match Checklist

**Before EVERY Match:**

```
□ Select TeleOp OpMode
□ Press INIT
□ SELECT CORRECT ALLIANCE (X=Blue, B=Red)
□ Wait for "Robot initialized and ready!"
□ VERIFY alliance shown on telemetry
   - Look for "Alliance: BLUE 🔵" or "Alliance: RED 🔴"
□ Press START when ready
```

**⚠️ CRITICAL:** Always verify the alliance displayed on Driver Hub matches your actual alliance station!

---

## Common Scenarios

### Scenario 1: Normal Match Setup
```
1. Drive team arrives at BLUE alliance station
2. Initialize TeleOp → Press INIT
3. Alliance selection screen appears
4. Driver presses X (BLUE)
5. Confirms "✓ BLUE Alliance Selected"
6. Robot initializes
7. Telemetry shows "Alliance: BLUE 🔵"
8. Press START when ready
```

### Scenario 2: Wrong Alliance Selected
```
1. Driver pressed B by mistake (wanted BLUE)
2. Robot initializes with RED alliance
3. Telemetry shows "Alliance: RED 🔴" ← WRONG!
4. BEFORE pressing START, notice the error
5. Press STOP
6. Re-initialize OpMode
7. This time press X for BLUE
8. Verify "Alliance: BLUE 🔵"
9. Press START
```

### Scenario 3: Emergency Mid-Match Switch
```
1. Match started with wrong alliance
2. Auto-align points robot at wrong goal
3. IMMEDIATELY hold DPAD LEFT + DPAD RIGHT
4. Wait 0.5 seconds (both buttons held)
5. Alliance switches
6. Telemetry updates to correct alliance
7. Auto-align now targets correct goal
```

---

## Technical Details

### How It Works

#### Alliance Selection Logic
```java
// TeleOp.java:75
private FieldPositions.Alliance alliance = FieldPositions.Alliance.BLUE;  // Default

// During init phase (lines 135-158):
while (!allianceSelected && !isStopRequested()) {
    if (gamepad1.x) {
        alliance = FieldPositions.Alliance.BLUE;
        allianceSelected = true;
    } else if (gamepad1.b) {
        alliance = FieldPositions.Alliance.RED;
        allianceSelected = true;
    } else if (gamepad1.dpad_up) {
        // Use default
        allianceSelected = true;
    }
}
```

#### Goal Position Calculation
```java
// FieldPositions.java:141-143
public static Pose getGoalPosition(Alliance alliance) {
    return (alliance == Alliance.BLUE) ? BLUE_GOAL_POSITION : RED_GOAL_POSITION;
}

// Goal coordinates:
BLUE_GOAL_POSITION = new Pose(-60, 60, 0);  // Top-left
RED_GOAL_POSITION = new Pose(60, 60, 0);    // Top-right
```

#### Emergency Switch Logic
```java
// TeleOp.java:364-372 (in handleAutoAlignControls)
if (gamepad1.dpad_left && gamepad1.dpad_right) {
    // Toggle alliance
    alliance = (alliance == FieldPositions.Alliance.BLUE)
        ? FieldPositions.Alliance.RED
        : FieldPositions.Alliance.BLUE;

    // Update LauncherAssist
    robot.launcherAssist.setAlliance(alliance);

    sleep(500);  // Prevent rapid toggling
}
```

---

## Troubleshooting

### Problem: Alliance selection screen doesn't appear
**Cause:** OpMode started instead of initialized
**Fix:** Press INIT (not START) when OpMode is selected

### Problem: Can't select alliance (buttons don't work)
**Cause:** Controller not connected or wrong gamepad
**Fix:**
- Verify gamepad1 is connected (Driver Hub shows gamepad icon)
- Make sure using gamepad1, not gamepad2

### Problem: Selected alliance but robot initialized with wrong one
**Cause:** Pressed wrong button or didn't wait for confirmation
**Fix:**
- Always wait for "✓ Alliance Selected" message
- Verify telemetry shows correct alliance before pressing START

### Problem: Auto-align points at wrong goal
**Symptoms:** Robot rotates 180° from expected direction
**Cause:** Alliance selected incorrectly
**Fix:**
- Check telemetry: Does alliance match your station?
- If wrong: STOP OpMode, re-initialize, select correct alliance
- If mid-match: Use emergency switch (DPAD LEFT + RIGHT)

### Problem: LauncherAssist is null (can't use auto features)
**Cause:** Pedro Pathing follower failed to initialize
**Fix:**
- Check PINPOINT_TROUBLESHOOTING.md
- Verify Pinpoint hardware is connected
- Alliance selection still works, but auto-align won't

---

## Important Notes

### Default Alliance
If you press **DPAD UP** without selecting X or B:
- Uses current default (BLUE by default)
- Useful if you want to keep the previously selected alliance

### No Code Changes Needed
**NEVER** need to edit code between matches anymore!
- All alliance selection happens via gamepad
- Same code works for BLUE and RED matches

### Competition Day Workflow
```
Match 1 (BLUE):  X → Initialize → START
Match 2 (RED):   B → Initialize → START
Match 3 (BLUE):  X → Initialize → START
Match 4 (RED):   B → Initialize → START
```

No rebuilding, no code changes, no deployment delays!

---

## Safety Features

### 1. Debounce Protection
- 300ms delay after button press
- Prevents accidental double-selection

### 2. Emergency Switch Delay
- Requires 0.5 seconds hold time
- Prevents accidental mid-match switching

### 3. Visual Confirmation
- Alliance always displayed in telemetry
- Color indicators (🔵 🔴) for quick recognition
- Confirmation messages during selection

### 4. Stop-Request Safety
- Can press STOP during alliance selection
- OpMode exits safely without initialization

---

## Related Files

- **TeleOp.java:75** - Alliance variable declaration
- **TeleOp.java:135-158** - Alliance selection logic
- **TeleOp.java:364-372** - Emergency switch logic
- **TeleOp.java:583** - Alliance telemetry display
- **FieldPositions.java:38-39** - Goal position constants
- **FieldPositions.java:141-143** - getGoalPosition() method
- **LauncherAssist.java:78** - Uses alliance for goal targeting

---

## Quick Reference Card (Print This!)

```
╔═══════════════════════════════════════════════╗
║       ALLIANCE SELECTION QUICK GUIDE          ║
╠═══════════════════════════════════════════════╣
║                                               ║
║  INIT PHASE:                                  ║
║  ┌─────────────────────────────────────────┐  ║
║  │ X Button     = BLUE Alliance 🔵         │  ║
║  │ B Button     = RED Alliance 🔴          │  ║
║  │ DPAD UP      = Use Default              │  ║
║  └─────────────────────────────────────────┘  ║
║                                               ║
║  EMERGENCY (Mid-Match):                       ║
║  ┌─────────────────────────────────────────┐  ║
║  │ DPAD LEFT + DPAD RIGHT (hold 0.5s)      │  ║
║  │ = Toggle Alliance                        │  ║
║  └─────────────────────────────────────────┘  ║
║                                               ║
║  ALWAYS VERIFY:                               ║
║  ✓ Telemetry shows correct alliance          ║
║  ✓ Color indicator matches station           ║
║  ✓ Before pressing START                     ║
║                                               ║
╚═══════════════════════════════════════════════╝
```

---

**Remember:** Alliance selection is now part of your pre-match routine, like checking battery voltage or verifying hardware config. Make it a habit!
