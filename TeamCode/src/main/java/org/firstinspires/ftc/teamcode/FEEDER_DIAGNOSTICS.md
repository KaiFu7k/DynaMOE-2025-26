# Feeder Servo Diagnostic Guide

## What We Added

Added comprehensive telemetry to diagnose feeder servo issues in TeleOp.

## New Telemetry Display

When launchers are active, you'll now see:

```
--- LAUNCHER STATUS ---
Launcher: ACTIVE
Target Speed: 1200 RPM
  Left: 1198 RPM
  Right: 1173 RPM
  Ready: YES
  Feeding: YES/NO
  Feed Progress: 50% (if feeding)
  X Pressed: YES/NO
  Y Pressed: YES/NO
  Can Feed: YES/NO
```

## How Feeding Works

### The Three Required Conditions

For feeding to activate, ALL three must be true:
1. **X or Y button pressed** (gamepad1.x or gamepad1.y)
2. **Launchers active** (launcherActive = true, set by pressing A)
3. **Launchers ready** (robot.launcher.isReady() returns true)

### Code Flow

```
gamepad1.x → TeleOp:361 → robot.launcher.startFeed(LEFT)
                        ↓
                   Launcher.java:171
                        ↓
                   leftFeeder.setPower(1.0)
                        ↓
                   leftFeederTimer.reset()
                        ↓
                   leftFeeding = true
```

### Auto-Stop Mechanism

Every loop cycle (~100Hz):
```
TeleOp:148 → robot.updateSubsystems()
                  ↓
          RobotHardware:123 → launcher.update()
                                    ↓
                              Launcher.java:211-220
                                    ↓
                        Check if timer >= 0.80 seconds
                                    ↓
                        If yes: setPower(0), leftFeeding=false
```

## Diagnostic Steps

### Step 1: Check All Three Conditions

1. Run TeleOp
2. Press A to spin up launchers
3. Wait for "Ready: YES"
4. Press and HOLD X button
5. Watch telemetry:

**Expected:**
```
X Pressed: YES
Can Feed: YES
Feeding: YES
```

**If you see:**
```
X Pressed: YES
Can Feed: NO
Feeding: NO
```
→ **Problem: Launchers not reaching target speed**
   - Check battery voltage (low battery = can't reach RPM)
   - Reduce target speed with D-Pad Down
   - Verify both motors show similar RPM

**If you see:**
```
X Pressed: YES
Can Feed: YES
Feeding: NO
```
→ **Problem: startFeed() not being called or failing**
   - Check logs for "Feeding LEFT" or "Feeding RIGHT" message
   - Proceed to Step 2

### Step 2: Check Hardware Configuration

Open Driver Hub → Configure → Active Configuration

**Required servo names (EXACT match):**
- `left_feeder` (CRServo)
- `right_feeder` (CRServo)
- `diverter` (Servo)

**Common mistakes:**
- Using DcMotor instead of CRServo
- Typo in name: "left_feed" vs "left_feeder"
- Wrong type: Servo instead of CRServo

### Step 3: Check Servo Controller

1. In Driver Hub configuration, note which port left_feeder is connected to
2. Is the servo controller showing as connected? (green indicator)
3. Try moving servos manually in Driver Hub test mode
4. If servos don't move in test mode → Hardware problem (controller not programmed, bad cable, bad servo)

### Step 4: Check Servo Direction

The code sets:
```java
rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
```

This means:
- `leftFeeder.setPower(1.0)` → Left servo spins forward at full speed
- `rightFeeder.setPower(1.0)` → Right servo spins backward at full speed (reversed)

**To test:**
1. When "Feeding: YES" appears, physically look at servos
2. Are they spinning?
3. Are they spinning in the correct direction to feed artifacts?

**If spinning wrong direction:**
- For left feeder: Add `leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);` at Launcher.java:79
- For right feeder: Remove or change line 79 to FORWARD

### Step 5: Check Initialization

Look for these messages in Driver Hub telemetry during initialization:

```
[RobotHardware] Initializing...
[Drivetrain] Initialized
[Launcher] Initialized
[Intake] Initialized
```

**If "[Launcher] Initialized" is missing:**
- Hardware config error (names don't match)
- Servo controller not detected
- Check Driver Hub logs for error messages

## Testing Procedure

### Quick Test (No artifacts needed)

1. Run TeleOp
2. Press A (launchers should spin up)
3. Wait 2-3 seconds for "Ready: YES"
4. Press X and watch telemetry:
   - Should show "Feeding: YES"
   - Should show "Feed Progress: 0% → 100%"
   - Should show "Feeding: NO" after ~0.8 seconds
5. Look at left feeder servo:
   - Should spin for 0.8 seconds
   - Should stop automatically

### Full Test (With artifacts)

1. Load one artifact into intake
2. Run TeleOp
3. Press A (launchers spin up)
4. Wait for "Ready: YES"
5. Press X (feed left)
6. Artifact should:
   - Move through feeder
   - Launch from left launcher
   - Exit robot

## Common Issues

### Issue: "Can Feed: NO" even after 5+ seconds

**Cause:** Launchers not reaching target velocity

**Solutions:**
1. Reduce target speed:
   - Press D-Pad Down 2-4 times (100-200 RPM reduction)
   - Try again
2. Check battery voltage:
   - Low battery can't provide enough power for high RPM
   - Replace battery
3. Check motor wiring:
   - Verify both launcher motors connected
   - Check for loose connections

### Issue: "Feeding: YES" but servos don't spin

**Cause:** Hardware configuration or connection problem

**Solutions:**
1. **Check servo controller connection:**
   - In Driver Hub, go to Manage → Control Hub
   - Look for servo controller (should be green)
   - If red/yellow: Power cycle robot
2. **Verify configuration:**
   - Go to Configure → Edit
   - Find left_feeder and right_feeder
   - Type should be: CRServo (NOT Servo, NOT DcMotor)
3. **Test in Driver Hub:**
   - Use built-in servo test
   - Manually set power to 1.0
   - If doesn't move: Hardware fault (bad servo, bad cable, bad controller)

### Issue: Servos spin but wrong direction

**Cause:** Servo direction needs to be reversed

**Fix:**
Edit Launcher.java line 79:
```java
// If left feeder needs reverse:
leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

// If right feeder needs forward instead of reverse:
rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
```

### Issue: Servos don't stop after 0.8 seconds

**Cause:** `robot.updateSubsystems()` not being called

**Check:**
1. Verify TeleOp line 148 has: `robot.updateSubsystems();`
2. Verify RobotHardware line 123 has: `launcher.update();`

If both are present and still not working:
- Add a manual stop: Press B to stop launchers (also stops feeders)

## Hardware Config Reference

### Correct Configuration (Driver Hub)

```
Control Hub Portal
└── left_launcher (DcMotorEx, port 0)
└── right_launcher (DcMotorEx, port 1)
└── left_front_drive (DcMotor, port 2)
└── right_front_drive (DcMotor, port 3)
└── left_back_drive (DcMotor, port 0)
└── right_back_drive (DcMotor, port 1)
└── intake (DcMotor, port 2)

Servo Controller
└── left_feeder (CRServo, port 0)
└── right_feeder (CRServo, port 1)
└── diverter (Servo, port 2)
```

## Code Reference

### Key Files

**TeleOp feeding logic:** `opmodes/DynaMOE_19889_TeleOp.java` lines 356-372
```java
if (gamepad1.x && launcherActive && robot.launcher.isReady()) {
    robot.launcher.startFeed(LauncherSide.LEFT);
}
```

**Launcher startFeed():** `subsystems/Launcher.java` lines 171-185
```java
public void startFeed(LauncherSide side) {
    if (side == LauncherSide.LEFT || side == LauncherSide.BOTH) {
        leftFeeder.setPower(FULL_SPEED);  // FULL_SPEED = 1.0
        leftFeederTimer.reset();
        leftFeeding = true;
    }
    // ...
}
```

**Launcher update():** `subsystems/Launcher.java` lines 211-220
```java
public void update() {
    if (leftFeeding && leftFeederTimer.seconds() >= FEED_TIME_SECONDS) {
        leftFeeder.setPower(STOP_SPEED);
        leftFeeding = false;
    }
    // ...
}
```

## What to Report Back

After testing with the new telemetry, please report:

1. **All Three Conditions:**
   - X Pressed: YES/NO
   - Can Feed: YES/NO
   - Feeding: YES/NO

2. **Physical Behavior:**
   - Do servos physically spin? YES/NO
   - If yes, correct direction? YES/NO
   - Do they auto-stop after 0.8s? YES/NO

3. **Hardware Config:**
   - Are left_feeder and right_feeder configured as CRServo? YES/NO
   - Is servo controller showing as connected (green)? YES/NO
   - Do servos work in Driver Hub test mode? YES/NO

4. **Any Error Messages:**
   - During initialization?
   - When pressing X/Y buttons?
   - In Driver Hub logs?

This information will help pinpoint the exact issue!
