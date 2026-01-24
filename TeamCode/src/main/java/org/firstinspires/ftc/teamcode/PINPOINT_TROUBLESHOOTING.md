# Pinpoint Odometry Troubleshooting Guide

## Overview
Your robot uses **GoBilda Pinpoint** odometry with **two dead wheels** for localization instead of an IMU. This is used by Pedro Pathing for field-centric drive and LauncherAssist auto-alignment.

## Configuration Requirements

### Hardware Configuration on Driver Hub
**CRITICAL:** These must be configured exactly:

1. **Pinpoint Device**
   - Device name: `pinpoint` (lowercase, exact match)
   - Device type: `GoBilda Pinpoint Driver`
   - I2C port: Usually I2C Bus 0 or 1

2. **Dead Wheel Connections**
   - Forward pod (Y-axis): Connected to Pinpoint encoder port
   - Strafe pod (X-axis): Connected to Pinpoint encoder port
   - Both must be properly seated in JST connectors

### Current Configuration Values
**Location:** `pedroPathing/Constants.java:57-64`

```java
public static PinpointConstants localizerConstants = new PinpointConstants()
    .forwardPodY(-3.5875)          // Forward pod offset from robot center
    .strafePodX(-6.609375)          // Strafe pod offset from robot center
    .distanceUnit(DistanceUnit.INCH)
    .hardwareMapName("pinpoint")    // ← Must match Driver Hub config
    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
```

## Symptoms When Pinpoint Fails

### LauncherAssist is NULL
**Symptoms:**
- Telemetry shows: `LauncherAssist: ✗ Disabled (no localization)`
- Auto-align button (Left Bumper) does nothing
- No distance/angle data displayed

**Root Cause:** The `follower` object failed to initialize because Pinpoint is not working

### Follower Initialization Error
**Symptoms:**
- Telemetry shows: `Follower Error: [error message]`
- OpMode may crash during init
- Field-centric drive won't work

## Troubleshooting Steps

### Step 1: Check Driver Hub Hardware Configuration
1. Open Driver Hub app
2. Go to **Configure Robot** → **Control Hub**
3. Look for device named `pinpoint`
4. Verify it's configured as `GoBilda Pinpoint Driver`
5. Check I2C bus number (usually Bus 0)

**Fix if missing:**
- Add new I2C device
- Set name to exactly `pinpoint` (lowercase)
- Select `GoBilda Pinpoint Driver` from dropdown
- Save configuration

### Step 2: Check Physical Connections
1. **Pinpoint Board Power**
   - Connected to Control Hub I2C port
   - LED should be lit when powered

2. **Dead Wheel Encoder Cables**
   - Forward pod: Check JST connector is fully seated
   - Strafe pod: Check JST connector is fully seated
   - Cables not damaged or pinched

3. **Dead Wheel Pods**
   - Wheels spin freely
   - Spring-loaded pods make contact with floor
   - No debris blocking wheels

### Step 3: Verify Pinpoint Firmware
The Pinpoint board needs proper firmware:
- Check GoBilda documentation for firmware version
- Update if necessary using GoBilda Pinpoint Utility

### Step 4: Test Pinpoint Independently
Create a simple test OpMode to verify Pinpoint works:

```java
@TeleOp(name="Test Pinpoint", group="Test")
public class TestPinpoint extends LinearOpMode {
    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry.addData("Pinpoint Status", "Found!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            telemetry.addData("X Position", pinpoint.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y Position", pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading", Math.toDegrees(pinpoint.getPosition().getHeading()));
            telemetry.addData("Forward Encoder", pinpoint.getEncoderY());
            telemetry.addData("Strafe Encoder", pinpoint.getEncoderX());
            telemetry.update();
        }
    }
}
```

**Expected behavior:**
- Encoders should change when you manually push the robot
- Position values should update as robot moves
- If encoders stay at 0, check dead wheel connections

### Step 5: Check I2C Bus Conflicts
If you have multiple I2C devices:
- Pinpoint
- IMU (even if not used)
- Color sensors
- Distance sensors

**Potential fix:**
- Move devices to different I2C buses
- Check for I2C address conflicts

### Step 6: Verify Encoder Directions
If Pinpoint works but tracking is backwards/wrong:

Test encoder directions:
1. Push robot forward → Forward encoder should increase
2. Push robot left → Strafe encoder should increase

**If backwards, fix in Constants.java:**
```java
.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
```

## Common Error Messages

### "Device 'pinpoint' not found"
**Cause:** Hardware config doesn't have a device named `pinpoint`
**Fix:** Add `pinpoint` device to Driver Hub configuration

### "I2C device timeout"
**Cause:** Pinpoint board not responding on I2C bus
**Fix:**
- Check I2C cable connection
- Try different I2C port
- Power cycle Control Hub

### "Encoder values not changing"
**Cause:** Dead wheels not connected or not touching ground
**Fix:**
- Check JST connector is seated
- Verify pods are spring-loaded and touching floor
- Clean dead wheels of debris

### NullPointerException in createFollower()
**Cause:** Pinpoint driver couldn't initialize
**Fix:**
- Check all physical connections
- Verify hardware config name matches exactly
- Update Pinpoint firmware

## Impact on Robot Functionality

### If Pinpoint Works (Normal Operation)
✓ Field-centric drive available
✓ LauncherAssist auto-alignment works
✓ Distance-based velocity recommendations
✓ Accurate position tracking for autonomous

### If Pinpoint Fails (Degraded Mode)
✗ LauncherAssist disabled (`robot.launcherAssist == null`)
✗ No auto-alignment to goal
✗ Robot-centric drive only (no field-centric)
✓ Manual launcher control still works
✓ Basic driving still functional

## Competition Day Checklist

Before each match:
- [ ] Power cycle robot completely
- [ ] Check Pinpoint LED is solid (not blinking)
- [ ] Verify telemetry shows `Follower Status: ✓ Initialized`
- [ ] Verify telemetry shows `LauncherAssist: ✓ Ready`
- [ ] Test field-centric drive rotates correctly with field
- [ ] Test auto-align button (Left Bumper) provides rotation
- [ ] Push robot manually to verify dead wheels spin freely

## Advanced Diagnostics

### Pinpoint LED Status
- **Solid:** Working correctly
- **Blinking slowly:** Initialization issue
- **Blinking fast:** I2C communication error
- **Off:** No power

### Telemetry Debug Info
Add to TeleOp loop for detailed diagnostics:
```java
if (follower != null) {
    Pose pose = follower.getPose();
    telemetry.addData("X", pose.getX());
    telemetry.addData("Y", pose.getY());
    telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
}
```

### Check Encoder Counts
In test OpMode, verify encoder counts are reasonable:
- Should change by hundreds/thousands when robot moves 1 inch
- If counts are tiny or huge, check encoder resolution setting

## Getting Help

If problems persist:
1. Check GoBilda Pinpoint documentation
2. Review Pedro Pathing Pinpoint setup guide
3. Check FTC Discord #programming channel
4. Verify firmware is up to date

## Related Files

- `pedroPathing/Constants.java` - Pinpoint configuration
- `robot/RobotHardware.java:90-96` - LauncherAssist initialization
- `opmodes/DynaMOE_19889_TeleOp.java:116-121` - Follower creation
- `subsystems/LauncherAssist.java` - Uses follower for pose tracking

## Quick Reference

**Hardware Config Name:** `pinpoint` (must be exact)
**Dead Wheels:** 2 (forward + strafe)
**Communication:** I2C
**Used For:**
- Pedro Pathing localization
- Field-centric drive heading
- LauncherAssist position/heading
