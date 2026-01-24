# Autonomous MOTIF Removal - Changelog

**Date:** 2026-01-24
**Change:** Removed MOTIF pattern checking from autonomous - now shoots all artifacts regardless of pattern

---

## What Changed

### Strategy Update
**OLD:** Check MOTIF pattern, shoot artifacts in specific order to match pattern
**NEW:** Shoot all 3 preloaded artifacts into goal (any order, ignore MOTIF)

### Why This Change?
- **Simplifies autonomous** - No pattern detection needed
- **Faster execution** - No time spent on MOTIF checking
- **More reliable** - No dependency on camera/vision detection
- **Guaranteed points** - All artifacts score (even if pattern is wrong)

---

## Technical Changes Made

### 1. Removed MotifDetector Initialization
**Before:**
```java
private MotifDetector motifDetector;

motifDetector = new MotifDetector(telemetry);
motifDetector.init(hardwareMap);
Motif detectedMotif = motifDetector.detectMotif();
```

**After:**
```java
// MotifDetector removed - no longer checking patterns
```

### 2. Updated Shooting Logic
**Before (Pattern-Based):**
```java
private void scoreAllArtifacts(Motif motif) {
    // Get pattern sequence (GPP, PGP, PPG)
    ArtifactColor[] pattern = robot.artifactManager.getPatternSequence(motif);

    // Launch artifacts in pattern order
    for (int i = 0; i < pattern.length; i++) {
        ArtifactColor targetColor = pattern[i];
        LauncherSide side = robot.artifactManager.findArtifactSide(targetColor);
        scoreArtifact(side, targetColor, i + 1);
    }
}
```

**After (Simple Order):**
```java
private void scoreAllArtifacts() {
    // Simple shooting order: LEFT, RIGHT, LEFT
    LauncherSide[] shootingOrder = {
        LauncherSide.LEFT,
        LauncherSide.RIGHT,
        LauncherSide.LEFT
    };

    // Launch all artifacts in simple order
    for (int i = 0; i < shootingOrder.length; i++) {
        LauncherSide side = shootingOrder[i];
        scoreArtifact(side, i + 1);
    }
}
```

### 3. Removed MOTIF Configuration
**Before:**
- Y button cycled MOTIF pattern during init
- Telemetry displayed selected MOTIF
- Configuration required MOTIF selection

**After:**
- Y button removed from controls
- Telemetry shows "SHOOT ALL ARTIFACTS"
- Configuration only requires position selection

### 4. Updated Method Signatures
**Before:**
```java
private void executeGoalSideAuto(Motif motif)
private void executePerimeterSideAuto(Motif motif)
private void scoreAllArtifacts(Motif motif)
private void scoreArtifact(LauncherSide side, ArtifactColor color, int artifactNum)
```

**After:**
```java
private void executeGoalSideAuto()  // No MOTIF parameter
private void executePerimeterSideAuto()  // No MOTIF parameter
private void scoreAllArtifacts()  // No MOTIF parameter
private void scoreArtifact(LauncherSide side, int artifactNum)  // No color tracking
```

---

## New Autonomous Behavior

### Shooting Order
**Fixed sequence: LEFT → RIGHT → LEFT**

This shoots all 3 preloaded artifacts:
- Artifact 1: From LEFT launcher
- Artifact 2: From RIGHT launcher
- Artifact 3: From LEFT launcher (uses second artifact in left queue)

### Default Preload Configuration
Assuming default preload (configured in ArtifactManager):
```
LEFT launcher:  [Green, Purple]  (2 artifacts)
RIGHT launcher: [Green]          (1 artifact)
```

**Shooting sequence:**
1. LEFT → Green artifact (from left launcher)
2. RIGHT → Green artifact (from right launcher)
3. LEFT → Purple artifact (from left launcher)

**Result:** All 3 artifacts scored in goal

---

## Match Day Workflow (Updated)

### Before Match
1. Place robot at chosen starting position
2. Select "DynaMOE 19889 Auto" from Driver Hub
3. Press **INIT**
4. Use **D-Pad** to select position:
   - UP = Blue Goal Side
   - DOWN = Blue Perimeter Side
   - LEFT = Red Goal Side
   - RIGHT = Red Perimeter Side
5. Press **A** to confirm
6. Wait for field **START**

**NOTE:** No MOTIF selection needed anymore!

### During Match
Autonomous will:
1. Navigate to launch zone (if starting from goal side)
2. Spin up launchers
3. Shoot artifact 1 (LEFT)
4. Shoot artifact 2 (RIGHT)
5. Shoot artifact 3 (LEFT)
6. Move off launch line (LEAVE points)

---

## Configuration Screen Changes

### OLD Configuration Display
```
=== DynaMOE 19889 AUTON CONFIG ===

Selected Position: BLUE_GOAL_SIDE
Status: Press A to confirm

MOTIF (manual): GPP

Left Slot: [GREEN, PURPLE]
Right Slot: [GREEN]

--- CONTROLS ---
Y: Cycle MOTIF (GPP/PGP/PPG)
A: CONFIRM Selection
```

### NEW Configuration Display
```
=== DynaMOE 19889 AUTON CONFIG ===

Selected Position: BLUE_GOAL_SIDE
Status: Press A to confirm

Strategy: SHOOT ALL ARTIFACTS
MOTIF Check: DISABLED

Artifacts to Score: 3

--- CONTROLS ---
A: CONFIRM Selection
```

---

## Telemetry Changes

### During Init
**Before:**
```
Position: BLUE_GOAL_SIDE
Status: CONFIRMED
Detected MOTIF: GPP
```

**After:**
```
Position: BLUE_GOAL_SIDE
Status: CONFIRMED
Strategy: SHOOT ALL ARTIFACTS (ignore MOTIF)
```

### During Match
**Before:**
```
=== SCORING ARTIFACTS ===
Target MOTIF: GPP
Pattern: [GREEN, PURPLE, PURPLE]
Scoring Artifact: 1 / 3
Color: GREEN
Using: LEFT
```

**After:**
```
=== SCORING ARTIFACTS ===
Strategy: SHOOT ALL
MOTIF Check: DISABLED
Scoring Artifact: 1 / 3
Color: GREEN
Using: LEFT
```

---

## Impact on Scoring

### MOTIF Bonus Points
**Status:** LOST (intentional trade-off)

**OLD Strategy:**
- If MOTIF correct: Score 3 artifacts + MOTIF bonus
- If MOTIF wrong: Score 0-3 artifacts (pattern might not complete)

**NEW Strategy:**
- Always score all 3 artifacts
- Never get MOTIF bonus
- **Guaranteed points** more valuable than **risky bonus**

### Point Calculation

**Artifact Points (guaranteed):**
- 3 artifacts scored = 3 × [points per artifact]

**MOTIF Bonus (lost):**
- MOTIF pattern bonus = 0 points

**LEAVE Points (guaranteed):**
- Robot exits launch line = LEAVE points

**Total:** Lower ceiling, higher floor (more consistent)

---

## Advantages of This Approach

### 1. Reliability
✅ No camera/vision dependency
✅ No pattern detection failures
✅ No color sensor errors
✅ Simple, predictable behavior

### 2. Consistency
✅ Same behavior every match
✅ No variance based on lighting
✅ No variance based on camera position
✅ Easier to debug and test

### 3. Speed
✅ No time spent on MOTIF detection
✅ Faster autonomous completion
✅ More time for other tasks

### 4. Simplicity
✅ Easier for drivers to understand
✅ Less code = fewer bugs
✅ Easier to maintain

---

## Disadvantages (Trade-offs)

### 1. Lower Maximum Score
❌ No MOTIF bonus points
❌ Lower ceiling for autonomous score

### 2. Less Game-Specific
❌ Not optimizing for DECODE game mechanics
❌ Missing out on bonus scoring opportunity

### 3. Strategic Inflexibility
❌ Can't adapt to detected patterns
❌ Fixed strategy regardless of game state

---

## When to Use This Approach

### ✅ Use This (Shoot All) When:
- MOTIF detection is unreliable or untested
- Camera/vision system has issues
- Simplicity and consistency are priorities
- Team is new to FTC or autonomous programming
- Time is limited for testing
- Competition environment is unpredictable

### ❌ Don't Use This (Use MOTIF) When:
- MOTIF detection is working reliably
- MOTIF bonus points are critical for winning
- Team has extensive testing and tuning time
- Camera setup is robust and tested
- Competition field has good lighting

---

## Reverting to MOTIF-Based Autonomous

If you want to go back to checking MOTIF patterns:

### 1. Restore MotifDetector
```java
private MotifDetector motifDetector;

motifDetector = new MotifDetector(telemetry);
motifDetector.init(hardwareMap);
```

### 2. Restore Detection Logic
```java
Motif detectedMotif = motifDetector.detectMotif();
executeGoalSideAuto(detectedMotif);  // Pass MOTIF
```

### 3. Restore Pattern-Based Scoring
```java
private void scoreAllArtifacts(Motif motif) {
    ArtifactColor[] pattern = robot.artifactManager.getPatternSequence(motif);
    // ... original pattern-following logic
}
```

### 4. Restore Y Button Control
```java
if (gamepad1.y) {
    motifDetector.cycleMotif();
    sleep(200);
}
```

**OR:** Use git to revert to previous version:
```bash
git checkout <commit-hash> -- opmodes/DynaMOE_19889_Auton.java
```

---

## Files Modified

**File:** `opmodes/DynaMOE_19889_Auton.java`

**Lines Changed:**
- Removed: MotifDetector import (line 37)
- Removed: MotifDetector initialization (~lines 94-96)
- Removed: MOTIF detection (~line 120)
- Removed: Y button MOTIF cycling (~lines 173-176)
- Modified: executeGoalSideAuto() - removed MOTIF parameter
- Modified: executePerimeterSideAuto() - removed MOTIF parameter
- Modified: scoreAllArtifacts() - simple LEFT/RIGHT/LEFT order
- Modified: scoreArtifact() - removed color parameter
- Modified: displayConfiguration() - removed MOTIF display
- Modified: telemetry throughout - updated messages

**Total Changes:** ~100 lines modified/removed

---

## Testing Checklist

Before competition, verify:

```
□ Robot shoots artifact from LEFT launcher (artifact 1)
□ Robot shoots artifact from RIGHT launcher (artifact 2)
□ Robot shoots artifact from LEFT launcher (artifact 3)
□ All 3 artifacts land in goal
□ Robot exits launch line after scoring
□ Works from BLUE GOAL SIDE position
□ Works from BLUE PERIMETER SIDE position
□ Works from RED GOAL SIDE position
□ Works from RED PERIMETER SIDE position
□ Launcher spins up correctly
□ No crashes or errors during autonomous
```

---

## Competition Day Notes

### Pre-Match
- Verify artifacts loaded: 2 in LEFT, 1 in RIGHT
- Position selection is critical (no MOTIF check to catch errors)
- Confirm launcher PIDF is tuned

### During Match
- Autonomous is simpler = less likely to fail
- All artifacts will be shot regardless of pattern
- No bonus points, but guaranteed artifact points

### Strategy Notes
- Consider this a "safe" autonomous
- Focus on reliable execution over maximum points
- Good choice for elimination matches (consistency matters)

---

## Future Enhancements (If Needed)

### 1. Add Alliance-Aware Shooting
Could shoot different patterns based on alliance:
```java
LauncherSide[] shootingOrder = (alliance == Alliance.BLUE)
    ? new LauncherSide[]{LEFT, RIGHT, LEFT}   // Blue pattern
    : new LauncherSide[]{RIGHT, LEFT, RIGHT}; // Red pattern
```

### 2. Add Distance-Based Velocity
Use LauncherAssist to adjust velocity based on launch position:
```java
double distance = /* calculate distance to goal */;
double velocity = robot.launcherAssist.calculateVelocity(distance);
robot.launcher.setVelocity(velocity);
```

### 3. Add Multiple Artifact Cycles
If time allows, could intake and score additional artifacts:
```java
// After scoring preload
intakeAndScore();  // Pick up 1 more artifact
intakeAndScore();  // Pick up 1 more artifact
```

---

## Summary

**What:** Removed MOTIF pattern checking from autonomous
**Why:** Simplify autonomous, improve reliability, guarantee points
**Trade-off:** Lost MOTIF bonus points, but gained consistency
**Result:** Simpler, faster, more reliable autonomous that always scores all 3 preloaded artifacts

**Recommendation:** Use this approach unless MOTIF detection is fully tested and reliable. Consistency beats bonus points in most competition scenarios.

---

**Questions?** See CLAUDE.md for architecture overview or ask your programming mentor!
