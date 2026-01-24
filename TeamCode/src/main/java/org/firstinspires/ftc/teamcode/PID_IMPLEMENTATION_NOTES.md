# PID Controller Implementation for Auto-Alignment

## Changes Made

### 1. LauncherAssist.java - Added Full PID Controller
**Location:** `subsystems/LauncherAssist.java`

#### Added Constants
```java
private static final double KP = 0.015;          // Proportional gain
private static final double KI = 0.001;          // Integral gain  
private static final double KD = 0.003;          // Derivative gain
private static final double INTEGRAL_MAX = 0.1;  // Anti-windup limit
```

#### Added State Variables
```java
private double previousError = 0;      // For derivative calculation
private double integralSum = 0;        // For integral accumulation
private long lastUpdateTime = 0;       // For dt calculation
```

#### Replaced getRotationPower() Method
- **Old:** Simple proportional control with fixed scaling
- **New:** Full PID control with:
  - **P term:** Responds proportionally to error
  - **I term:** Eliminates steady-state error (with anti-windup protection)
  - **D term:** Dampens oscillation and overshoot
  - **Time-based calculation:** Uses actual dt for accurate I and D terms
  - **Auto-reset:** Clears PID state when aligned to prevent windup

#### Added resetPID() Method
Resets all PID state variables. Called when:
- Switching from manual to auto-align
- Switching from auto-align to manual
- Robot becomes aligned (automatic)

### 2. DynaMOE_19889_TeleOp.java - Added PID Reset Calls
**Location:** `opmodes/DynaMOE_19889_TeleOp.java`

Added `robot.launcherAssist.resetPID()` calls in `handleAutoAlignControls()`:
- When engaging auto-align (Left Bumper pressed)
- When disengaging auto-align (Left Bumper pressed again)

This prevents accumulated integral error from affecting the next alignment attempt.

## How PID Works

### Proportional (P) Term
- `P = KP * error`
- Responds to current error
- Larger error → stronger correction
- **Alone:** Fast response but may oscillate or not reach target

### Integral (I) Term  
- `I = KI * Σ(error * dt)`
- Accumulates error over time
- Eliminates steady-state error (small persistent offset)
- **Anti-windup:** Clamped to prevent overshooting when error is large for too long
- **Alone:** Very slow, can cause instability

### Derivative (D) Term
- `D = KD * (error - previousError) / dt`
- Responds to rate of change of error
- Dampens oscillation and overshoot
- Predicts future error trend
- **Alone:** Only responds to changes, won't correct static error

### Combined PID
```
output = P + I + D
```
- **P:** Gets you close to target quickly
- **D:** Prevents overshoot and oscillation
- **I:** Eliminates final small errors

## Tuning Guide

### Step 1: Tune KP (Proportional)
1. Set KI = 0, KD = 0
2. Start with KP = 0.01
3. Increase KP until robot responds quickly but oscillates
4. Reduce KP by 20-30%
5. **Current value:** 0.015

**Signs of bad KP:**
- Too low: Slow response, won't reach target
- Too high: Overshoots, oscillates back and forth

### Step 2: Tune KD (Derivative)  
1. Keep KP from Step 1, KI = 0
2. Start with KD = 0.001
3. Increase KD to reduce oscillation
4. Stop when movement becomes too sluggish
5. **Current value:** 0.003

**Signs of bad KD:**
- Too low: Still oscillates around target
- Too high: Sluggish, jerky movement, sensitive to noise

### Step 3: Tune KI (Integral)
1. Keep KP and KD from previous steps
2. Start with KI = 0.0005
3. Increase KI until steady-state error disappears
4. If slow oscillation appears, reduce KI
5. **Current value:** 0.001

**Signs of bad KI:**
- Too low: Small error remains, never quite reaches target
- Too high: Slow oscillation, overshoot after reaching target

### Testing Checklist
- [ ] Test at various starting angles (30°, 90°, 180°)
- [ ] Test at various distances from goal
- [ ] Test with full battery
- [ ] Test with low battery (below 12V)
- [ ] Verify no oscillation when close to target
- [ ] Verify reaches target within 3° consistently
- [ ] Test rapid on/off toggling of auto-align
- [ ] Check response time (should align in < 1 second)

## Expected Behavior

### Good PID Tuning
1. Robot starts rotating toward goal
2. Rotation speed decreases as it approaches target angle
3. Smoothly comes to rest at target (no overshoot)
4. Stays at target angle without drifting
5. No oscillation or "hunting" around target

### Signs of Poor Tuning

**Oscillation:** Robot rotates back and forth around target
- **Fix:** Increase KD, decrease KP

**Overshoot:** Robot rotates past target, then corrects
- **Fix:** Increase KD, decrease KP  

**Slow Response:** Takes too long to reach target
- **Fix:** Increase KP

**Steady-State Error:** Stops close but not at target
- **Fix:** Increase KI (slightly)

**Windup:** Overshoots badly after being misaligned for a while
- **Fix:** Decrease INTEGRAL_MAX

## Advanced Tuning

### Battery Compensation
If behavior changes significantly with battery voltage:
- Add feedforward term based on battery voltage
- Scale constants based on voltage reading

### Noise Filtering
If D term causes jittery motion:
- Add low-pass filter to derivative calculation
- Use moving average of last N errors

### Adaptive Gains
For competition-level performance:
- Reduce KP when close to target (smaller errors)
- Increase KD when approaching quickly
- Disable I term when far from target (only use near alignment)

## Troubleshooting

### Problem: Robot doesn't move at all
- Check: Is `angleError` being calculated correctly?
- Check: Is `ROTATION_SPEED` too low? (should be 0.3)
- Check: Is deadband compensation active? (0.05 minimum power)

### Problem: Robot moves but never stops
- Check: Is `ALIGNMENT_TOLERANCE_DEGREES` too small? (should be 3.0)
- Check: Is `isAligned` being set correctly in `update()`?
- Check: Is PID being reset when aligned?

### Problem: Integral windup (massive overshoot)
- Check: Is `INTEGRAL_MAX` too high? (should be 0.1)
- Check: Is `integralSum` being clamped correctly?
- Check: Is PID being reset when switching modes?

### Problem: Jittery or noisy rotation
- Check: Is KD too high? (reduce it)
- Check: Is `dt` calculation reasonable? (should be 0.02-0.05 seconds)
- Check: Are there sensor noise issues with heading?

## Implementation Notes

### Why Time-Based?
The PID uses actual time delta (`dt`) for I and D calculations:
- Makes tuning independent of loop frequency
- More accurate and predictable behavior
- Handles variable loop times gracefully

### Anti-Windup Protection
Integral term is clamped to prevent:
- Large accumulated error when far from target
- Massive overshoot when finally approaching target
- Instability during mode switches

### Automatic Reset
PID state resets when:
- `isAligned == true`: Prevents drift from accumulated I term
- Mode switches: Prevents old error from affecting new alignment
- Manual calls to `resetPID()`: Allows driver to clear state

## Competition Day Tips

1. **Test before each match** - Battery voltage affects behavior
2. **Have backup values** - Keep old constants commented in code
3. **Monitor telemetry** - Watch angle error and alignment status
4. **Quick disable** - Left bumper toggles off if behavior is bad
5. **Document changes** - Note any constant adjustments between matches

## Files Modified

1. `subsystems/LauncherAssist.java` - PID implementation
2. `opmodes/DynaMOE_19889_TeleOp.java` - PID reset calls
3. `PID_IMPLEMENTATION_NOTES.md` - This file

## Testing Procedure

### Basic Test
1. Drive robot to various positions on field
2. Press Left Bumper to engage auto-align
3. Observe rotation behavior
4. Press Left Bumper to disengage
5. Repeat at different angles and distances

### Advanced Test
1. Enable telemetry logging for angle error
2. Record video of robot behavior
3. Plot angle error over time
4. Analyze overshoot, settling time, steady-state error
5. Adjust constants based on data

## Further Resources

- PID explanation: https://en.wikipedia.org/wiki/PID_controller
- FTC PID tuning: https://docs.ftclib.org/ftclib/features/controllers
- Control systems basics: https://gm0.org/en/latest/docs/software/control-loops.html
