# Launcher Speed Control Guide (Plan A - Manual)

## Quick Start

**New Feature:** You can now adjust launcher speed in real-time using D-Pad Up/Down!

### Basic Usage (Gamepad 1 Only)

1. **Before launching:**
   - Press **D-Pad Up** to increase speed (+50 RPM per press)
   - Press **D-Pad Down** to decrease speed (-50 RPM per press)
   - Watch telemetry: "Target Speed: XXXX RPM"

2. **Spin up:**
   - Press **A** to start launchers at current speed

3. **While spinning:**
   - Can still adjust with D-Pad Up/Down (applies immediately!)
   - Watch "Ready: YES" indicator
   - Current RPM shown for both wheels

4. **Feed:**
   - Press **X** for left, **Y** for right (when ready)

5. **Stop:**
   - Press **B** to stop launchers

### Speed Range

- **Minimum:** 800 RPM (safety limit)
- **Default:** 1200 RPM (close shot)
- **Maximum:** 1600 RPM (safety limit)
- **Increment:** 50 RPM per button press

### Quick Reference Speed Chart

Use this as a starting point (tune with testing):

| Distance | Recommended RPM | D-Pad Presses from 1200 |
|----------|----------------|------------------------|
| 24-36"   | 1100 RPM       | Down x 2               |
| 36-48"   | 1200 RPM       | (Default)              |
| 48-60"   | 1300 RPM       | Up x 2                 |
| 60-72"   | 1400 RPM       | Up x 4                 |
| 72-84"   | 1500 RPM       | Up x 6                 |
| 84"+     | 1550 RPM       | Up x 7                 |

## Advanced: 2-Driver Setup (Optional)

If you have Gamepad 2 (operator), use quick presets:

- **D-Pad Up:** Far shot (1350 RPM)
- **D-Pad Down:** Close shot (1200 RPM)
- **D-Pad Left:** Max range (1550 RPM)
- **D-Pad Right:** Min safe (900 RPM)

Presets apply instantly, even while launchers are spinning.

## Tips for Tomorrow's Scrimmage

### Before Match

1. **Test at different distances:**
   - Stand at 3-4 different spots on field
   - Try different speeds, note what works
   - Fill in your own speed chart

2. **Practice speed adjustment:**
   - Get muscle memory for D-Pad Up/Down
   - Practice adjusting while moving
   - Practice mid-shot corrections

### During Match

1. **Start conservative:**
   - Begin at 1200 RPM (default)
   - Adjust based on first shot result
   - Too short? Press Up a few times
   - Too far? Press Down

2. **Trust the telemetry:**
   - "Target Speed" = what you've set
   - "Left/Right RPM" = actual motor speeds
   - "Ready: YES" = safe to feed

3. **Common adjustments:**
   - Overshoot by 6 inches? Try -100 RPM (Down x 2)
   - Undershoot by 6 inches? Try +100 RPM (Up x 2)
   - Adjust in small increments

## Troubleshooting

### "Speed won't change!"
- Check if you hit min (800) or max (1600) limit
- Telemetry will show you're at boundary

### "Launcher not reaching target speed"
- Check battery voltage (low battery = can't reach high RPM)
- Try reducing target speed
- Verify both motors showing similar RPM

### "Ready never shows YES"
- Current speed might be too high for battery voltage
- Try reducing by 100-200 RPM
- Check that both wheels are spinning

### "Can't remember current speed"
- Look at telemetry: "Target Speed: XXXX RPM"
- Always displayed, even when launchers stopped

## Testing Data Sheet

Use this during practice to record what works:

| Distance (inches) | Target RPM | Result | Notes |
|-------------------|------------|--------|-------|
|                   |            |        |       |
|                   |            |        |       |
|                   |            |        |       |
|                   |            |        |       |
|                   |            |        |       |
|                   |            |        |       |

**Result codes:**
- ✓ = Made it
- ↑ = Too high
- ↓ = Too short
- ← = Too left
- → = Too right

## Future Enhancements

After you gather testing data, we can implement:
- **Plan B (Zones):** Automatic speed based on position
- **Plan C (Physics):** Calculated speed from distance

For now, manual control gives you maximum flexibility while you learn optimal speeds!

## Controls Summary Card

**Print this and tape to Driver Station:**

```
╔══════════════════════════════════════╗
║   LAUNCHER SPEED CONTROL (PLAN A)   ║
╠══════════════════════════════════════╣
║  D-Pad UP    = +50 RPM              ║
║  D-Pad DOWN  = -50 RPM              ║
║                                      ║
║  Range: 800 - 1600 RPM              ║
║  Default: 1200 RPM                   ║
║                                      ║
║  Watch: "Target Speed" in telemetry ║
╚══════════════════════════════════════╝
```

Good luck at your scrimmage tomorrow! 🚀
