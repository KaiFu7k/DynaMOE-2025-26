package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FieldPositions;

/**
 * LauncherAssist subsystem for auto-alignment and velocity calculation
 *
 * FEATURES:
 * - Calculates distance from robot to goal
 * - Calculates optimal heading to face goal
 * - Determines required launcher velocity based on distance
 * - Provides PID-controlled rotation commands for auto-alignment
 *
 * USAGE:
 * 1. Call update() in main loop to refresh calculations
 * 2. Call getRotationPower() to get motor powers for alignment (uses PID)
 * 3. Call isAligned() to check if robot is facing goal
 * 4. Call getRecommendedVelocity() to get launch speed for current distance
 * 5. Call resetPID() when switching between manual/auto modes
 *
 * PID TUNING GUIDE:
 * Start with current values and tune in this order:
 * 1. KP (Proportional): Increase until robot responds quickly but may oscillate
 *    - Too low: Slow response, won't reach target
 *    - Too high: Overshoots and oscillates
 *    - Start: 0.015, try range: 0.01 - 0.03
 *
 * 2. KD (Derivative): Increase to reduce oscillation/overshoot
 *    - Too low: Oscillates around target
 *    - Too high: Sluggish, jerky movement
 *    - Start: 0.003, try range: 0.001 - 0.01
 *
 * 3. KI (Integral): Add small amount to eliminate steady-state error
 *    - Too low: Won't fully reach target (small error remains)
 *    - Too high: Slow oscillation, integral windup
 *    - Start: 0.001, try range: 0.0005 - 0.003
 *
 * Testing procedure:
 * - Test at various distances and angles
 * - Check response when battery is low vs full
 * - Verify no oscillation when close to target
 * - Ensure reaches target quickly without overshoot
 */
public class LauncherAssist {

    // Constants
    private static final double ALIGNMENT_TOLERANCE_DEGREES = 3.0;  // How close is "aligned"
    private static final double ROTATION_SPEED = 0.5;  // Max speed for auto-rotation (0.0 to 1.0)
    private static final double MIN_ROTATION_POWER = 0.15;  // Minimum power to overcome static friction

    // PID Constants for rotation control
    // KP increased significantly since angleError is in radians (45° = 0.785 rad)
    private static final double KP = 0.8;  // Proportional gain (tune this first)
    private static final double KD = 0.05;  // Derivative gain (dampens oscillation)
    private static final double INTEGRAL_MAX = 0.1;  // Anti-windup: max integral contribution

    // Velocity lookup table (distance in inches -> RPM)
    // TODO: Tune these values based on real testing
    private static final double[][] VELOCITY_TABLE = {
        // {distance, RPM}
        {24, 1000},   // Very close
        {36, 1150},   // Close
        {48, 1250},   // Medium
        {60, 1350},   // Far
        {72, 1450},   // Very far
        {84, 1550}    // Max range
    };

    // Launcher physical constants
    private static final double LAUNCHER_HEIGHT_INCHES = 17.0;
    private static final double LAUNCHER_ANGLE_DEGREES = 45.0;  // Estimated from curved launcher ramp

    // State
    private Follower follower;
    private Telemetry telemetry;
    private FieldPositions.Alliance currentAlliance;

    // Calculated values (updated each cycle)
    private double distanceToGoal = 0;
    private double angleToGoal = 0;
    private double angleError = 0;
    private double recommendedVelocity = 0;
    private boolean isAligned = false;

    // PID state variables
    private double previousError = 0;
    private double integralSum = 0;
    private long lastUpdateTime = 0;
    private double lastRotationPower = 0;  // Debug: track last calculated power

    /**
     * Constructor
     * @param follower Pedro Pathing follower for pose tracking
     * @param alliance Current alliance (RED or BLUE)
     * @param telemetry Telemetry for debug output
     */
    public LauncherAssist(Follower follower, FieldPositions.Alliance alliance, Telemetry telemetry) {
        this.follower = follower;
        this.currentAlliance = alliance;
        this.telemetry = telemetry;
        this.lastUpdateTime = System.currentTimeMillis();
    }

    /**
     * Update calculations based on current robot position
     * Call this in your main loop to keep calculations fresh
     */
    public void update() {
        // Get current robot pose from Pedro Pathing
        Pose currentPose = follower.getPose();

        // Get goal position for current alliance
        Pose goalPose = FieldPositions.getGoalPosition(currentAlliance);

        // Calculate distance to goal (pythagorean theorem)
        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();
        distanceToGoal = Math.sqrt(dx * dx + dy * dy);

        // Calculate angle to goal (atan2 handles all quadrants correctly)
        angleToGoal = Math.atan2(dy, dx);

        // Calculate angle error (how far off from facing goal)
        // Note: Pedro Pathing heading is negated compared to standard math convention
        // So we add the heading instead of subtracting it
        // Normalize to -PI to +PI range
        angleError = normalizeAngle(angleToGoal + currentPose.getHeading());

        // Check if aligned (within tolerance)
        isAligned = Math.abs(Math.toDegrees(angleError)) < ALIGNMENT_TOLERANCE_DEGREES;

        // Calculate recommended velocity based on distance
        recommendedVelocity = calculateVelocity(distanceToGoal);
    }

    /**
     * Get rotation power for auto-alignment using PID control
     * Use this to rotate the robot to face the goal
     *
     * @return Rotation power for right stick X (-1.0 to 1.0)
     *         Positive = rotate counterclockwise
     *         Negative = rotate clockwise
     *         0 = aligned or not active
     */
    public double getRotationPower() {
        if (isAligned) {
            // Reset PID state when aligned to prevent integral windup
            integralSum = 0;
            previousError = 0;
            return 0.0;
        }

        // Calculate time delta for derivative and integral terms
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
        lastUpdateTime = currentTime;

        // Prevent division by zero or unreasonable dt
        if (dt <= 0 || dt > 0.5) {
            dt = 0.02;  // Default to 50Hz update rate
        }

        // Proportional term: error in radians
        double proportional = KP * angleError;

        // Integral term: accumulated error over time (with anti-windup)
        // Clamp integral to prevent windup
        // Derivative term: rate of change of error
        double derivative = KD * (angleError - previousError) / dt;
        previousError = angleError;

        // Combine PID terms
        double rotationPower = proportional + derivative;

        // Clamp to maximum rotation speed
        rotationPower = Math.max(-ROTATION_SPEED, Math.min(ROTATION_SPEED, rotationPower));

        // Add minimum power to overcome static friction (deadband compensation)
        if (Math.abs(rotationPower) > 0.01 && Math.abs(rotationPower) < MIN_ROTATION_POWER) {
            rotationPower = Math.signum(rotationPower) * MIN_ROTATION_POWER;
        }

        // Debug: store for telemetry
        lastRotationPower = rotationPower;

        return rotationPower;
    }

    /**
     * Calculate required launcher velocity for given distance
     * Uses linear interpolation between lookup table values
     *
     * @param distance Distance to goal in inches
     * @return Recommended launcher velocity in RPM
     */
    private double calculateVelocity(double distance) {
        // Handle edge cases
        if (distance <= VELOCITY_TABLE[0][0]) {
            return VELOCITY_TABLE[0][1];  // Minimum distance -> minimum velocity
        }
        if (distance >= VELOCITY_TABLE[VELOCITY_TABLE.length - 1][0]) {
            return VELOCITY_TABLE[VELOCITY_TABLE.length - 1][1];  // Max distance -> max velocity
        }

        // Find the two points to interpolate between
        for (int i = 0; i < VELOCITY_TABLE.length - 1; i++) {
            double dist1 = VELOCITY_TABLE[i][0];
            double dist2 = VELOCITY_TABLE[i + 1][0];

            if (distance >= dist1 && distance <= dist2) {
                // Linear interpolation
                double vel1 = VELOCITY_TABLE[i][1];
                double vel2 = VELOCITY_TABLE[i + 1][1];

                double ratio = (distance - dist1) / (dist2 - dist1);
                return vel1 + ratio * (vel2 - vel1);
            }
        }

        // Fallback (should never reach here)
        return 1250;
    }

    /**
     * Normalize angle to range -PI to +PI
     * @param angle Angle in radians
     * @return Normalized angle in radians
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Check if robot is aligned to goal
     * @return True if within alignment tolerance
     */
    public boolean isAligned() {
        return isAligned;
    }

    /**
     * Get current distance to goal
     * @return Distance in inches
     */
    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    /**
     * Get current angle error
     * @return Angle error in degrees (positive = need to rotate CCW)
     */
    public double getAngleErrorDegrees() {
        return Math.toDegrees(angleError);
    }

    /**
     * Get recommended launcher velocity for current position
     * @return Velocity in RPM
     */
    public double getRecommendedVelocity() {
        return recommendedVelocity;
    }

    /**
     * Get minimum acceptable velocity (5% below target)
     * @return Minimum velocity in RPM
     */
    public double getMinimumVelocity() {
        return recommendedVelocity * 0.95;
    }

    /**
     * Change alliance (if robot switches sides)
     * @param alliance New alliance
     */
    public void setAlliance(FieldPositions.Alliance alliance) {
        this.currentAlliance = alliance;
    }

    /**
     * Check if robot is in valid launch zone
     * @return True if robot is inside launch zone boundaries
     */
    public boolean isInLaunchZone() {
        Pose currentPose = follower.getPose();
        return FieldPositions.isInLaunchZone(currentPose);
    }

    /**
     * Reset PID controller state
     * Call this when switching between manual and auto-align modes
     * or when you want to clear accumulated integral error
     */
    public void resetPID() {
        integralSum = 0;
        previousError = 0;
        lastUpdateTime = System.currentTimeMillis();
    }

    /**
     * Get the last calculated rotation power (for telemetry/debug)
     */
    public double getLastRotationPower() {
        return lastRotationPower;
    }

    /**
     * Get the target angle to goal (for telemetry/debug)
     * @return Target angle in degrees
     */
    public double getTargetAngleDegrees() {
        return Math.toDegrees(angleToGoal);
    }

    /**
     * Get the current robot heading (for telemetry/debug)
     * @return Current heading in degrees
     */
    public double getCurrentHeadingDegrees() {
        return Math.toDegrees(follower.getPose().getHeading());
    }

    /**
     * Update telemetry with launcher assist status
     */
    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== LAUNCHER ASSIST ===");
        telemetry.addData("Distance", "%.1f in", distanceToGoal);
        telemetry.addData("Angle Error", "%.1f°", Math.toDegrees(angleError));
        telemetry.addData("Rotation Power", "%.3f", lastRotationPower);
        telemetry.addData("Aligned", isAligned ? "YES" : "NO");
        telemetry.addData("Recommended RPM", "%.0f", recommendedVelocity);
        telemetry.addData("In Launch Zone", isInLaunchZone() ? "YES" : "NO");
    }
}
