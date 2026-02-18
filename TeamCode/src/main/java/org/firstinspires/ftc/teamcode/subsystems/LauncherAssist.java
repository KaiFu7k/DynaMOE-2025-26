package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FieldPositions;

/**
 * LauncherAssist Subsystem for DynaMOE Team 19889
 *
 * PURPOSE:
 * This subsystem helps drivers aim and shoot artifacts into the goal by:
 * 1. AUTO-ALIGN: Automatically rotating the robot to face the goal
 * 2. AUTO-VELOCITY: Calculating the correct launcher speed based on distance
 *
 * HOW IT WORKS:
 * - Uses the Pinpoint odometry (via Follower) to know robot's position on field
 * - Calculates angle and distance to the goal
 * - PD controller outputs rotation power to align robot
 * - Velocity table (LERP) outputs recommended launcher RPM
 *
 * USAGE IN TELEOP:
 * 1. Call update() every loop to recalculate position/angle
 * 2. When driver enables auto-align, use getRotationPower() instead of joystick for rx
 * 3. Use getRecommendedVelocity() to set launcher speed automatically
 * 4. Check isAligned() to know when robot is aimed at goal
 */
public class LauncherAssist {

    // ==================== ALIGNMENT CONSTANTS ====================

    /** How close (in degrees) the robot needs to be to consider itself "aligned" */
    private static final double ALIGNMENT_TOLERANCE_DEGREES = 4.0;

    /** Maximum rotation speed (0 to 1) - limits how fast robot can spin during alignment */
    private static final double ROTATION_SPEED = 0.6;

    /** Minimum power to overcome static friction - robot won't move below this */
    private static final double MIN_ROTATION_POWER = 0.18;

    // ==================== PD CONTROLLER CONSTANTS ====================
    // Note: This is a PD controller (no Integral term used currently)

    /** Proportional gain - higher = faster response but more overshoot */
    private static final double KP = 1.0;

    /** Derivative gain - higher = more damping, reduces oscillation */
    private static final double KD = 0.3;

    /** Integral max (not currently used, but kept for future tuning) */
    private static final double INTEGRAL_MAX = 0.1;

    // ==================== VELOCITY TABLE ====================
    /**
     * VELOCITY LOOKUP TABLE - Maps distance (inches) to launcher velocity (RPM)
     *
     * FORMAT: {distance_in_inches, velocity_in_RPM}
     *
     * TUNED VALUES from testing on 2026-02-03
     * The code uses Linear Interpolation (LERP) between these points.
     *
     * Note: Data shows velocity plateaus around 1180 RPM for 70-90 inch range,
     * then increases steadily from 100+ inches.
     */
    private static final double[][] VELOCITY_TABLE = {
        {60, 1090},    // Close shot
        {70, 1170},    //
        {80, 1180},    // Velocity plateaus here
        {90, 1180},    // Still plateau
        {101, 1230},   // Starting to increase
        {113, 1250},   //
        {120, 1300},   //
        {130, 1320},   //
        {143, 1420},   // Steeper increase
        {150, 1490},   // Far shot
        {157, 1480}    // Max distance tested (slight decrease - may be measurement variance)
    };

    // ==================== STATE VARIABLES ====================

    private Follower follower;           // Pedro Pathing follower for odometry
    private Telemetry telemetry;         // For debug output
    private FieldPositions.Alliance currentAlliance;  // BLUE or RED (determines goal position)

    private double distanceToGoal = 0;   // Calculated distance in inches
    private double angleToGoal = 0;      // Angle FROM robot TO goal (radians)
    private double angleError = 0;       // Difference between current heading and target (radians)
    private double recommendedVelocity = 0;  // Calculated launcher RPM
    private boolean isAligned = false;   // True when robot is aimed at goal (within tolerance)

    // PD Controller state
    private double previousError = 0;    // Last loop's angle error (for derivative)
    private double integralSum = 0;      // Accumulated error (not currently used)
    private long lastUpdateTime = 0;     // Timestamp for dt calculation
    private double lastRotationPower = 0; // Last output (for telemetry/debugging)

    // ==================== CONSTRUCTOR ====================

    /**
     * Create a new LauncherAssist
     * @param follower Pedro Pathing Follower (provides robot position via odometry)
     * @param alliance Current alliance (BLUE or RED) - determines which goal to aim at
     * @param telemetry Telemetry for debug output
     */
    public LauncherAssist(Follower follower, FieldPositions.Alliance alliance, Telemetry telemetry) {
        this.follower = follower;
        this.currentAlliance = alliance;
        this.telemetry = telemetry;
        this.lastUpdateTime = System.currentTimeMillis();
    }

    // ==================== MAIN UPDATE METHOD ====================

    /**
     * UPDATE - Call this EVERY LOOP in TeleOp!
     *
     * Recalculates:
     * - Distance to goal
     * - Angle to goal
     * - Angle error (how far off we are)
     * - Whether we're aligned
     * - Recommended velocity
     *
     * Must be called before getRotationPower() or getRecommendedVelocity()
     */
    public void update() {
        // Get current robot position from odometry
        Pose currentPose = follower.getPose();

        // Get goal position based on alliance (Blue goal or Red goal)
        Pose goalPose = FieldPositions.getGoalPosition(currentAlliance);

        // Calculate vector from robot to goal
        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();

        // Distance = magnitude of vector (Pythagorean theorem)
        distanceToGoal = Math.sqrt(dx * dx + dy * dy);

        // Angle to goal = direction of vector (atan2 handles all quadrants correctly)
        // Result is in radians, 0 = right, PI/2 = up, PI = left, -PI/2 = down
        angleToGoal = Math.atan2(dy, dx);

        // Error = how far we need to rotate
        // Positive error = need to rotate counter-clockwise
        // Negative error = need to rotate clockwise
        angleError = normalizeAngle(angleToGoal - currentPose.getHeading());

        // Check if we're close enough to be considered "aligned"
        isAligned = Math.abs(Math.toDegrees(angleError)) < ALIGNMENT_TOLERANCE_DEGREES;

        // Look up recommended velocity from table
        recommendedVelocity = calculateVelocity(distanceToGoal);
    }

    // ==================== AUTO-ALIGN (PD CONTROLLER) ====================

    /**
     * GET ROTATION POWER - Returns motor power to rotate robot toward goal
     *
     * Uses a PD (Proportional-Derivative) controller:
     * - P term: Rotate faster when error is larger
     * - D term: Slow down as we approach target (prevents overshoot)
     *
     * DRIVETRAIN NOTE:
     * In our mecanum drivetrain, positive rx = clockwise rotation.
     * So if we need to rotate counter-clockwise (positive error),
     * we output NEGATIVE power (hence the negative sign on line 91).
     *
     * @return Rotation power from -ROTATION_SPEED to +ROTATION_SPEED
     *         Use this as 'rx' in drivetrain calculations when auto-align is active
     */
    public double getRotationPower() {
        // If already aligned, stop rotating and reset PD state
        if (isAligned) {
            // Don't reset previousError — keeps derivative smooth when oscillating near target
            return 0.0;
        }

        // Calculate time since last update (for derivative term)
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
        lastUpdateTime = currentTime;

        // Sanity check dt - if too small or too large, use default
        if (dt <= 0 || dt > 0.5) dt = 0.02;  // Assume ~50Hz loop rate

        // PD Controller calculation
        double proportional = KP * angleError;                    // P: proportional to error
        double derivative = KD * (angleError - previousError) / dt;  // D: rate of change
        previousError = angleError;

        // Calculate output power
        // Negative sign: positive error (CCW needed) requires negative rx (CCW rotation)
        double rotationPower = -(proportional + derivative);

        // Clamp to max rotation speed
        rotationPower = Math.max(-ROTATION_SPEED, Math.min(ROTATION_SPEED, rotationPower));

        // Apply minimum power to overcome static friction
        // Only if we're trying to move (power > 0.01) but below minimum threshold
        if (Math.abs(rotationPower) > 0.01 && Math.abs(rotationPower) < MIN_ROTATION_POWER) {
            rotationPower = Math.signum(rotationPower) * MIN_ROTATION_POWER;
        }

        lastRotationPower = rotationPower;
        return rotationPower;
    }

    // ==================== AUTO-VELOCITY (LERP TABLE) ====================

    /**
     * CALCULATE VELOCITY - Looks up launcher RPM based on distance
     *
     * Uses Linear Interpolation (LERP) between table entries:
     * - If distance is below minimum table entry, return minimum velocity
     * - If distance is above maximum table entry, return maximum velocity
     * - Otherwise, interpolate between the two nearest entries
     *
     * LERP Formula: velocity = vel1 + (distance - dist1) / (dist2 - dist1) * (vel2 - vel1)
     *
     * @param distance Distance to goal in inches
     * @return Recommended launcher velocity in RPM
     */
    private double calculateVelocity(double distance) {
        // Below minimum distance - return lowest velocity
        if (distance <= VELOCITY_TABLE[0][0]) {
            return VELOCITY_TABLE[0][1];
        }

        // Above maximum distance - return highest velocity
        if (distance >= VELOCITY_TABLE[VELOCITY_TABLE.length - 1][0]) {
            return VELOCITY_TABLE[VELOCITY_TABLE.length - 1][1];
        }

        // Find the two table entries that bracket our distance
        for (int i = 0; i < VELOCITY_TABLE.length - 1; i++) {
            double dist1 = VELOCITY_TABLE[i][0];
            double dist2 = VELOCITY_TABLE[i + 1][0];

            if (distance >= dist1 && distance <= dist2) {
                double vel1 = VELOCITY_TABLE[i][1];
                double vel2 = VELOCITY_TABLE[i + 1][1];

                // Linear interpolation between the two points
                // Example: if distance=42, dist1=36, dist2=48, vel1=1150, vel2=1250
                // fraction = (42-36)/(48-36) = 0.5
                // velocity = 1150 + 0.5 * (1250-1150) = 1200 RPM
                double fraction = (distance - dist1) / (dist2 - dist1);
                return vel1 + fraction * (vel2 - vel1);
            }
        }

        // Fallback (should never reach here)
        return 1250;
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Normalize angle to range [-PI, PI]
     * This ensures we always take the shortest rotation path
     * Example: if angle is 350°, normalize to -10° (rotate 10° clockwise, not 350° counter-clockwise)
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ==================== GETTERS ====================

    /** @return True if robot is aimed at goal (within ALIGNMENT_TOLERANCE_DEGREES) */
    public boolean isAligned() { return isAligned; }

    /** @return Distance from robot to goal in inches */
    public double getDistanceToGoal() { return distanceToGoal; }

    /** @return Angle error in degrees (positive = need to rotate CCW) */
    public double getAngleErrorDegrees() { return Math.toDegrees(angleError); }

    /** @return Recommended launcher velocity in RPM based on distance */
    public double getRecommendedVelocity() { return recommendedVelocity; }

    /** @return True if robot is within the designated launch zone */
    public boolean isInLaunchZone() { return FieldPositions.isInLaunchZone(follower.getPose()); }

    /** @return Last calculated rotation power (for telemetry) */
    public double getLastRotationPower() { return lastRotationPower; }

    /** @return Target angle in degrees (direction to goal) */
    public double getTargetAngleDegrees() { return Math.toDegrees(angleToGoal); }

    /** @return Current robot heading in degrees */
    public double getCurrentHeadingDegrees() { return Math.toDegrees(follower.getPose().getHeading()); }

    // ==================== SETTERS ====================

    /** Set alliance (call if alliance changes or was set wrong) */
    public void setAlliance(FieldPositions.Alliance alliance) { this.currentAlliance = alliance; }

    /** Reset PD controller state (call when enabling/disabling auto-align) */
    public void resetPID() {
        integralSum = 0;
        previousError = 0;
        lastUpdateTime = System.currentTimeMillis();
    }
}
