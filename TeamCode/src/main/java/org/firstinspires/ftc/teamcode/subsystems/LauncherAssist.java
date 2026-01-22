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
 * - Provides rotation commands for auto-alignment
 *
 * USAGE:
 * 1. Call update() in main loop to refresh calculations
 * 2. Call getRotationPower() to get motor powers for alignment
 * 3. Call isAligned() to check if robot is facing goal
 * 4. Call getRecommendedVelocity() to get launch speed for current distance
 */
public class LauncherAssist {

    // Constants
    private static final double ALIGNMENT_TOLERANCE_DEGREES = 3.0;  // How close is "aligned"
    private static final double ROTATION_SPEED = 0.3;  // Speed for auto-rotation (0.0 to 1.0)

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
        // Normalize to -PI to +PI range
        angleError = normalizeAngle(angleToGoal - currentPose.getHeading());

        // Check if aligned (within tolerance)
        isAligned = Math.abs(Math.toDegrees(angleError)) < ALIGNMENT_TOLERANCE_DEGREES;

        // Calculate recommended velocity based on distance
        recommendedVelocity = calculateVelocity(distanceToGoal);
    }

    /**
     * Get rotation power for auto-alignment
     * Use this to rotate the robot to face the goal
     *
     * @return Rotation power for right stick X (-1.0 to 1.0)
     *         Positive = rotate counterclockwise
     *         Negative = rotate clockwise
     *         0 = aligned or not active
     */
    public double getRotationPower() {
        if (isAligned) {
            return 0.0;  // Already aligned, no rotation needed
        }

        // Proportional control: rotate faster when further from target
        double rotationPower = angleError * 2.0;  // Scale factor

        // Clamp to maximum rotation speed
        if (rotationPower > ROTATION_SPEED) {
            rotationPower = ROTATION_SPEED;
        } else if (rotationPower < -ROTATION_SPEED) {
            rotationPower = -ROTATION_SPEED;
        }

        // Add minimum power to overcome friction (only if not aligned)
        if (Math.abs(rotationPower) > 0 && Math.abs(rotationPower) < 0.1) {
            rotationPower = Math.signum(rotationPower) * 0.1;
        }

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
     * Update telemetry with launcher assist status
     */
    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== LAUNCHER ASSIST ===");
        telemetry.addData("Distance", "%.1f in", distanceToGoal);
        telemetry.addData("Angle Error", "%.1f°", Math.toDegrees(angleError));
        telemetry.addData("Aligned", isAligned ? "YES" : "NO");
        telemetry.addData("Recommended RPM", "%.0f", recommendedVelocity);
        telemetry.addData("In Launch Zone", isInLaunchZone() ? "YES" : "NO");
    }
}
