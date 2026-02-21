package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FieldPositions;
import org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;

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
@Config
public class LauncherAssist {

    // ==================== ALIGNMENT CONSTANTS ====================

    /** How close (in degrees) the robot needs to be to consider itself "aligned" */
    private static final double ALIGNMENT_TOLERANCE_DEGREES = 4.0;

    /** Maximum rotation speed (0 to 1) - limits how fast robot can spin during alignment */
    private static final double ROTATION_SPEED = 1;

    /** Minimum power to overcome static friction - robot won't move below this */
    private static final double MIN_ROTATION_POWER = 0.18;

    // ==================== LAUNCHER OFFSET ====================
    /** Distance (inches) from robot center to each launcher, perpendicular to heading */
    private static final double LAUNCHER_OFFSET_INCHES = 3.1875;

    // ==================== DUAL PIDF CONSTANTS ====================
    // Uses Pedro's PIDFController with piecewise switching:
    //   - Primary PIDF: used when |heading error| > HEADING_PIDF_SWITCH (aggressive)
    //   - Secondary PIDF: used when |heading error| <= HEADING_PIDF_SWITCH (fine-tuning)

    /** Threshold (radians) for switching between primary and secondary PIDF */
    private static final double HEADING_PIDF_SWITCH = Math.PI / 20;  // 9 degrees

    // Primary PIDF coefficients (large error - aggressive correction)
    public static double PRIMARY_KP = 3.0;
    public static double PRIMARY_KI = 0;
    public static double PRIMARY_KD = 0.05;
    public static double PRIMARY_KF = 0.025;

    // Secondary PIDF coefficients (small error - fine-tuning)
    public static double SECONDARY_KP = 1.0;
    public static double SECONDARY_KI = 0;
    public static double SECONDARY_KD = 0.02;
    public static double SECONDARY_KF = 0.025;

    // ==================== VELOCITY TABLE ====================
    /**
     * VELOCITY LOOKUP TABLE - Maps distance (inches) to launcher velocity (RPM)
     *
     * FORMAT: {distance_in_inches, velocity_in_RPM}
     *
     * TUNED VALUES from testing on 2026-02-03
     * The code uses Linear Interpolation (LERP) between these points.
     *
     * Re-tuned 2026-02-14 with new field test data.
     * Close range (36-60 in) added, 100+ inch range corrected upward significantly.
     */
    private static final double[][] VELOCITY_TABLE = {
            {36, 1040},       // Very close (114.9, 100.6)
            {51, 1110},       // Close (81.6, 129.9)
            {55, 1110},       // Close (80.7, 114.7)
            {58, 1120},       // Close-mid (93.3, 89.2)
            {60, 1070},       // Close-mid (81.3, 100.3)
            {80, 1180},       // Mid (78.6, 73.5)
            {88, 1220},       // Mid (76.5, 64.5)
            {107, 1300},      // Mid-far (96.4, 31.8)
            {124, 1443},      // Far — avg of 1480, 1400, 1450 at 123-129 in
            {155, 1420},      // Original far
            {163, 1490},      // Original far
            {170, 1480}       // Max distance tested
    };

    // ==================== STATE VARIABLES ====================

    private Follower follower;           // Pedro Pathing follower for odometry
    private Telemetry telemetry;         // For debug output
    private FieldPositions.Alliance currentAlliance;  // BLUE or RED (determines goal position)

    private double distanceToGoal = 0;   // Calculated distance in inches
    private double angleToGoal = 0;      // Angle FROM launcher TO goal (radians)
    private double angleError = 0;       // Difference between current heading and target (radians)
    private double recommendedVelocity = 0;  // Calculated launcher RPM
    private boolean isAligned = false;   // True when robot is aimed at goal (within tolerance)
    private LauncherSide activeSide = LauncherSide.LEFT;  // Which launcher we're currently aligning for

    // Dual PIDF Controllers (two separate instances, like Pedro's Follower)
    private PIDFController headingPIDF;           // Primary: large errors
    private PIDFController secondaryHeadingPIDF;  // Secondary: small errors (fine-tuning)
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
        buildHeadingPIDF();
    }

    /**
     * Builds the two separate heading PIDFControllers (matching Pedro's Follower approach).
     * Each controller has its own independent integral/derivative state.
     * Primary is used for large errors, secondary for fine-tuning near target.
     */
    private void buildHeadingPIDF() {
        headingPIDF = new PIDFController(
                new PIDFCoefficients(PRIMARY_KP, PRIMARY_KI, PRIMARY_KD, PRIMARY_KF)
        );
        secondaryHeadingPIDF = new PIDFController(
                new PIDFCoefficients(SECONDARY_KP, SECONDARY_KI, SECONDARY_KD, SECONDARY_KF)
        );
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
        double heading = currentPose.getHeading();

        // Offset the origin point from robot center to the active launcher.
        // "Left" of the robot's heading = +90° from heading (perpendicular left).
        // LEFT launcher is to the left of center → offset in the +perpendicular direction.
        // RIGHT launcher is to the right → offset in the -perpendicular direction.
        double launcherX = currentPose.getX();
        double launcherY = currentPose.getY();

        if (activeSide == LauncherSide.LEFT) {
            // Perpendicular left = heading + PI/2
            launcherX += LAUNCHER_OFFSET_INCHES * Math.cos(heading + Math.PI / 2);
            launcherY += LAUNCHER_OFFSET_INCHES * Math.sin(heading + Math.PI / 2);
        } else if (activeSide == LauncherSide.RIGHT) {
            // Perpendicular right = heading - PI/2
            launcherX += LAUNCHER_OFFSET_INCHES * Math.cos(heading - Math.PI / 2);
            launcherY += LAUNCHER_OFFSET_INCHES * Math.sin(heading - Math.PI / 2);
        }
        // LauncherSide.BOTH → no offset, use robot center

        // Get goal position based on alliance (Blue goal or Red goal)
        Pose goalPose = FieldPositions.getGoalPosition(currentAlliance);

        // Calculate vector from launcher position to goal
        double dx = goalPose.getX() - launcherX;
        double dy = goalPose.getY() - launcherY;

        // Distance from launcher to goal
        distanceToGoal = Math.sqrt(dx * dx + dy * dy);

        // Angle from launcher to goal
        angleToGoal = Math.atan2(dy, dx);

        // Error = how far we need to rotate so the launcher points at goal
        angleError = normalizeAngle(angleToGoal - heading);

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
        // Turn direction: +1 for CCW, -1 for CW (used as feedforward input)
        double turnDirection = (angleError > 0) ? 1 : -1;

        double pidfOutput;

        // Switch between controllers based on error magnitude (same as Pedro's Follower)
        if (Math.abs(angleError) < HEADING_PIDF_SWITCH) {
            // Small error: use secondary (fine-tuning) controller
            secondaryHeadingPIDF.updateFeedForwardInput(turnDirection);
            secondaryHeadingPIDF.updateError(angleError);
            pidfOutput = secondaryHeadingPIDF.run();
        } else {
            // Large error: use primary (aggressive) controller
            headingPIDF.updateFeedForwardInput(turnDirection);
            headingPIDF.updateError(angleError);
            pidfOutput = headingPIDF.run();
        }

        // Positive error (need CCW) → positive PIDF output → positive rx
        // In mecanum math: +rx → left side faster, right side slower → CCW rotation ✓
        double rotationPower = -pidfOutput;

        // Clamp to max rotation speed
        rotationPower = Math.max(-ROTATION_SPEED, Math.min(ROTATION_SPEED, rotationPower));

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
    public static double calculateVelocity(double distance) {
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
     * Compute the robot heading (radians) needed so that a specific launcher side
     * at the given robot center position aims at the goal.
     *
     * Uses one iteration: compute the launcher offset position assuming the nominal
     * center-aimed heading, then recalculate the heading from that offset position.
     * At typical distances (50-170 in) with a 3.1875" offset, the error is negligible.
     *
     * @param robotX   Robot center X position
     * @param robotY   Robot center Y position
     * @param goalX    Goal X position
     * @param goalY    Goal Y position
     * @param side     Which launcher side (LEFT, RIGHT, or BOTH for center)
     * @return Heading in radians that aims the specified launcher at the goal
     */
    public static double getHeadingForSide(double robotX, double robotY,
                                           double goalX, double goalY, LauncherSide side) {
        // Start with the center-aimed heading
        double nominalHeading = Math.atan2(goalY - robotY, goalX - robotX);

        if (side == LauncherSide.BOTH) return nominalHeading;

        // Compute launcher position at that nominal heading
        double perpAngle = (side == LauncherSide.LEFT)
                ? nominalHeading + Math.PI / 2
                : nominalHeading - Math.PI / 2;
        double launcherX = robotX + LAUNCHER_OFFSET_INCHES * Math.cos(perpAngle);
        double launcherY = robotY + LAUNCHER_OFFSET_INCHES * Math.sin(perpAngle);

        // Recompute heading from the offset launcher position to the goal
        return Math.atan2(goalY - launcherY, goalX - launcherX);
    }

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

    /** Set which launcher side we're aligning for (offsets aim point from robot center) */
    public void setActiveSide(LauncherSide side) { this.activeSide = side; }

    /** Reset both PIDF controllers' state (call when enabling/disabling auto-align) */
    public void resetPID() {
        headingPIDF.reset();
        secondaryHeadingPIDF.reset();
    }
}