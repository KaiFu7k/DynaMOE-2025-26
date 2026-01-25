package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Centralized storage for all field positions and coordinates
 * All coordinates in inches, angles in radians
 *
 * COORDINATE SYSTEM (2025-26 DECODE season):
 * - Origin (0, 0) is at BOTTOM-LEFT corner of field
 * - X: 0 to 144 inches (left to right)
 * - Y: 0 to 144 inches (bottom to top)
 * - Center of field: (72, 72)
 * - Heading: 0° = right (+X), 90° = up (+Y), 180° = left (-X), 270° = down (-Y)
 *
 * FIELD LAYOUT:
 * - Blue goal: top-left corner (near 0, 144)
 * - Red goal: top-right corner (near 144, 144)
 * - Blue alliance starts: left/bottom side
 * - Red alliance starts: right/bottom side
 *
 * IMPORTANT: Tune these values for your specific field!
 */
public class FieldPositions {

    // ==================== STARTING POSITIONS ====================
    // Where robot begins autonomous
    // Robots start near bottom of field, facing toward goals (up/+Y direction)

    // Blue Goal Side: Left side of field, near blue goal corner
    public static final Pose BLUE_GOAL_SIDE_START = new Pose(36, 132, Math.toRadians(90));
    // Red Goal Side: Right side of field, near red goal corner
    public static final Pose RED_GOAL_SIDE_START = new Pose(108, 132, Math.toRadians(90));
    // Blue Perimeter: Bottom edge of field, facing up
    public static final Pose BLUE_PERIMETER_START = new Pose(48, 9, Math.toRadians(90));
    // Red Perimeter: Bottom edge of field, facing up
    public static final Pose RED_PERIMETER_START = new Pose(96, 9, Math.toRadians(90));

    // ==================== LAUNCH POSITIONS ====================
    // Where robot shoots artifacts (must be inside LAUNCH ZONE)
    // Launch zone is diamond-shaped in center of field

    // Blue launch (goal side): Aim toward blue goal (top-left, heading ~135°)
    public static final Pose BLUE_LAUNCH_POSE = new Pose(48, 96, Math.toRadians(135));
    // Blue launch (perimeter side): Far shot position, aim toward blue goal (~110° calculated from atan2)
    public static final Pose BLUE_PERIMETER_LAUNCH_POSE = new Pose(48, 15, Math.toRadians(110));
    // Red launch (goal side): Aim toward red goal (top-right, heading ~45°)
    public static final Pose RED_LAUNCH_POSE = new Pose(96, 96, Math.toRadians(45));
    // Red launch (perimeter side): Far shot position, aim toward red goal (~70° calculated from atan2)
    public static final Pose RED_PERIMETER_LAUNCH_POSE = new Pose(96, 15, Math.toRadians(70));

    // ==================== LEAVE POSITIONS ====================
    // Final position to score LEAVE points (must be off LAUNCH LINE)
    // Blue faces left (180°), Red faces right (0°) - mirrored across center

    public static final Pose BLUE_LEAVE_POSE = new Pose(48, 58, Math.toRadians(180));
    public static final Pose RED_LEAVE_POSE = new Pose(96, 58, Math.toRadians(0));

    // ==================== SPIKE POSITIONS ====================
    // Positions of artifacts on the field
    // Mirrored: Red_X = 144 - Blue_X, heading flips 180° to 0°

    public static final Pose BLUE_SPIKE_MIDDLE = new Pose(15, 58, Math.toRadians(180));
    public static final Pose RED_SPIKE_MIDDLE = new Pose(129, 58, Math.toRadians(0));

    // ==================== PARK POSITIONS ====================
    // Final parking positions after scoring

    public static final Pose BLUE_PARK_POSE = new Pose(48, 36, Math.toRadians(180));
    public static final Pose RED_PARK_POSE = new Pose(96, 36, Math.toRadians(0));

    // ==================== GOAL POSITIONS ====================
    // Physical goal locations on field (for auto-alignment)
    // Field is 144" x 144", origin at bottom-left corner
    // Goals are in top corners

    public static final Pose BLUE_GOAL_POSITION = new Pose(0, 144, Math.toRadians(135));   // Top-left corner
    public static final Pose RED_GOAL_POSITION = new Pose(144, 144, Math.toRadians(45));   // Top-right corner

    // Goal physical dimensions (from game manual)
    public static final double GOAL_HEIGHT_BOTTOM_INCHES = 38.75;  // Bottom of opening
    public static final double GOAL_HEIGHT_TOP_INCHES = 53.75;     // Top of opening (38.75 + 15)
    public static final double GOAL_HEIGHT_CENTER_INCHES = 46.25;  // Center target point
    public static final double GOAL_OPENING_WIDTH_INCHES = 26.5;   // Width of opening

    // ==================== LAUNCH ZONE BOUNDARIES ====================
    // Diamond-shaped launch zone in center of field
    // Approximate boundaries based on field diagram

    // Launch zone is roughly a diamond centered at (72, 72)
    // These are approximate - tune based on actual field measurements
    private static final double LAUNCH_ZONE_MIN_X = 32;
    private static final double LAUNCH_ZONE_MAX_X = 112;
    private static final double LAUNCH_ZONE_MIN_Y = 72;
    private static final double LAUNCH_ZONE_MAX_Y = 120;

    // ==================== ENUMS ====================

    /**
     * Four possible starting positions (combines alliance + side)
     */
    public enum StartPosition {
        BLUE_GOAL_SIDE,       // Blue alliance, near the GOAL
        BLUE_PERIMETER_SIDE,  // Blue alliance, on perimeter LAUNCH LINE
        RED_GOAL_SIDE,        // Red alliance, near the GOAL
        RED_PERIMETER_SIDE    // Red alliance, on perimeter LAUNCH LINE
    }

    /**
     * Alliance color
     */
    public enum Alliance {
        RED,
        BLUE
    }

    // ==================== HELPER METHODS ====================

    /**
     * Get starting pose based on selected position
     */
    public static Pose getStartPose(StartPosition position) {
        switch (position) {
            case BLUE_GOAL_SIDE:
                return BLUE_GOAL_SIDE_START;
            case BLUE_PERIMETER_SIDE:
                return BLUE_PERIMETER_START;
            case RED_GOAL_SIDE:
                return RED_GOAL_SIDE_START;
            case RED_PERIMETER_SIDE:
                return RED_PERIMETER_START;
            default:
                return BLUE_GOAL_SIDE_START;
        }
    }

    /**
     * Get launch pose based on alliance (for goal side)
     */
    public static Pose getLaunchPose(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? BLUE_LAUNCH_POSE : RED_LAUNCH_POSE;
    }

    /**
     * Get perimeter launch pose based on alliance (for far shots)
     */
    public static Pose getPerimeterLaunchPose(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? BLUE_PERIMETER_LAUNCH_POSE : RED_PERIMETER_LAUNCH_POSE;
    }

    /**
     * Get leave pose based on alliance
     */
    public static Pose getLeavePose(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? BLUE_LEAVE_POSE : RED_LEAVE_POSE;
    }

    /**
     * Get spike pose based on alliance (middle spike)
     */
    public static Pose getSpikeMiddlePose(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? BLUE_SPIKE_MIDDLE : RED_SPIKE_MIDDLE;
    }

    /**
     * Get final parking pose based on alliance
     */
    public static Pose getParkPose(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? BLUE_PARK_POSE : RED_PARK_POSE;
    }

    /**
     * Get alliance from start position
     */
    public static Alliance getAlliance(StartPosition position) {
        return (position == StartPosition.BLUE_GOAL_SIDE || position == StartPosition.BLUE_PERIMETER_SIDE)
            ? Alliance.BLUE
            : Alliance.RED;
    }

    /**
     * Check if starting from goal side
     */
    public static boolean isGoalSide(StartPosition position) {
        return position == StartPosition.BLUE_GOAL_SIDE || position == StartPosition.RED_GOAL_SIDE;
    }

    /**
     * Check if starting from perimeter side
     */
    public static boolean isPerimeterSide(StartPosition position) {
        return position == StartPosition.BLUE_PERIMETER_SIDE || position == StartPosition.RED_PERIMETER_SIDE;
    }

    /**
     * Get goal position for alliance
     * @param alliance RED or BLUE
     * @return Goal pose for that alliance
     */
    public static Pose getGoalPosition(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? BLUE_GOAL_POSITION : RED_GOAL_POSITION;
    }

    /**
     * Check if robot position is inside launch zone
     * @param pose Current robot pose
     * @return True if inside launch zone boundaries
     */
    public static boolean isInLaunchZone(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();

        // Simple rectangular boundary check
        // TODO: Refine to actual diamond shape if needed
        return x >= LAUNCH_ZONE_MIN_X && x <= LAUNCH_ZONE_MAX_X &&
               y >= LAUNCH_ZONE_MIN_Y && y <= LAUNCH_ZONE_MAX_Y;
    }

    /**
     * Get distance from pose to goal
     * @param pose Current robot pose
     * @param alliance Current alliance
     * @return Distance in inches
     */
    public static double getDistanceToGoal(Pose pose, Alliance alliance) {
        Pose goalPose = getGoalPosition(alliance);
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
