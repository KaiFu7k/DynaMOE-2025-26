package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Centralized storage for all field positions and coordinates
 * All coordinates in inches, angles in radians
 * Origin (0,0) is center of field, positive X = right, positive Y = away from driver
 *
 * IMPORTANT: Tune these values for your specific field!
 */
public class FieldPositions {

    // ==================== STARTING POSITIONS ====================
    // Where robot begins autonomous

    public static final Pose BLUE_GOAL_SIDE_START = new Pose(-36, 60, Math.toRadians(270));
    public static final Pose RED_GOAL_SIDE_START = new Pose(36, 60, Math.toRadians(270));
    public static final Pose BLUE_PERIMETER_START = new Pose(-60, 36, Math.toRadians(0));
    public static final Pose RED_PERIMETER_START = new Pose(60, 36, Math.toRadians(180));

    // ==================== LAUNCH POSITIONS ====================
    // Where robot shoots artifacts (must be inside LAUNCH ZONE)

    public static final Pose BLUE_LAUNCH_POSE = new Pose(-24, 48, Math.toRadians(315));
    public static final Pose RED_LAUNCH_POSE = new Pose(24, 48, Math.toRadians(45));

    // ==================== LEAVE POSITIONS ====================
    // Final position to score LEAVE points (must be off LAUNCH LINE)

    public static final Pose BLUE_LEAVE_POSE = new Pose(-36, 24, Math.toRadians(0));
    public static final Pose RED_LEAVE_POSE = new Pose(36, 24, Math.toRadians(180));

    // ==================== GOAL POSITIONS ====================
    // Physical goal locations on field (for auto-alignment)
    // Field is 144" x 144", origin at center
    // Goals are in top corners (positive Y direction)

    public static final Pose BLUE_GOAL_POSITION = new Pose(-60, 60, 0);  // Top-left corner
    public static final Pose RED_GOAL_POSITION = new Pose(60, 60, 0);    // Top-right corner

    // Goal physical dimensions (from game manual)
    public static final double GOAL_HEIGHT_BOTTOM_INCHES = 38.75;  // Bottom of opening
    public static final double GOAL_HEIGHT_TOP_INCHES = 53.75;     // Top of opening (38.75 + 15)
    public static final double GOAL_HEIGHT_CENTER_INCHES = 46.25;  // Center target point
    public static final double GOAL_OPENING_WIDTH_INCHES = 26.5;   // Width of opening

    // ==================== LAUNCH ZONE BOUNDARIES ====================
    // Diamond-shaped launch zone in center of field
    // Approximate boundaries based on field diagram

    // Launch zone is roughly a diamond from center
    // These are approximate - tune based on actual field measurements
    private static final double LAUNCH_ZONE_MIN_X = -40;
    private static final double LAUNCH_ZONE_MAX_X = 40;
    private static final double LAUNCH_ZONE_MIN_Y = 20;
    private static final double LAUNCH_ZONE_MAX_Y = 65;

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
     * Get launch pose based on alliance
     */
    public static Pose getLaunchPose(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? BLUE_LAUNCH_POSE : RED_LAUNCH_POSE;
    }

    /**
     * Get leave pose based on alliance
     */
    public static Pose getLeavePose(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? BLUE_LEAVE_POSE : RED_LEAVE_POSE;
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
