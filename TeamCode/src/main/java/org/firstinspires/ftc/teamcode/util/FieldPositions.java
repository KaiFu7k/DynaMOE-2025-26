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
}
