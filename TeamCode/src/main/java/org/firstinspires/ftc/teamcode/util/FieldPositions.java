package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Field positions and coordinates for DECODE 2025-26.
 * Origin (0,0) at bottom-left. X: 0-144 (left-right), Y: 0-144 (bottom-top).
 * Heading: 0°=right, 90°=up, 180°=left, 270°=down.
 */
public class FieldPositions {

    // Starting positions
    public static final Pose BLUE_GOAL_SIDE_START = new Pose(36, 132, Math.toRadians(90));
    public static final Pose RED_GOAL_SIDE_START = new Pose(108, 132, Math.toRadians(90));
    public static final Pose BLUE_PERIMETER_START = new Pose(48, 9, Math.toRadians(90));
    public static final Pose RED_PERIMETER_START = new Pose(96, 9, Math.toRadians(90));

    // Launch positions
    public static final Pose BLUE_LAUNCH_POSE = new Pose(48, 96, Math.toRadians(135));
    public static final Pose BLUE_PERIMETER_LAUNCH_POSE = new Pose(48, 15, Math.toRadians(110));
    public static final Pose RED_LAUNCH_POSE = new Pose(96, 96, Math.toRadians(45));
    public static final Pose RED_PERIMETER_LAUNCH_POSE = new Pose(96, 15, Math.toRadians(70));

    // Leave positions
    public static final Pose BLUE_LEAVE_POSE = new Pose(48, 58, Math.toRadians(180));
    public static final Pose RED_LEAVE_POSE = new Pose(96, 58, Math.toRadians(0));

    // Spike positions
    // Blue Spikes
    public static final Pose BLUE_SPIKE_TOP = new Pose(15, 84, Math.toRadians(180));
    public static final Pose BLUE_SPIKE_MIDDLE = new Pose(15, 58, Math.toRadians(180));
    public static final Pose BLUE_SPIKE_BOTTOM = new Pose(15, 36, Math.toRadians(180));

    // Red Spikes
    public static final Pose RED_SPIKE_TOP = new Pose(129, 84, Math.toRadians(0));
    public static final Pose RED_SPIKE_MIDDLE = new Pose(129, 58, Math.toRadians(0));
    public static final Pose RED_SPIKE_BOTTOM = new Pose(129, 36, Math.toRadians(0));

    // Park positions
    public static final Pose BLUE_PARK_POSE = new Pose(48, 36, Math.toRadians(180));
    public static final Pose RED_PARK_POSE = new Pose(96, 36, Math.toRadians(0));

    // Goal positions (for auto-alignment)
    public static final Pose BLUE_GOAL_POSITION = new Pose(0, 144, Math.toRadians(135));
    public static final Pose RED_GOAL_POSITION = new Pose(144, 144, Math.toRadians(45));

    // Goal dimensions
    public static final double GOAL_HEIGHT_CENTER_INCHES = 46.25;

    // Launch zone boundaries (rectangular approximation)
    private static final double LAUNCH_ZONE_MIN_X = 32;
    private static final double LAUNCH_ZONE_MAX_X = 112;
    private static final double LAUNCH_ZONE_MIN_Y = 72;
    private static final double LAUNCH_ZONE_MAX_Y = 120;

    public enum StartPosition {
        BLUE_GOAL_SIDE, BLUE_PERIMETER_SIDE, RED_GOAL_SIDE, RED_PERIMETER_SIDE
    }

    public enum Alliance { RED, BLUE }

    public static Pose getStartPose(StartPosition pos) {
        switch (pos) {
            case BLUE_GOAL_SIDE: return BLUE_GOAL_SIDE_START;
            case BLUE_PERIMETER_SIDE: return BLUE_PERIMETER_START;
            case RED_GOAL_SIDE: return RED_GOAL_SIDE_START;
            case RED_PERIMETER_SIDE: return RED_PERIMETER_START;
            default: return BLUE_GOAL_SIDE_START;
        }
    }

    public static Pose getPerimeterStartPose(Alliance a) { return a == Alliance.BLUE ? BLUE_PERIMETER_START : RED_PERIMETER_START; }
    public static Pose getLaunchPose(Alliance a) { return a == Alliance.BLUE ? BLUE_LAUNCH_POSE : RED_LAUNCH_POSE; }
    public static Pose getPerimeterLaunchPose(Alliance a) { return a == Alliance.BLUE ? BLUE_PERIMETER_LAUNCH_POSE : RED_PERIMETER_LAUNCH_POSE; }
    public static Pose getLeavePose(Alliance a) { return a == Alliance.BLUE ? BLUE_LEAVE_POSE : RED_LEAVE_POSE; }
    public static Pose getSpikeTopPose(Alliance a) { return a == Alliance.BLUE ? BLUE_SPIKE_TOP : RED_SPIKE_TOP; }
    public static Pose getSpikeMiddlePose(Alliance a) { return a == Alliance.BLUE ? BLUE_SPIKE_MIDDLE : RED_SPIKE_MIDDLE; }
    public static Pose getSpikeBottomPose(Alliance a) { return a == Alliance.BLUE ? BLUE_SPIKE_BOTTOM : RED_SPIKE_BOTTOM; }
    public static Pose getParkPose(Alliance a) { return a == Alliance.BLUE ? BLUE_PARK_POSE : RED_PARK_POSE; }
    public static Pose getGoalPosition(Alliance a) { return a == Alliance.BLUE ? BLUE_GOAL_POSITION : RED_GOAL_POSITION; }

    public static Alliance getAlliance(StartPosition pos) {
        return (pos == StartPosition.BLUE_GOAL_SIDE || pos == StartPosition.BLUE_PERIMETER_SIDE) ? Alliance.BLUE : Alliance.RED;
    }

    public static boolean isGoalSide(StartPosition pos) {
        return pos == StartPosition.BLUE_GOAL_SIDE || pos == StartPosition.RED_GOAL_SIDE;
    }

    public static boolean isPerimeterSide(StartPosition pos) {
        return pos == StartPosition.BLUE_PERIMETER_SIDE || pos == StartPosition.RED_PERIMETER_SIDE;
    }

    public static boolean isInLaunchZone(Pose pose) {
        return pose.getX() >= LAUNCH_ZONE_MIN_X && pose.getX() <= LAUNCH_ZONE_MAX_X &&
               pose.getY() >= LAUNCH_ZONE_MIN_Y && pose.getY() <= LAUNCH_ZONE_MAX_Y;
    }

    public static double getDistanceToGoal(Pose pose, Alliance alliance) {
        Pose goal = getGoalPosition(alliance);
        double dx = goal.getX() - pose.getX();
        double dy = goal.getY() - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
