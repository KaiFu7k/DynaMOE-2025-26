package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * RobotState - Persists data between OpModes
 *
 * PURPOSE:
 * Static variables survive between OpMode transitions (Auton -> TeleOp).
 * This allows TeleOp to know where the robot ended up after Autonomous.
 *
 * USAGE:
 * At the end of Autonomous:
 *   RobotState.saveAutonEndPose(follower.getPose());
 *
 * At the start of TeleOp:
 *   Pose startPose = RobotState.getAutonEndPose();
 *   if (startPose != null) {
 *       follower.setStartingPose(startPose);
 *   }
 *
 * NOTE: Static variables are cleared when the app is restarted.
 * If you restart the Robot Controller app between Auton and TeleOp,
 * the pose will be lost.
 */
public class RobotState {

    // Last known pose from Autonomous
    private static Pose autonEndPose = null;

    // Timestamp when pose was saved (for staleness checking)
    private static long autonEndTime = 0;

    // Alliance used in Auton (so TeleOp can inherit it)
    private static FieldPositions.Alliance lastAlliance = null;

    /**
     * Save the robot's pose at the end of Autonomous
     * Call this at the very end of your Auton OpMode
     *
     * @param pose The robot's final pose from follower.getPose()
     */
    public static void saveAutonEndPose(Pose pose) {
        autonEndPose = pose;
        autonEndTime = System.currentTimeMillis();
    }

    /**
     * Save the robot's pose and alliance at the end of Autonomous
     *
     * @param pose The robot's final pose
     * @param alliance The alliance used in Auton
     */
    public static void saveAutonEndPose(Pose pose, FieldPositions.Alliance alliance) {
        autonEndPose = pose;
        autonEndTime = System.currentTimeMillis();
        lastAlliance = alliance;
    }

    /**
     * Get the pose saved from Autonomous
     *
     * @return The saved pose, or null if no pose was saved
     */
    public static Pose getAutonEndPose() {
        return autonEndPose;
    }

    /**
     * Get the alliance used in Autonomous
     *
     * @return The saved alliance, or null if not saved
     */
    public static FieldPositions.Alliance getLastAlliance() {
        return lastAlliance;
    }

    /**
     * Check if we have a valid pose from a recent Auton run
     * Considers pose "stale" if more than 5 minutes old
     *
     * @return true if we have a recent, valid pose
     */
    public static boolean hasValidAutonPose() {
        if (autonEndPose == null) return false;

        // Check if pose is less than 5 minutes old
        long ageMs = System.currentTimeMillis() - autonEndTime;
        long fiveMinutesMs = 5 * 60 * 1000;

        return ageMs < fiveMinutesMs;
    }

    /**
     * Get the age of the saved pose in seconds
     *
     * @return Age in seconds, or -1 if no pose saved
     */
    public static double getPoseAgeSeconds() {
        if (autonEndPose == null) return -1;
        return (System.currentTimeMillis() - autonEndTime) / 1000.0;
    }

    /**
     * Clear all saved state
     * Call this if you want to force TeleOp to use manual position selection
     */
    public static void clear() {
        autonEndPose = null;
        autonEndTime = 0;
        lastAlliance = null;
    }
}
