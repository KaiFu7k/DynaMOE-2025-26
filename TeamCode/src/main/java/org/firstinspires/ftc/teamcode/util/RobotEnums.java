package org.firstinspires.ftc.teamcode.util;

/**
 * Shared enums used across multiple subsystems
 * Prevents duplication and type conflicts
 */
public class RobotEnums {

    /**
     * Which launcher side to use
     */
    public enum LauncherSide {
        LEFT,
        RIGHT,
        BOTH
    }

    /**
     * Artifact colors for DECODE game
     */
    public enum ArtifactColor {
        PURPLE,
        GREEN
    }

    /**
     * MOTIF patterns displayed on OBELISK
     * Determines the order artifacts must be launched
     */
    public enum Motif {
        GPP,  // Green, Purple, Purple (launch order)
        PGP,  // Purple, Green, Purple
        PPG   // Purple, Purple, Green
    }
}
